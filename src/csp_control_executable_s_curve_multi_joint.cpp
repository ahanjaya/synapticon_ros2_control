#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>

#include "ethercat.h"

#define EC_TIMEOUTMON 500
#define M_PI 3.14159

// Enum for state machine
enum StateMachine {
    INIT = 0,
    PROFILE_TORQUE_MODE,
    FAULT_RESET,
    READY_TO_SWITCH_ON,
    SWITCH_ON,
    CSP_MODE,
    OPERATION_ENABLED,
    DELAY,
    NORMAL_OPERATION,
    GENERATE_TARGET_POS,
    FINISH,
};

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
StateMachine state = PROFILE_TORQUE_MODE;

/* define pointer structure */
#pragma pack(1)
typedef struct {
    uint16_t Statusword;
    int8_t OpModeDisplay;
    int32_t PositionValue;
    int32_t VelocityValue;
    int16_t TorqueValue;
    uint16_t AnalogInput1;
    uint16_t AnalogInput2;
    uint16_t AnalogInput3;
    uint16_t AnalogInput4;
    uint32_t TuningStatus;
    uint32_t DigitalInputs;
    uint32_t UserMISO;
    uint32_t Timestamp;
    int32_t PositionDemandInternalValue;
    int32_t VelocityDemandValue;
    int16_t TorqueDemand;
} in_somanet_50t;

typedef struct {
    uint16_t Controlword;
    int8_t OpMode;
    int16_t TargetTorque;
    int32_t TargetPosition;
    int32_t TargetVelocity;
    int16_t TorqueOffset;
    int32_t TuningCommand;
    int32_t PhysicalOutputs;
    int32_t BitMask;
    int32_t UserMOSI;
    int32_t VelocityOffset;
} out_somanet_50t;
#pragma pack()

// Add before the cyclic loop (inside simpletest), after variable declarations:
struct SCurveProfile {
    int32_t start_pos;
    int32_t end_pos;
    double duration;  // seconds
};

double clamp(double v, double min_v, double max_v) { return std::max(min_v, std::min(v, max_v)); }

// Simple S-curve position generator (smoothstep)
double s_curve_position(const SCurveProfile &prof, double t) {
    double T = prof.duration;
    double s = clamp(t / T, 0.0, 1.0);
    double smooth = s * s * (3 - 2 * s);
    return prof.start_pos + (prof.end_pos - prof.start_pos) * smooth;
}

double encoder_to_rad(int32_t encoder_value, double mechanical_reduction, uint32_t encoder_resolution) {
    return (1 / mechanical_reduction) * encoder_value * 2 * 3.14159 / encoder_resolution;
}

double rad_to_encoder(double rad_value, double mechanical_reduction, uint32_t encoder_resolution) {
    return (1 / mechanical_reduction) * rad_value * encoder_resolution / (2 * 3.14159);
}

void simpletest(const char *ifname) {
    int i, chk;
    needlf = false;
    inOP = false;
    const uint8 num_joints = 6;  // Number of joints (1-6)

    std::vector<bool> is_fault_reset(num_joints, false);
    std::vector<bool> is_ready_switch_on(num_joints, false);
    std::vector<bool> is_switched_on(num_joints, false);
    std::vector<bool> is_current_target_equal(num_joints, false);
    std::vector<bool> is_operation_enabled(num_joints, false);
    std::vector<uint32_t> encoder_resolutions(num_joints, 0);
    std::vector<uint32_t> mechanical_reductions(num_joints, 1.0);
    std::vector<double> targets_pos_rad(num_joints, 0.0);

    SCurveProfile prof[num_joints];
    bool prof_active = false;
    int prof_cycle = 0;
    bool move_home = true;
    int delay_counter = 0;
    int num_of_cycles = 0;

    double t, required_time, min_duration_per_joint;
    int32_t s_curve_target_pos;

    constexpr char EXPECTED_SLAVE_NAME[] = "SOMANET";
    const int usleep_cycle_time = 1000;
    const double cycle_time = static_cast<double>(usleep_cycle_time) / 1000000.0;  // seconds
    const int delay_cycles = static_cast<int>(2.0 / cycle_time);                   // 2 seconds delay
    // Define max speed for each joint (counts/sec)
    const double max_speed[num_joints] = {436907, 436907, 436907, 218453, 218453, 218453};  // index 0 unused

    std::cout << std::fixed << std::setprecision(6);
    printf("Starting simple test\n");

    // initialise SOEM, bind socket to ifname
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n", ifname);
        // find and auto-config slaves

        if (ec_config_init(false) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);
            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");

            // wait for all slaves to reach SAFE_OP state
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
                   ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
            printf("Request operational state for all slaves\n");

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            // send one valid process data to make outputs in slaves happy
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            // request OP state for all slaves
            ec_writestate(0);
            chk = 200;

            // wait for all slaves to reach OP state */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            // Read encoder resolution for each joint
            for (size_t joint_idx = 1; joint_idx < (num_joints + 1); ++joint_idx) {
                // Verify slave name
                if (strcmp(ec_slave[joint_idx].name, EXPECTED_SLAVE_NAME) != 0) {
                    printf("Expected slave %s at position %zu, but got %s instead", EXPECTED_SLAVE_NAME, joint_idx, ec_slave[joint_idx].name);
                    // return ERROR;
                }

                uint8_t encoder_source;
                int size = sizeof(encoder_source);
                ec_SDOread(joint_idx, 0x2012, 0x09, false, &size, &encoder_source, EC_TIMEOUTRXM);
                size = sizeof(encoder_resolutions[joint_idx - 1]);
                if (encoder_source == 1) {
                    ec_SDOread(joint_idx, 0x2110, 0x03, false, &size, &encoder_resolutions[joint_idx - 1], EC_TIMEOUTRXM);
                } else if (encoder_source == 2) {
                    ec_SDOread(joint_idx, 0x2112, 0x03, false, &size, &encoder_resolutions[joint_idx - 1], EC_TIMEOUTRXM);
                } else {
                    printf("No encoder configured for position control on joint %zu. Terminating the program", joint_idx);
                    // return ERROR;
                }
                std::cout << "J" << joint_idx << " | Encoder Resolution: " << encoder_resolutions[joint_idx - 1] << std::endl;
            }

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");
                inOP = true;

                // create and connect struture pointers to I/O for 6 joints
                in_somanet_50t *in_somanet[7] = {nullptr};
                out_somanet_50t *out_somanet[7] = {nullptr};
                for (int s = 1; s <= 6; ++s) {
                    in_somanet[s] = (in_somanet_50t *)ec_slave[s].inputs;
                    out_somanet[s] = (out_somanet_50t *)ec_slave[s].outputs;
                }

                // cyclic loop
                while (1) {
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC) {
                        // state machine
                        switch (state) {
                            case INIT:
                                break;

                            case PROFILE_TORQUE_MODE:
                                // Set all joints to PROFILE_TORQUE_MODE
                                for (int s = 1; s <= 6; ++s) {
                                    out_somanet[s]->OpMode = PROFILE_TORQUE_MODE;
                                    out_somanet[s]->TorqueOffset = 0;
                                    out_somanet[s]->TargetTorque = 0;
                                }
                                state = FAULT_RESET;  // Move to next state
                                break;

                            case FAULT_RESET:
                                for (int s = 1; s <= 6; ++s) {
                                    // Fault reset: Fault -> Switch on disabled,
                                    // if the drive is in fault state
                                    if ((in_somanet[s]->Statusword & 0b0000000001001111) == 0b0000000000001000) {
                                        std::cout << "Fault reset for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b10000000;
                                        is_fault_reset.at(s - 1) = false;
                                    } else {
                                        is_fault_reset.at(s - 1) = true;
                                    }
                                }

                                // check if all is_fault_reset true then move to next state
                                if (std::all_of(is_fault_reset.begin(), is_fault_reset.end(), [](bool v) { return v; })) {
                                    std::cout << "All joints fault reset." << std::endl;
                                    state = READY_TO_SWITCH_ON;  // Move to next state
                                    std::fill(is_fault_reset.begin(), is_fault_reset.end(), false);
                                    break;
                                }
                                break;

                            case READY_TO_SWITCH_ON:
                                for (int s = 1; s <= 6; ++s) {
                                    // Shutdown: Switch on disabled -> Ready to switch on
                                    if ((in_somanet[s]->Statusword & 0b0000000001001111) == 0b0000000001000000) {
                                        // std::cout << "Ready to switch on for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00000110;  // 6
                                        is_ready_switch_on.at(s - 1) = false;
                                    } else {
                                        is_ready_switch_on.at(s - 1) = true;
                                    }
                                }

                                // check if all is_ready_switch_on true then move to next state
                                if (std::all_of(is_ready_switch_on.begin(), is_ready_switch_on.end(), [](bool v) { return v; })) {
                                    std::cout << "All joints ready to switch on." << std::endl;
                                    state = SWITCH_ON;  // Move to next state
                                    std::fill(is_ready_switch_on.begin(), is_ready_switch_on.end(), false);
                                    break;
                                }
                                break;

                            case SWITCH_ON:
                                for (int s = 1; s <= 6; ++s) {
                                    // Switch on: Ready to switch on -> Switched on
                                    if ((in_somanet[s]->Statusword & 0b0000000001101111) == 0b0000000000100001) {
                                        // std::cout << "Switching on for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00000111;  // 7
                                        is_switched_on.at(s - 1) = false;
                                    } else {
                                        is_switched_on.at(s - 1) = true;
                                    }
                                }

                                // check if all is_switched_on true then move to next state
                                if (std::all_of(is_switched_on.begin(), is_switched_on.end(), [](bool v) { return v; })) {
                                    std::cout << "All joints switched on." << std::endl;
                                    state = CSP_MODE;  // Move to next state
                                    std::fill(is_switched_on.begin(), is_switched_on.end(), false);
                                    break;
                                }
                                break;

                            case CSP_MODE:
                                // Set Target Position with Current
                                // Position for Safety
                                for (int s = 1; s <= 6; ++s) {
                                    out_somanet[s]->TargetPosition = in_somanet[s]->PositionValue;

                                    std::cout << "J" << s << " | Current: " << in_somanet[s]->PositionValue
                                              << " | Target: " << out_somanet[s]->TargetPosition << std::endl;
                                    out_somanet[s]->OpMode = 8;  // Set to CSP mode

                                    if (out_somanet[s]->TargetPosition == in_somanet[s]->PositionValue) {
                                        is_current_target_equal.at(s - 1) = true;
                                    } else {
                                        is_current_target_equal.at(s - 1) = false;
                                    }
                                }
                                // check if all is_current_target_equal true then move to next state
                                if (std::all_of(is_current_target_equal.begin(), is_current_target_equal.end(), [](bool v) { return v; })) {
                                    std::cout << "All joints set to CSP mode with current position." << std::endl;
                                    state = OPERATION_ENABLED;  // Move to next state
                                    std::fill(is_current_target_equal.begin(), is_current_target_equal.end(), false);
                                    break;
                                }
                                break;

                            case OPERATION_ENABLED:
                                for (int s = 1; s <= 6; ++s) {
                                    // Enable operation: Switched on ->
                                    // Operation enabled
                                    if ((in_somanet[s]->Statusword & 0b0000000001101111) == 0b0000000000100011) {
                                        // std::cout << "Enable operation for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00001111;  // 15
                                        is_operation_enabled.at(s - 1) = false;
                                    } else {
                                        is_operation_enabled.at(s - 1) = true;
                                    }
                                }

                                // check if all is_switched_on true then move to next state
                                if (std::all_of(is_operation_enabled.begin(), is_operation_enabled.end(), [](bool v) { return v; })) {
                                    std::cout << "All joints enabled." << std::endl;
                                    state = DELAY;  // Move to next state
                                    std::fill(is_operation_enabled.begin(), is_operation_enabled.end(), false);
                                }

                                break;

                            case DELAY:
                                if (++delay_counter >= delay_cycles) {
                                    state = GENERATE_TARGET_POS;  // Move to next state
                                    delay_counter = 0;
                                }

                                break;

                            case GENERATE_TARGET_POS:
                                std::cout << std::string(50, '*') << std::endl;

                                if (move_home) {
                                    std::cout << "Moving all joints to home position..." << std::endl;
                                    std::fill(targets_pos_rad.begin(), targets_pos_rad.end(), 0.0);  // Reset target positions to 0 radians
                                } else {
                                    std::cout << "Moving all joints to random target positions..." << std::endl;

                                    // Generate a random target position for all joints
                                    for (int s = 1; s <= 6; ++s) {
                                        // Random target position between -90 and 90 degrees
                                        targets_pos_rad[s - 1] = ((double)rand() / RAND_MAX) * (M_PI / 2 * 2) - M_PI / 2;
                                    }
                                }

                                move_home = !move_home;  // Toggle move_home for next iteration

                                state = NORMAL_OPERATION;
                                break;

                            case NORMAL_OPERATION: {
                                if (!prof_active) {
                                    required_time = 0.0;

                                    for (int s = 1; s <= 6; ++s) {
                                        prof[s - 1].start_pos = in_somanet[s]->PositionValue;

                                        // Target position
                                        prof[s - 1].end_pos =
                                            rad_to_encoder(targets_pos_rad[s - 1], mechanical_reductions[s - 1], encoder_resolutions[s - 1]);

                                        min_duration_per_joint = std::abs(prof[s - 1].end_pos - prof[s - 1].start_pos) / max_speed[s - 1];

                                        std::cout << "J" << s << " | Start: " << prof[s - 1].start_pos << " | End: " << prof[s - 1].end_pos
                                                  << " | Required Time: " << min_duration_per_joint << std::endl;

                                        if (min_duration_per_joint > required_time) {
                                            required_time = min_duration_per_joint;
                                        }
                                    }
                                    required_time = std::max(required_time, 1.0);  // Ensure at least 1 second
                                    std::cout << "Min required time for all joints: " << required_time << " seconds." << std::endl;

                                    for (int s = 1; s <= 6; ++s) {
                                        prof[s - 1].duration = required_time;
                                    }

                                    num_of_cycles = static_cast<int>(required_time / cycle_time);
                                    std::cout << "Number of cycles: " << num_of_cycles << std::endl;

                                    prof_cycle = 0;
                                    prof_active = true;
                                }

                                // open loop S-Curve position control
                                t = prof_cycle * cycle_time;
                                for (int s = 1; s <= 6; ++s) {
                                    s_curve_target_pos = (int32_t)s_curve_position(prof[s - 1], t);
                                    out_somanet[s]->TargetPosition = s_curve_target_pos;
                                }
                                prof_cycle++;

                                if (prof_cycle > num_of_cycles) {
                                    prof_active = false;
                                    std::cout << "All joints reached target position." << std::endl;
                                    state = DELAY;  // Move to next state
                                }
                                break;
                            }

                            case FINISH:
                                break;

                            default:
                                std::cout << "Unknown state." << std::endl;
                                break;
                        }
                    }
                    osal_usleep(usleep_cycle_time);  // Sleep for 5 ms
                }
                inOP = false;
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf(
                            "Slave %d State=0x%2.2x StatusCode=0x%4.4x : "
                            "%s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            // request INIT state for all slaves
            ec_writestate(0);
        } else {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        // stop SOEM, close socket
        ec_close();
    } else {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    int slave;
    (void)ptr;  // unused

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = false;
                printf("\n");
            }
            // one ore more slaves are not responding
            ec_group[currentgroup].docheckstate = false;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = true;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf(
                            "ERROR : slave %d is in SAFE_OP + ERROR, "
                            "attempting ack.\n",
                            slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf(
                            "WARNING : slave %d is in SAFE_OP, change to "
                            "OPERATIONAL.\n",
                            slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = false;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        // re-check state
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = true;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = false;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = false;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[]) {
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1) {
        osal_thread_create(&thread1, 128000, (void *)&ecatcheck, (void *)&ctime);
        /* start cyclic part */
        simpletest(argv[1]);
    } else {
        ec_adaptert *adapter = NULL;
        printf("\nAvailable adapters:\n");
        adapter = ec_find_adapters();
        while (adapter != NULL) {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
        ec_free_adapters(adapter);
    }

    printf("End program\n");
    return (0);
}
