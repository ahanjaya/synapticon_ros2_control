#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

// Enum for state machine
enum StateMachine {
    INIT = 0,
    PROFILE_TORQUE_MODE,
    FAULT_RESET,
    READY_TO_SWITCH_ON,
    SWITCH_ON,
    CSP_MODE,
    OPERATION_ENABLED,
    NORMAL_OPERATION,
    QUICK_STOP,
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

void simpletest(const char *ifname) {
    int i, j, chk;
    needlf = false;
    inOP = false;
    std::vector<bool> is_fault_reset(6, false);
    std::vector<bool> is_ready_switch_on(6, false);
    std::vector<bool> is_switched_on(6, false);
    std::vector<bool> is_current_target_equal(6, false);
    std::vector<bool> is_operation_enabled(6, false);

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
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");
                inOP = true;

                j = 0;

                // create and connect struture pointers to I/O for 6 joints
                in_somanet_50t *in_somanet[7] = {nullptr};
                out_somanet_50t *out_somanet[7] = {nullptr};
                for (int s = 1; s <= 6; ++s) {
                    in_somanet[s] = (in_somanet_50t *)ec_slave[s].inputs;
                    out_somanet[s] = (out_somanet_50t *)ec_slave[s].outputs;
                }

                // cyclic loop
                // for (i = 1; i <= 10000; i++) {
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

                                    // check if all is_fault_reset true then
                                    // move to next state
                                    if (std::all_of(is_fault_reset.begin(), is_fault_reset.end(), [](bool v) { return v; })) {
                                        std::cout << "All joints fault reset." << std::endl;
                                        state = READY_TO_SWITCH_ON;  // Move
                                                                     // to next
                                                                     // state
                                        std::fill(is_fault_reset.begin(), is_fault_reset.end(), false);
                                        break;
                                    }
                                }
                                break;

                            case READY_TO_SWITCH_ON:
                                for (int s = 1; s <= 6; ++s) {
                                    // Shutdown: Switch on disabled ->
                                    // Ready to switch on
                                    if ((in_somanet[s]->Statusword & 0b0000000001001111) == 0b0000000001000000) {
                                        // std::cout << "Ready to switch
                                        // on for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00000110;  // 6
                                        is_ready_switch_on.at(s - 1) = false;
                                    } else {
                                        is_ready_switch_on.at(s - 1) = true;
                                    }

                                    // check if all is_ready_switch_on
                                    // true then move to next state
                                    if (std::all_of(is_ready_switch_on.begin(), is_ready_switch_on.end(), [](bool v) { return v; })) {
                                        std::cout << "All joints ready "
                                                     "to switch on."
                                                  << std::endl;
                                        state = SWITCH_ON;  // Move to next
                                                            // state
                                        std::fill(is_ready_switch_on.begin(), is_ready_switch_on.end(), false);
                                        break;
                                    }
                                }
                                break;

                            case SWITCH_ON:
                                for (int s = 1; s <= 6; ++s) {
                                    // Switch on: Ready to switch on ->
                                    // Switched on
                                    if ((in_somanet[s]->Statusword & 0b0000000001101111) == 0b0000000000100001) {
                                        // std::cout << "Switching on
                                        // for J" << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00000111;  // 7
                                        is_switched_on.at(s - 1) = false;
                                    } else {
                                        is_switched_on.at(s - 1) = true;
                                    }

                                    // check if all is_switched_on true
                                    // then move to next state
                                    if (std::all_of(is_switched_on.begin(), is_switched_on.end(), [](bool v) { return v; })) {
                                        std::cout << "All joints switched on." << std::endl;
                                        state = CSP_MODE;  // Move to
                                                           // next state
                                        std::fill(is_switched_on.begin(), is_switched_on.end(), false);
                                        break;
                                    }
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

                                    // check if all
                                    // is_current_target_equal true then
                                    // move to next state
                                    if (std::all_of(is_current_target_equal.begin(), is_current_target_equal.end(), [](bool v) { return v; })) {
                                        std::cout << "All joints set to CSP "
                                                     "mode "
                                                     "with current position."
                                                  << std::endl;
                                        state = OPERATION_ENABLED;  // Move
                                                                    // to
                                                                    // next
                                                                    // state
                                        std::fill(is_current_target_equal.begin(), is_current_target_equal.end(), false);
                                        break;
                                    }
                                }
                                break;

                            case OPERATION_ENABLED:
                                for (int s = 1; s <= 6; ++s) {
                                    // Enable operation: Switched on ->
                                    // Operation enabled
                                    if ((in_somanet[s]->Statusword & 0b0000000001101111) == 0b0000000000100011) {
                                        // std::cout << "Enable
                                        // operation for J"
                                        // << s << std::endl;
                                        out_somanet[s]->Controlword = 0b00001111;  // 15
                                        is_operation_enabled.at(s - 1) = false;
                                    } else {
                                        is_operation_enabled.at(s - 1) = true;
                                    }

                                    // check if all is_switched_on true
                                    // then move to next state
                                    if (std::all_of(is_operation_enabled.begin(), is_operation_enabled.end(), [](bool v) { return v; })) {
                                        static int delay_counter = 0;
                                        if (++delay_counter >= 400) {  // 2s

                                            std::cout << "All joints enabled." << std::endl;
                                            state = NORMAL_OPERATION;  // Move to
                                                                       // next state
                                            delay_counter = 0;
                                        }
                                        std::fill(is_operation_enabled.begin(), is_operation_enabled.end(), false);
                                        break;
                                    }
                                }

                                break;

                            case NORMAL_OPERATION: {
                                static SCurveProfile prof[7];
                                static bool prof_active = false;
                                static double cycle_time = 0.005;  // 5 ms
                                static int prof_cycle = 0;
                                static const int32_t tolerance = 50;  // counts
                                static bool move_positive = true;
                                static int delay_counter = 0;
                                static const int delay_cycles = 1000;  // 2 seconds delay
                                static bool all_within_tolerance = false;

                                // Define max speed for each joint (counts/sec)
                                static const double max_speed[7] = {0, 436907, 436907, 436907, 218453, 218453, 218453};  // index 0 unused

                                if (!prof_active && delay_counter == 0) {
                                    double required_time = 0.0;

                                    std::cout << std::string(50, '*') << std::endl;
                                    for (int s = 1; s <= 6; ++s) {
                                        prof[s].start_pos = in_somanet[s]->PositionValue;
                                        if (move_positive) {
                                            prof[s].end_pos = prof[s].start_pos + (rand() % 200001);
                                        } else {
                                            prof[s].end_pos = 0.0;
                                        }

                                        double min_duration_per_joint = std::abs(prof[s].end_pos - prof[s].start_pos) / max_speed[s];
                                        std::cout << "J" << s << " | Start: " << prof[s].start_pos << " | End: " << prof[s].end_pos
                                                  << " | Required Time: " << min_duration_per_joint << std::endl;

                                        if (min_duration_per_joint > required_time) {
                                            required_time = min_duration_per_joint;
                                        }
                                    }

                                    std::cout << "Min required time for all joints: " << required_time << " seconds." << std::endl;
                                    required_time = std::max(required_time, 0.6);  // Ensure at least 600 ms
                                    std::cout << "Post Process Required time for all joints: " << required_time << " seconds." << std::endl;

                                    for (int s = 1; s <= 6; ++s) {
                                        prof[s].duration = required_time;
                                    }

                                    prof_cycle = 0;
                                    prof_active = true;
                                    all_within_tolerance = false;
                                }

                                double t = prof_cycle * cycle_time;
                                bool all_reached = true;
                                for (int s = 1; s <= 6; ++s) {
                                    int32_t target = (int32_t)s_curve_position(prof[s], t);
                                    if (prof_active && t <= prof[s].duration) {
                                        out_somanet[s]->TargetPosition = target;
                                        if (std::abs(in_somanet[s]->PositionValue - target) > tolerance) {
                                            all_reached = false;
                                        }
                                    } else {
                                        out_somanet[s]->TargetPosition = prof[s].end_pos;
                                        if (std::abs(in_somanet[s]->PositionValue - prof[s].end_pos) > tolerance) {
                                            all_reached = false;
                                        }
                                    }
                                }

                                if (prof_active) {
                                    if (t < prof[1].duration) {
                                        prof_cycle++;
                                    } else if (all_reached) {
                                        prof_active = false;
                                        delay_counter = delay_cycles;
                                        move_positive = !move_positive;
                                    }
                                    // If not all reached, keep holding target until all are within tolerance
                                } else if (delay_counter > 0) {
                                    for (int s = 1; s <= 6; ++s) {
                                        out_somanet[s]->TargetPosition = prof[s].end_pos;
                                    }
                                    delay_counter--;
                                }
                                break;
                            }

                            default:
                                std::cout << "Unknown state." << std::endl;
                                break;
                        }
                    }
                    osal_usleep(5000);
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
        /* create thread to handle slave error handling in OP */
        //      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*
        //      &ctime);
        osal_thread_create(&thread1, 128000, (void *)&ecatcheck, (void *)&ctime);
        /* start cyclic part */
        simpletest(argv[1]);
    } else {
        printf(
            "TODO: Add instructions on how this executable should be used. "
            "Maybe add the following example\n");
        printf(
            "ros2 run synapticon_ros2_control torque_control_executable -- "
            "eno0\n");
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
