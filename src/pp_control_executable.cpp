#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

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

void simpletest(const char *ifname) {
    int i, j, chk;
    needlf = false;
    inOP = false;

    printf("Starting simple test\n");

    // initialise SOEM, bind socket to ifname
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n", ifname);
        // find and auto-config slaves

        if (ec_config_init(false) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);

            ec_config_map(&IOmap);

            ec_configdc();

            // --- Set Profile Position parameters via SDO ---
            uint32_t profile_velocity = 5000;       // counts/sec
            uint32_t profile_acceleration = 50000;  // counts/sec^2
            uint32_t profile_deceleration = 50000;  // counts/sec^2
            uint32_t quick_stop_decel = 1000000;    // counts/sec^2

            int slave = 1;  // EtherCAT slaves are 1-based in SOEM

            // Set Profile Velocity (0x6081)
            ec_SDOwrite(slave, 0x6081, 0, FALSE, sizeof(profile_velocity),
                        &profile_velocity, EC_TIMEOUTRXM);
            // Set Profile Acceleration (0x6083)
            ec_SDOwrite(slave, 0x6083, 0, FALSE, sizeof(profile_acceleration),
                        &profile_acceleration, EC_TIMEOUTRXM);
            // Set Profile Deceleration (0x6084)
            ec_SDOwrite(slave, 0x6084, 0, FALSE, sizeof(profile_deceleration),
                        &profile_deceleration, EC_TIMEOUTRXM);
            // Set Quick Stop Deceleration (0x6085)
            ec_SDOwrite(slave, 0x6085, 0, FALSE, sizeof(quick_stop_decel),
                        &quick_stop_decel, EC_TIMEOUTRXM);

            printf("Profile Position parameters set via SDO.\n");
            // --- End SDO setup ---

            printf("Slaves mapped, state to SAFE_OP.\n");
            // wait for all slaves to reach SAFE_OP state
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments,
                   ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
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
                // create and connect struture pointers to I/O
                in_somanet_50t *in_somanet_1;
                in_somanet_1 = (in_somanet_50t *)ec_slave[0].inputs;
                out_somanet_50t *out_somanet_1;
                out_somanet_1 = (out_somanet_50t *)ec_slave[0].outputs;

                // cyclic loop
                for (i = 1; i <= 10000; i++) {
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC) {
                        j++;
                        // Profile position mode
                        if (j == 1) out_somanet_1->OpMode = 1;

                        // Fault reset: Fault -> Switch on disabled, if the
                        // drive is in fault state
                        if ((in_somanet_1->Statusword & 0b0000000001001111) ==
                            0b0000000000001000)
                            out_somanet_1->Controlword = 0b10000000;

                        // Shutdown: Switch on disabled -> Ready to switch on
                        else if ((in_somanet_1->Statusword &
                                  0b0000000001001111) == 0b0000000001000000)
                            out_somanet_1->Controlword = 0b00000110;

                        // Switch on: Ready to switch on -> Switched on
                        else if ((in_somanet_1->Statusword &
                                  0b0000000001101111) == 0b0000000000100001)
                            out_somanet_1->Controlword = 0b00000111;

                        // Enable operation: Switched on -> Operation enabled
                        else if ((in_somanet_1->Statusword &
                                  0b0000000001101111) == 0b0000000000100011)
                            out_somanet_1->Controlword = 0b00001111;

                        // Sending profile Position command
                        else if ((in_somanet_1->Statusword &
                                  0b0000000001101111) == 0b0000000000100111) {
                            static int toggle = 0;
                            static int32_t last_target = 0;

                            // Always send the command, update target every 200
                            // cycles
                            if (i % 1000 == 0) {
                                last_target = in_somanet_1->PositionValue +
                                              100000 * (toggle ? 1 : -1);
                                toggle = !toggle;

                                // Set "new set-point" bit (bit 4) motor will
                                // move on rising edge
                                out_somanet_1->TargetPosition = last_target;
                                out_somanet_1->Controlword = 0b00011111;
                                printf("\nNew target position: %d\n",
                                       last_target);
                            } else {
                                // Keep sending the last target, clear "new
                                // set-point" bit
                                out_somanet_1->TargetPosition = last_target;
                                out_somanet_1->Controlword = 0b00001111;
                            }
                        }

                        printf("Processdata cycle %4d , WKC %d ,", i, wkc);
                        printf(" Statusword: %X ,", in_somanet_1->Statusword);
                        printf(" Op Mode Display: %d ,",
                               in_somanet_1->OpModeDisplay);
                        printf(" ActualPos: %" PRId32 " ,",
                               in_somanet_1->PositionValue);
                        printf(" ActualVel: %" PRId32 " ,",
                               in_somanet_1->VelocityValue);
                        printf(" DemandVel: %" PRId32 " ,",
                               in_somanet_1->VelocityDemandValue);
                        printf(" ActualTorque: %" PRId32 " ,",
                               in_somanet_1->TorqueValue);
                        printf(" DemandTorque: %" PRId32 " ,",
                               in_somanet_1->TorqueDemand);

                        printf(" T:%" PRId64 "\r", ec_DCtime);
                        needlf = true;
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
                            "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                            ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
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
        if (inOP &&
            ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = false;
                printf("\n");
            }
            // one ore more slaves are not responding
            ec_group[currentgroup].docheckstate = false;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) &&
                    (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = true;
                    if (ec_slave[slave].state ==
                        (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf(
                            "ERROR : slave %d is in SAFE_OP + ERROR, "
                            "attempting ack.\n",
                            slave);
                        ec_slave[slave].state =
                            (EC_STATE_SAFE_OP + EC_STATE_ACK);
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
                        ec_statecheck(slave, EC_STATE_OPERATIONAL,
                                      EC_TIMEOUTRET);
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
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[]) {
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1) {
        /* create thread to handle slave error handling in OP */
        //      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*)
        //      &ctime);
        osal_thread_create(&thread1, 128000, (void *)&ecatcheck,
                           (void *)&ctime);
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
