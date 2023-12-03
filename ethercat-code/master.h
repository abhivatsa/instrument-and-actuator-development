#pragma once

#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <thread>
#include <signal.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sched.h>
#include <stdbool.h>
#include "ecrt.h"
#include "SharedObject.h"

// Define your EtherCAT constants here
#define DOMAIN1_START 1
#define DOMAIN1_END (DOMAIN1_START + NUM_JOINTS)

// Example definition, replace it with your actual slave configuration
#define ingeniaDenalliXcr 0x0000029c, 0x03831002

#define PERIOD_NS (2000000)
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

struct JointPdos {
    unsigned int statusword;
    unsigned int mode_of_operation_display;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_input_value;
    unsigned int error_code;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int torque_offset;
    unsigned int velocity_offset;
};

class Master {
public:
    Master();
    ~Master();

    void run();
private:
    ec_master_t *master;
    ec_master_state_t masterState;
    ec_domain_t *domain;
    ec_domain_state_t domainState;
    static uint8_t *domainPd;
    unsigned int counter;
    unsigned int syncRefCounter;
    const struct timespec cycleTime;
    bool driveSwitchedOn[NUM_JOINTS];
    bool allDriveEnabled;
    bool run_ethercat_loop = true;
    JointPdos driveOffset[NUM_JOINTS];
    JointData *jointDataPtr;
    SystemStateData *systemStateDataPtr;

    void updateState();
    void readDriveState(uint16_t status, int joint_num);

    uint16_t transitionToSwitchedOn(uint16_t status, uint16_t command, int joint_num);
    uint16_t transitionToOperationEnabled(uint16_t status, uint16_t command, int joint_num);
    uint16_t transitionToFaultState(uint16_t status, uint16_t command, int joint_num);

    struct timespec timespecAdd(struct timespec time1, struct timespec time2);

    void checkDomainState();
    void checkMasterState();
    void cyclicTask();
    
    void sdoMapping(ec_slave_config_t *sc, int jnt_ctr);
    void pdoMapping(ec_slave_config_t *sc);

    
    void configureSharedMemory();
    void createSharedMemory(int& shm_fd, const char* name, int size);
    void mapSharedMemory(void*& ptr, int shm_fd, int size);
    void initializeSharedData();

    void stackPrefault();
    void setRealtimePriority();

    void signalHandler(int signum);

};
