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
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <ecrt.h>
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

volatile sig_atomic_t exitFlag = 0;

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

enum class ControlWordValues : uint16_t {
    CW_SHUTDOWN             = 0x06,
    CW_SWITCH_ON            = 0x07,
    CW_ENABLE_OPERATION     = 0x0F,
    CW_DISABLE_VOLTAGE      = 0x00,
    CW_QUICK_STOP           = 0x02,
    // Add more control word values as needed
};

enum class StatusWordValues : uint16_t {
    SW_NOT_READY_TO_SWITCH_ON = 0x0000,
    SW_SWITCH_ON_DISABLED = 0x0040,
    SW_READY_TO_SWITCH_ON = 0x0021,
    SW_SWITCHED_ON = 0x0023,
    SW_OPERATION_ENABLED = 0x0027,
    SW_QUICK_STOP_ACTIVE = 0x0007,
    SW_FAULT_REACTION_ACTIVE = 0x000F,
    SW_FAULT = 0x0008
    // Add more status word values as needed
};

class EthercatMaster {
public:
    
    EthercatMaster();
    ~EthercatMaster();
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

    void readDriveState(uint16_t statusword, int joint_num);

    uint16_t transitionToSwitchedOn(uint16_t status, int joint_num);
    uint16_t transitionToOperationEnabled(uint16_t status, int joint_num);
    uint16_t transitionToFaultState(uint16_t status, int joint_num);

    void transitionToState(ControlWordValues value, int jnt_ctr); 

    struct timespec timespecAdd(struct timespec time1, struct timespec time2);

    void checkDomainState();
    void checkMasterState();
    
    void pdoMapping(ec_slave_config_t *sc);

    void configureSharedMemory();
    void createSharedMemory(int& shm_fd, const char* name, int size);
    void mapSharedMemory(void*& ptr, int shm_fd, int size);
    void initializeSharedData();

    void stackPrefault();

    static void signalHandler(int signum);
    
    // Organized Cyclic Task Structure
    void cyclicTask();
    void performPeriodicTasks();
    void handleDriveStates();
    void initializeDrives();
    void enableDrivesIfInitialized();
    void handleSwitchedOnState();
    void updateJointDataInSwitchedOnState();
    void handleSafetyCheckDone();
    void handleSwitchToOperationState();
    void updateSwitchToOperationState();
    void handleOperationEnabledState();
    void handlePositionMode();
    void handleVelocityMode();
    void handleTorqueMode();
    void handleErrorState();

};
