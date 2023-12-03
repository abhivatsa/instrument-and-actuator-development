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

<<<<<<< HEAD
// Define your EtherCAT constants here
=======
/****************************************************************************/

#define DOMAIN1_POSITION 0
>>>>>>> d312dfc19e8e814b2a3aa21d0d209afe7fb91b0f
#define DOMAIN1_START 1
#define DOMAIN1_END (DOMAIN1_START + NUM_JOINTS)

// Example definition, replace it with your actual slave configuration
#define ingeniaDenalliXcr 0x0000029c, 0x03831002

<<<<<<< HEAD
=======
/****************************************************************************/
#define CLOCK_TO_USE CLOCK_MONOTONIC
// #define MEASURE_TIMING 0
/** Task period in ns. */
>>>>>>> d312dfc19e8e814b2a3aa21d0d209afe7fb91b0f
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

<<<<<<< HEAD

};

=======
/****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

void read_drive_state(uint16_t status, int joint_num){

    // cout << "update_state" << endl;
    if (((status | 65456) ^ 65456) == 0)
    {
        std::cout<<"Not ready to switch on, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65456) ^ 65520) == 0)
    {
        std::cout<<"Switch on Disabled, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65424) ^ 65457) == 0)
    {
        std::cout<<"Ready to Switch on, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65424) ^ 65459) == 0)
    {
        std::cout<<"Switched On, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65424) ^ 65463) == 0)
    {
        std::cout<<"Operation Enabled, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65456) ^ 65471) == 0)
    {
        // Fault Reaction Active
        std::cout << "Fault reaction active" << std::endl;
    }
    else if (((status | 65456) ^ 65464) == 0)
    {
        // Fault
        std::cout << "Fault" << std::endl;
    }
    else{
        std::cout<<"State Unknown"<<std::endl;
    }

}

/*****************************************************************************/

uint16_t transition_to_switched_on(uint16_t status, uint16_t command, int joint_num)
{
    // cout << "update_state" << endl;
    if (((status | 65456) ^ 65456) == 0)
    {
        // std::cout<<"Not ready to switch on, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65456) ^ 65520) == 0)
    {
        // std::cout<<"Switch on Disabled, joint num : "<<joint_num<<std::endl;
        command = 6;
    }
    else if (((status | 65424) ^ 65457) == 0)
    {
        // std::cout<<"Ready to Switch on, joint num : "<<joint_num<<std::endl;
        command = 7;
    }
    else if (((status | 65424) ^ 65459) == 0)
    {
        // std::cout<<"Switched On, joint num : "<<joint_num<<std::endl;
        // command = 15;
        drive_switched_on[joint_num] = true;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

/*****************************************************************************/

uint16_t transition_to_operation_enabled(uint16_t status, uint16_t command, int joint_num)
{
    if (((status | 65424) ^ 65459) == 0)
    {
        std::cout << "Switched On, joint num : " << joint_num << std::endl;
        command = 15;
    }
    else if (((status | 65424) ^ 65463) == 0)
    {
        // printf(" Operation Enabled \n");
        // Operation Enabled
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

/*****************************************************************************/

uint16_t transition_to_fault_state(uint16_t status, uint16_t command, int joint_num)
{
    // cout << "update_state" << endl;
    if (((status | 65456) ^ 65471) == 0)
    {
        // Fault Reaction Active
        std::cout << "Fault reaction active" << std::endl;
    }
    else if (((status | 65456) ^ 65464) == 0)
    {
        // Fault
        std::cout << "Fault" << std::endl;
        command = 15;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}
>>>>>>> d312dfc19e8e814b2a3aa21d0d209afe7fb91b0f
