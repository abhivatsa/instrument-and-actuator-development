#ifndef SHARED_OBJECT_H
#define SHARED_OBJECT_H

#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

#include <bits/stdc++.h>
#include <sys/time.h>

struct JointData
{

    void setZero()
    {
        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            joint_position[jnt_ctr] = 0;
            joint_velocity[jnt_ctr] = 0;
            joint_torque[jnt_ctr] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
            sterile_detection_status = false;
            instrument_detection_status = false;
        }
    }

    double joint_position[3];
    double joint_velocity[3];
    double joint_torque[3];
    double target_position[3];
    double target_velocity[3];
    double target_torque[3];
    bool sterile_detection_status;
    bool instrument_detection_status;
};

enum class DriveState
{
    INITIALIZE,
    NOT_READY_TO_SWITCH_ON,
    SWITCHED_ON,
    SWITCH_TO_OPERATION,
    OPERATION_ENABLED,
    ERROR,
};

enum class OperationModeState{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};

enum class SafetyStates
{
    INITIALIZE,
    INITIALIZE_DRIVES,
    SAFETY_CHECK,
    READY_FOR_OPERATION,
    OPERATION,
    ERROR,
};

struct SystemStateData
{
    void setZero(){
        current_state = DriveState::INITIALIZE;
        drive_operation_mode = OperationModeState::POSITION_MODE;
        state = SafetyStates::INITIALIZE;
        initialize_drives = false;
        switch_to_operation = false;

        status_switched_on = false;
        status_operation_enabled = false;
        safety_controller_enabled = false;
        trigger_error_mode = false;

        safety_check_done = false;
        start_safety_check = false;
        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++){
            drive_enable_for_operation[jnt_ctr] = false;
        }
    }

    DriveState current_state;
    OperationModeState drive_operation_mode;
    SafetyStates state;
    // Variables for Drive status
    bool status_switched_on; 
    bool status_operation_enabled;

    // To determine where safety Code started Running
    bool safety_controller_enabled; 

    // both Variables ghas to Come from Safety Code
    bool trigger_error_mode;   // Chaged from safety controller 
    
    // Variables to initialize and check the status of safety code at 1000Hz
    bool start_safety_check;
    bool safety_check_done;

    bool initialize_drives;
    bool switch_to_operation;
    
    bool drive_enable_for_operation[3];
};

JointData *joint_data_ptr;
SystemStateData *system_state_data_ptr;

#endif
