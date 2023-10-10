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
        }
    }

    double joint_position[3];
    double joint_velocity[3];
    double joint_torque[3];
    double target_position[3];
    double target_velocity[3];
    double target_torque[3];
};

enum DriveState
{
    DISABLED,
    SWITCHED_ON,
    SAFETY_CONTROLLER_ENABLED,
    READY_FOR_OPERATION,
    SWITCH_TO_OPERATION,
    OPERATION_ENALBLED,
    ERROR,
};

struct SystemStateData
{
    void setZero(){
        current_state = DriveState::SWITCHED_ON;

        status_switched_on = false;
        status_operation_enabled = false;
        safety_controller_enabled =  false;
        trigger_error_mode = false;
        trigger_operation_enabled = false;
        ready_for_operation = false;
        safety_check_done = false;
        start_safety_check = false;
        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++){
            drive_enable_for_operation[jnt_ctr] = false;
        }
    }

    DriveState current_state;

    bool status_switched_on;
    bool status_operation_enabled;
    bool safety_controller_enabled; // 
    bool trigger_error_mode;   // Chaged from safety controller 
    bool trigger_operation_enabled; // Done From UI to enable motors
    bool start_safety_check;
    bool ready_for_operation;
    bool drive_enable_for_operation[3];

    bool safety_check_done;
};

JointData *joint_data_ptr;
SystemStateData *system_state_data_ptr;

#endif
