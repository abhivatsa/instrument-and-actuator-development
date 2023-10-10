#include <cmath>
#include <unistd.h>
#include <iostream>

#include "MotionPlanning/ForwardKinematics.h"
#include "MotionPlanning/IK6AxisInline.h"
#include "MotionPlanning/Jacobian.h"

struct JointData
{

    void setZero()
    {
        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            joint_position[jnt_ctr] = 0;
            joint_velocity[jnt_ctr] = 0;
            joint_torque[jnt_ctr] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
        }
    }

    double joint_position[6];
    double joint_velocity[6];
    double joint_torque[6];
    double target_position[6];
    double target_velocity[6];
    double target_torque[6];
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
        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++){
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
    bool drive_enable_for_operation[6];

    bool safety_check_done;
};

JointData *joint_data_ptr;
SystemStateData *system_state_data_ptr; 

int pos_limit_check(double* joint_pos);
