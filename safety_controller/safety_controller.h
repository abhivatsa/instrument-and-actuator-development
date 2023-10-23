#include <cmath>
#include <unistd.h>
#include <iostream>

#include "MotionPlanning/ForwardKinematics.h"
#include "MotionPlanning/IK6AxisInline.h"
#include "MotionPlanning/Jacobian.h"

enum class DriveState
{
    INITIALIZE,
    NOT_READY_TO_SWITCH_ON,
    SWITCHED_ON,
    SWITCH_TO_OPERATION,
    OPERATION_ENABLED,
    ERROR,
};

enum class OperationModeState
{
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

struct AppData
{
    void setZero()
    {
        switch_to_operation = false;
        initialize_drives = false;
        initialize_system = false;
        trigger_error = false;
        safety_process_status = false;
        drive_initialized = false;
        safety_check_done = false;
        reset_error = false;
        operation_enable_status = false;

        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            actual_position[jnt_ctr] = 0;
            actual_velocity[jnt_ctr] = 0;
            actual_torque[jnt_ctr] = 0;
            cart_pos[3] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
            drive_operation_mode = OperationModeState::POSITION_MODE;
            switched_on = false;
            sterile_detection = false;
            instrument_detection = false;
            simulation_mode = false;
        }
    }

    double actual_position[3];
    double actual_velocity[3];
    double actual_torque[3];
    double cart_pos[3];
    double target_position[3];
    double target_velocity[3];
    double target_torque[3];
    OperationModeState drive_operation_mode;
    bool switched_on;
    bool sterile_detection;
    bool instrument_detection;
    bool simulation_mode;

    bool trigger_error;
    bool safety_process_status;
    bool initialize_system;
    bool initialize_drives;
    bool drive_initialized;
    bool switch_to_operation;
    bool safety_check_done;
    bool operation_enable_status;
    bool reset_error;
};

struct SystemStateData
{
    void setZero()
    {
        current_state = DriveState::INITIALIZE;
        drive_operation_mode = OperationModeState::POSITION_MODE;
        initialize_drives = false;
        switch_to_operation = false;

        status_switched_on = false;
        status_operation_enabled = false;
        safety_controller_enabled = false;
        trigger_error_mode = false;

        safety_check_done = false;
        start_safety_check = false;
        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            drive_enable_for_operation[jnt_ctr] = false;
        }
    }

    DriveState current_state;
    OperationModeState drive_operation_mode;
    // Variables for Drive status
    bool status_switched_on;
    bool status_operation_enabled;

    // To determine where safety Code started Running
    bool safety_controller_enabled;

    // both Variables ghas to Come from Safety Code
    bool trigger_error_mode; // Chaged from safety controller

    // Variables to initialize and check the status of safety code at 1000Hz
    bool start_safety_check;
    bool safety_check_done;

    bool initialize_drives;
    bool switch_to_operation;

    bool drive_enable_for_operation[3];
};

JointData *joint_data_ptr;
SystemStateData *system_state_data_ptr;
AppData *app_data_ptr;

int pos_limit_check(double *joint_pos);

void read_data();
void write_data();
bool check_limits();
