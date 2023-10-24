#ifndef INSTRUMENT_MOTION_PLANNER_H
#define INSTRUMENT_MOTION_PLANNER_H

#include <cmath>
#include <unistd.h>
#include <iostream>

int write_to_drive(double joint_pos[3], double joint_vel[3]);

int changeSystemState();



// structer for system data
enum class SystemState
{
    POWER_OFF,
    INITIALIZING_SYSTEM,
    HARWARE_CHECK,
    READY,
    IN_EXECUTION,
    RECOVERY,
    ERROR
};

enum class ActuatorState
{
    NONE,
    STERILE_MOUNTED,
    STERILE_ENGAGED,
    INSTRUMENT_MOUNTED,
    INSTRUMENT_ENGAGED
};

enum class CommandType
{
    NONE,
    JOG,
    HAND_CONTROL,
    STERILE_ENGAGEMENT,
    INSTRUMENT_ENGAGEMENT
};

enum class OperationModeState
{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
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

struct SystemData
{
    SystemState getSystemState() const { return system_state; }
    void setSystemState(SystemState state) { previous_state = system_state; system_state = state; }
    ActuatorState getActuatorState() const { return actuator_state; }
    void setActuatorState(ActuatorState state) {actuator_state = state; }
    void powerOn() { request = system_state == SystemState::POWER_OFF ? 1 : 0; }
    void powerOff() { request = system_state == SystemState::READY ? -1 : 0; }

    void resetError(){ 
        if(previous_state == SystemState::IN_EXECUTION)
            previous_state = SystemState::READY;
        system_state = previous_state; 
        }
    int request = 0;

private:
    SystemState system_state = SystemState::POWER_OFF;
    SystemState previous_state = SystemState::POWER_OFF;
    ActuatorState actuator_state = ActuatorState::NONE;
};

struct CommandData
{
    void setJog(int index, int dir, int mode)
    {
        this->type = CommandType::JOG;
        jog_data.index = index - 1;
        jog_data.dir = dir;
        jog_data.type = mode;
    }
    void setHandControl()
    {
        this->type = CommandType::HAND_CONTROL;
    }
    void setSterileEngagement()
    {
        this->type = CommandType::STERILE_ENGAGEMENT;
    }
    void setInstrumentEngagement()
    {
        this->type = CommandType::INSTRUMENT_ENGAGEMENT;
    }
    void setNone(){
        this->type = CommandType::NONE;
    }

    CommandType type;
    struct
    {
        int index;
        int dir;
        int type;
    } jog_data;
    struct
    {
        int type;
        double goal_position[3];
    } move_to_data;
};

struct ForceDimData
{

    void setZero()
    {

        gripper_pos = 0;
        gripper_vel = 0;

        for (int ctr = 0; ctr < 3; ctr++)
        {
            cart_pos[ctr] = 0;
            cart_linear_vel[ctr] = 0;
            cart_angular_vel[ctr] = 0;
        }

        for (int ctr = 0; ctr < 9; ctr++)
        {
            cart_orient[ctr] = 0;
        }
    }

    double cart_pos[3];
    double cart_linear_vel[3];
    double cart_orient[9];
    double cart_angular_vel[3];
    double gripper_pos;
    double gripper_vel;
};

SystemData *system_data_ptr;
AppData *app_data_ptr;
CommandData *commmand_data_ptr;
ForceDimData *force_dim_ptr;

#endif