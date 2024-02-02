#pragma once

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cmath>

constexpr int NUM_JOINTS = 4; // Change this to the desired number of joints

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
        // Initialize boolean flags
        switch_to_operation = false;
        initialize_drives = false;
        initialize_system = false;
        trigger_error = false;
        safety_process_status = false;
        drive_initialized = false;
        safety_check_done = false;
        reset_error = false;
        operation_enable_status = false;

        // Use std::fill_n for array initialization
        std::fill_n(actual_position, NUM_JOINTS, 0.0);
        std::fill_n(actual_velocity, NUM_JOINTS, 0.0);
        std::fill_n(actual_torque, NUM_JOINTS, 0.0);
        std::fill_n(cart_pos, NUM_JOINTS, 0.0);
        std::fill_n(target_position, NUM_JOINTS, 0.0);
        std::fill_n(target_velocity, NUM_JOINTS, 0.0);
        std::fill_n(target_torque, NUM_JOINTS, 0.0);

        // Initialize other members
        drive_operation_mode = OperationModeState::POSITION_MODE;
        switched_on = false;
        sterile_detection = false;
        instrument_detection = false;
        simulation_mode = false;
    }

    double actual_position[NUM_JOINTS];
    double actual_velocity[NUM_JOINTS];
    double actual_torque[NUM_JOINTS];
    double cart_pos[NUM_JOINTS];
    double target_position[NUM_JOINTS];
    double target_velocity[NUM_JOINTS];
    double target_torque[NUM_JOINTS];
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

        // Use std::fill_n for array initialization
        std::fill_n(cart_pos, 3, 0.0);
        std::fill_n(cart_linear_vel, 3, 0.0);
        std::fill_n(cart_angular_vel, 3, 0.0);
        std::fill_n(cart_orient, 9, 0.0);
    }

    double cart_pos[3];
    double cart_linear_vel[3];
    double cart_orient[9];
    double cart_angular_vel[3];
    double gripper_pos;
    double gripper_vel;
};