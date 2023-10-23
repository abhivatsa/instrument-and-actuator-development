#include <cmath>
#include <unistd.h>
#include <iostream>

#include "MotionPlanning/ForwardKinematics.h"
#include "MotionPlanning/IK6AxisInline.h"
#include "MotionPlanning/Jacobian.h"

int write_to_drive(double joint_pos[3], double joint_vel[3]);

int pt_to_pt_mvmt(double ini_pos[3], double final_pos[3]);

int changeSystemState();

double jog(int index, int dir, int type);

double hand_control_jog(double start_pos[3], Eigen::Vector3d &eef_pos, Eigen::Matrix3d &eef_orient);

// structer for system data
enum SystemState
{
    POWER_OFF,
    INITIALIZING_SYSTEM,
    HARWARE_CHECK,
    READY,
    IN_EXECUTION,
    RECOVERY,
    ERROR
};

enum ActuatorState
{
    NONE,
    STERILE_MOUNTED,
    STERILE_ENGAGED,
    INSTRUMENT_MOUNTED,
    INSTRUMENT_ENGAGED
};

enum CommandType
{
    NONE,
    JOG,
    HAND_CONTROL,
    STERILE_ENGAGEMENT,
    INSTRUMENT_ENGAGEMENT
};

enum OperationModeState
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

            init_system = -1;
            init_hardware_check = -1;
            init_ready_for_operation = -1;
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

    int init_system;
    int init_hardware_check;
    int init_ready_for_operation;

    bool initialize_system;
    bool initialize_drives;
    bool switch_to_operation;
};

struct SystemData
{
    SystemState getSystemState() const { return system_state; }
    void setSystemState(SystemState state) { system_state = state; }
    ActuatorState getActuatorState() const { return actuator_state; }
    void setActuatorState(ActuatorState state) {actuator_state = state; }
    void powerOn() { request = system_state == SystemState::POWER_OFF ? 1 : 0; }
    void powerOff() { request = system_state == SystemState::READY ? -1 : 0; }
    int request = 0;

private:
    SystemState system_state = SystemState::POWER_OFF;
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