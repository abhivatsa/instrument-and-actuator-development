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

double hand_control_jog(double start_pos[3], Eigen::Vector3d& eef_pos, Eigen::Matrix3d& eef_orient);

// structer for system data
enum SystemState{
    POWER_OFF,
    INITIALIZING_SYSTEM,
    HARWARE_CHECK,
    READY,
    IN_EXECUTION,
    RECOVERY,
    ERROR
};

enum CommandType{
    NONE,
    JOG,
    HAND_CONTROL,
    STERILE_ENGAGEMENT,
    INSTRUMENT_ENGAGEMENT
};

enum OperationModeState{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};

struct AppData
{
    void setZero()
    {
        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            actual_position[jnt_ctr] = 0;
            actual_velocity[jnt_ctr] = 0;
            actual_torque[jnt_ctr] = 0;
            actual_cart_pos[3] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
            target_cart_pos[3] = 0;
            drive_operation_mode = OperationModeState::POSITION_MODE;
            switched_on = false;
        }
    }

    double actual_position[3];
    double actual_velocity[3];
    double actual_torque[3];
    double actual_cart_pos[3];
    double target_position[3];
    double target_velocity[3];
    double target_torque[3];
    double target_cart_pos[3];
    OperationModeState drive_operation_mode;
    bool switched_on;

};

struct SystemData
{
    SystemState getSystemState() const {return system_state;}
    void setSystemState(SystemState state){system_state = state;}
    void powerOn(){ request = system_state == SystemState::POWER_OFF ? 1 : 0;}
    void powerOff(){request = system_state == SystemState::READY ? -1 : 0;}
    int request = 0;
private:
    SystemState system_state = SystemState::POWER_OFF; 
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
    void setHandControl(){
        this->type = CommandType::HAND_CONTROL;
    }
    void setSterileEngagement(){
        this->type = CommandType::STERILE_ENGAGEMENT;
    }
    void setInstrumentEngagement(){
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


SystemData *system_data_ptr;
AppData *app_data_ptr;
CommandData *commmand_data_ptr;