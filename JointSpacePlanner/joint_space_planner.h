#include <cmath>
#include <unistd.h>
#include <iostream>

#include "MotionPlanning/ForwardKinematics.h"
#include "MotionPlanning/IK6AxisInline.h"
#include "MotionPlanning/Jacobian.h"

int conv_radians_to_count(double rad, int joint_num);

double conv_count_to_rad(double count, int joint_num);

int write_to_drive(double joint_pos[6], double joint_vel[6]);

int pt_to_pt_mvmt(double ini_pos[6], double final_pos[6]);

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
    GRAVITY,
    MOVE_TO
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

struct RobotState
{
    void setZero()
    {
        for(int i = 0; i < 6; i++)
        {
            joint_position[i] = 0;
            cart_position[i] = 0;
            joint_velocity[i] = 0;
            joint_torque[i] = 0;
        }
    }
    double cart_position[6];
    double joint_position[6];
    double joint_velocity[6];
    double joint_torque[6];
};

struct CommandData
{
    void setJog(int index, int dir, int mode)
    {
        this->type = CommandType::JOG;
        jog_data.index = index;
        jog_data.dir = dir;
        jog_data.type = mode;
    }
    void setMoveTo(double goal[6], int type)
    {
        for(int i = 0; i < 6; i ++)
        {
            move_to_data.goal_position[i] = goal[i];
        }
        move_to_data.type = type;
        this->type = CommandType::MOVE_TO;
    }
    CommandType type;
    struct{
        int index;
        int dir;
        int type;
    } jog_data;
    struct{
        int type;
        double goal_position[6];
    } move_to_data;
};
