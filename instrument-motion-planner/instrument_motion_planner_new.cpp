#include "instrument_motion_planer_new.h"

#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

using namespace std;

/* pointer to shared memory object */
double *force_dim_ptr;
motion_planning::ForwardKinematics fk_solver;
motion_planning::IK6AxisInline ik_solver;
motion_planning::Jacobian jac_solver(1);


int main()
{

    /* the size (in bytes) of shared memory object */
    const int SIZE_SysData = sizeof(SystemData);
    const int SIZE_AppData = sizeof(AppData);
    const int SIZE_ComData = sizeof(CommandData);
    const int SIZE_ForceDimData = sizeof(ForceDimData);

    /* shared memory file descriptor */
    int shm_fd_SysData;
    double shm_fd_AppData;
    double shm_fd_ComData;
    double shm_fd_ForceDimData;

    /* open the shared memory object */
    shm_fd_SysData = shm_open("SysData", O_CREAT | O_RDWR, 0666);
    shm_fd_AppData = shm_open("AppData", O_CREAT | O_RDWR, 0666);
    shm_fd_ComData = shm_open("ComData", O_CREAT | O_RDWR, 0666);
    shm_fd_ForceDimData = shm_open("ForceDimData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_SysData, SIZE_SysData);
    ftruncate(shm_fd_AppData, SIZE_AppData);
    ftruncate(shm_fd_ComData, SIZE_ComData);
    ftruncate(shm_fd_ForceDimData, SIZE_ForceDimData);

    /* memory map the shared memory object */
    system_data_ptr = static_cast<SystemData *>(mmap(0, SIZE_SysData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_SysData, 0));
    app_data_ptr = static_cast<AppData *>(mmap(0, SIZE_AppData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_AppData, 0));
    commmand_data_ptr = static_cast<CommandData *>(mmap(0, SIZE_ComData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ComData, 0));
    force_dim_ptr = static_cast<ForceDimData *>(mmap(0, SIZE_ForceDimData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ForceDimData, 0));

    double start_pos[3] = {0, 0, 0};

    start_pos[0] = force_dim_ptr->cart_pos[0];
    start_pos[1] = force_dim_ptr->cart_pos[1];
    start_pos[2] = force_dim_ptr->cart_pos[2];

    bool in_operation_ = false;
    system_data_ptr->setSystemState(SystemState::POWER_OFF);
    system_data_ptr->request = 0;
    app_data_ptr->setZero();
    commmand_data_ptr->type = CommandType::NONE;

    bool hand_Controller_switch = false;

    while (1)
    {
        switch (system_data_ptr->getSystemState())
        {
            case SystemState::POWER_OFF:
            {/* code */
                if(system_data_ptr->request == 1)
                {
                    app_data_ptr->initialize_system = true;
                    system_data_ptr->setSystemState(SystemState::INITIALIZING_SYSTEM);
                }
                else
                {
                    app_data_ptr->initialize_system = false;
                }
                system_data_ptr->request = 0;
                break;
            }
            case SystemState::INITIALIZING_SYSTEM:
            {
                if(app_data_ptr->safety_process_status)
                {
                    app_data_ptr->initialize_drives = true;
                    if(app_data_ptr->drive_initialized)
                    {
                        if(app_data_ptr->trigger_error)
                        {
                            app_data_ptr->safety_check_done  = false;
                            system_data_ptr->setSystemState(SystemState::ERROR);
                        }
                        else
                        {
                            app_data_ptr->safety_check_done  = false;
                            system_data_ptr->setSystemState(SystemState::HARWARE_CHECK);
                        }
                    }
                }
                else
                {
                    app_data_ptr->initialize_drives = false;
                }
                break;
            }
            case SystemState::HARWARE_CHECK:
            {
                app_data_ptr->initialize_drives = false;
                app_data_ptr->drive_initialized = false;

                if(app_data_ptr->safety_check_done)
                {
                    if(app_data_ptr->trigger_error)
                    {
                        system_data_ptr->setSystemState(SystemState::ERROR);
                    }
                    else
                    {
                        system_data_ptr->setSystemState(SystemState::READY);
                    }
                }
                break;
            }
            case SystemState::READY:
            {
                app_data_ptr->switch_to_operation = true;
                if(app_data_ptr->trigger_error)
                {
                    system_data_ptr->setSystemState(SystemState::ERROR);
                }
                else if(app_data_ptr->operation_enalble_status)
                {
                    if(commmand_data_ptr->type != CommandType::NONE)
                    {
                        system_data_ptr->setSystemState(SystemState::IN_EXECUTION);
                    }
                }
                break;
            }
            case SystemState::IN_EXECUTION:
            {
                if(app_data_ptr->trigger_error)
                {
                    system_data_ptr->setSystemState(SystemState::ERROR);
                }
                else
                {
                    if(commmand_data_ptr->type == CommandType::JOG)
                    {
                        Jog();
                    }
                    else if(commmand_data_ptr->type == CommandType::HAND_CONTROL)
                    {

                    }
                    else if(commmand_data_ptr->type == CommandType::STERILE_ENGAGEMENT)
                    {

                    }
                    else if(commmand_data_ptr->type == CommandType::INSTRUMENT_ENGAGEMENT)
                    {

                    }
                    else
                    {

                    }

                    if(app_data_ptr->trigger_error)
                    {
                        system_data_ptr->setSystemState(SystemState::ERROR);
                    }
                }
                break;
            }
            case SystemState::ERROR:
            {
                app_data_ptr ->trigger_error = false;
                if(app_data_ptr->reset_error)
                {
                    system_data_ptr->resetError();
                }
                break;
            }
            case SystemState::RECOVERY:  // Not required
            {
                break;
            }
            default:
                break;
        }

        usleep(1000);
    }

    return 0;
}

int write_to_drive(double joint_pos[3], double joint_vel[3])
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        app_data_ptr->target_position[jnt_ctr] = joint_pos[jnt_ctr];
    }

    return 0;
}


void Jog()
{
    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
    while(commmand_data_ptr->jog_data.type == 0)
    {
        jog(commmand_data_ptr->jog_data.index, commmand_data_ptr->jog_data.dir, 0);
        if(app_data_ptr->trigger_error)
            break;
    }

    while(commmand_data_ptr->jog_data.type == 1)
    {
        jog(commmand_data_ptr->jog_data.index, commmand_data_ptr->jog_data.dir, 1);
        if(app_data_ptr->trigger_error)
            break;
    }

    commmand_data_ptr->type = CommandType::NONE;
}


double jog(int index, int dir, int type)
{
    if (type == 0) // joint space
    {
        /* code */
        double command_pos[3];
        double command_vel[3] = {0, 0, 0};
        std::copy(std::begin(app_data_ptr->actual_position), std::end(app_data_ptr->actual_position), std::begin(command_pos));
        command_pos[index] = command_pos[index] + dir * 0.001;
        write_to_drive(command_pos, command_vel);
        usleep(1000);
    }
    else // task space
    {

        // HAs to be Rewritten For Instrument

        std::vector<double> current_pos, desired_pos;
        current_pos.resize(6);
        desired_pos.resize(6);
        std::copy(std::begin(app_data_ptr->actual_position), std::end(app_data_ptr->actual_position), std::begin(current_pos));

        Eigen::MatrixXd trans_mat;
        trans_mat.resize(4, 4);

        fk_solver.computeFK(current_pos, trans_mat);

        Eigen::Vector3d eef_pos;
        eef_pos = trans_mat.topRightCorner(3, 1);

        Eigen::MatrixXd eef_orient;
        eef_orient = trans_mat.topLeftCorner(3, 3);

        // std::cout<<"eef_pos : \n"<<eef_pos<<std::endl;
        // std::cout<<"eef_orient : \n"<<eef_orient<<std::endl;

        std::cout << "index : " << index << ", dir : " << dir << std::endl;

        if (index < 3)
        {
            eef_pos[index] = eef_pos[index] + dir * 0.00005;
        }

        // std::cout<<"updated eef_pos : \n"<<eef_pos<<std::endl;

        ik_solver.computeIK(eef_pos, eef_orient, current_pos, desired_pos);

        double command_pos[6];
        double command_vel[6] = {0, 0, 0, 0, 0, 0};

        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            // std::cout<<"jnt _ctr : "<<jnt_ctr<<", current_pso : "<<current_pos[jnt_ctr]<<", desired_pos : "<<desired_pos[jnt_ctr]<<std::endl;
            command_pos[jnt_ctr] = desired_pos[jnt_ctr];
        }

        write_to_drive(command_pos, command_vel);
        usleep(1000);
    }
    return 1;
}