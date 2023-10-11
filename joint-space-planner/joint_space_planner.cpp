#include "joint_space_planner.h"

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

    /* shared memory file descriptor */
    int shm_fd_SysData;
    double shm_fd_AppData;
    double shm_fd_ComData;

    /* open the shared memory object */
    shm_fd_SysData = shm_open("SysData", O_CREAT | O_RDWR, 0666);
    shm_fd_AppData = shm_open("AppData", O_CREAT | O_RDWR, 0666);
    shm_fd_ComData = shm_open("ComData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_SysData, SIZE_SysData);
    ftruncate(shm_fd_AppData, SIZE_AppData);
    ftruncate(shm_fd_ComData, SIZE_ComData);

    /* memory map the shared memory object */
    system_data_ptr = static_cast<SystemData *>(mmap(0, SIZE_SysData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_SysData, 0));
    app_data_ptr = static_cast<AppData *>(mmap(0, SIZE_AppData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_AppData, 0));
    commmand_data_ptr = static_cast<CommandData *>(mmap(0, SIZE_ComData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ComData, 0));

    /* the size (in bytes) of shared memory object */
    const double FORCE_DIM_SIZE = sizeof(double[20]);

    /* shared memory file descriptor */
    int shm_force_dim;

    /* open the shared memory object */
    shm_force_dim = shm_open("force_dim_data", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_force_dim, FORCE_DIM_SIZE);

    /* memory map the shared memory object */
    force_dim_ptr = static_cast<double *>(mmap(0, FORCE_DIM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_force_dim, 0));

    double start_pos[3] = {0, 0, 0};

    start_pos[0] = force_dim_ptr[0];
    start_pos[1] = force_dim_ptr[1];
    start_pos[2] = force_dim_ptr[2];

    std::cout<<"start_pos 0 : "<<start_pos[0]<<", start_pos 1 : "<<start_pos[1]<<", stgart_pos 2 : "<<start_pos[2]<<std::endl;

    bool in_operation_ = false;
    system_data_ptr->setSystemState(SystemState::POWER_OFF);
    system_data_ptr->request = 0;
    app_data_ptr->setZero();
    commmand_data_ptr->type = CommandType::NONE;

    Eigen::MatrixXd trans_mat;
    Eigen::Vector3d eef_pos;
    Eigen::Matrix3d eef_orient;

    bool hand_Controller_switch = false;

    while (1)
    {

        if (!in_operation_ && system_data_ptr->request != 0)
        {
            changeSystemState();
        }

        if (commmand_data_ptr->type != CommandType::NONE)
        {

            if (!in_operation_)
            {

                if (commmand_data_ptr->type == CommandType::JOG)
                {
                    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
                    while (commmand_data_ptr->jog_data.type == 0)
                        jog(commmand_data_ptr->jog_data.index, commmand_data_ptr->jog_data.dir, 0);

                    while (commmand_data_ptr->jog_data.type == 1)
                        jog(commmand_data_ptr->jog_data.index, commmand_data_ptr->jog_data.dir, 1);
                    
                    commmand_data_ptr->type = CommandType::NONE;
                }
                else if (commmand_data_ptr->type == CommandType::HAND_CONTROL)
                {
                    /* code */
                    std::cout << "Hand Control Enabled \n";
                    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
                    if (!hand_Controller_switch){

                        std::vector<double> current_pos, desired_pos;
                        current_pos.resize(6);
                        desired_pos.resize(6);
                        std::copy(std::begin(app_data_ptr->actual_position), std::end(app_data_ptr->actual_position), std::begin(current_pos));

                        
                        trans_mat.resize(4, 4);

                        fk_solver.computeFK(current_pos, trans_mat);

                        
                        eef_pos = trans_mat.topRightCorner(3, 1);

                        
                        eef_orient = trans_mat.topLeftCorner(3, 3);

                        std::cout<<"eef_pos : \n"<<eef_pos<<std::endl;

                        hand_Controller_switch = true;

                    }
                    else{
                        hand_control_jog(start_pos, eef_pos, eef_orient);
                    }
                    
                }
                else if (commmand_data_ptr->type == CommandType::STERILE_ENGAGEMENT)
                {
                    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
                    /* code */
                }
                else if (commmand_data_ptr->type == CommandType::INSTRUMENT_ENGAGEMENT)
                {
                    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
                    /* code */
                }
                else
                {
                    app_data_ptr->drive_operation_mode = OperationModeState::POSITION_MODE;
                    in_operation_ = true;
                    system_data_ptr->setSystemState(SystemState::IN_EXECUTION);
                    // move point to point
                    double init_pos[6];
                    double final_pos[6];
                    std::copy(std::begin(app_data_ptr->actual_position), std::end(app_data_ptr->actual_position), std::begin(init_pos));
                    std::copy(std::begin(commmand_data_ptr->move_to_data.goal_position), std::end(commmand_data_ptr->move_to_data.goal_position), std::begin(final_pos));
                    pt_to_pt_mvmt(init_pos, final_pos);

                    commmand_data_ptr->type = CommandType::NONE;
                }
                in_operation_ = false;
                system_data_ptr->setSystemState(SystemState::READY);
            }
            // commmand_data_ptr->type = CommandType::NONE;
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

int changeSystemState()
{
    std::cout << "Change state called\n";
    if (system_data_ptr->request == 1)
    {
        system_data_ptr->setSystemState(SystemState::INITIALIZING_SYSTEM);
        sleep(2);
        system_data_ptr->setSystemState(SystemState::HARWARE_CHECK);
        sleep(2);
        system_data_ptr->setSystemState(SystemState::READY);
    }
    else if (system_data_ptr->request == -1)
    {
        system_data_ptr->setSystemState(SystemState::POWER_OFF);
    }

    system_data_ptr->request = 0;
    return 1;
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

        // Eigen::MatrixXd jacobian;
        // jac_solver.computeJacobianInline(current_pos, jacobian);
        // Eigen::VectorXd cart_vel;
        // cart_vel.resize(6);
        // cart_vel[index] = 0.001;
        // Eigen::VectorXd del_pos = jacobian*cart_vel;

        // double command_pos[6];
        // double command_vel[6] = {0,0,0,0,0,0};

        // for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++){
        //     command_pos[jnt_ctr] = current_pos[jnt_ctr] + del_pos[jnt_ctr];
        // }

        write_to_drive(command_pos, command_vel);
        usleep(1000);
    }
    return 1;
}

double hand_control_jog(double start_pos[3], Eigen::Vector3d& eef_pos, Eigen::Matrix3d& eef_orient)
{
    // std::cout<<"eef_orient : \n"<<eef_orient<<std::endl;

    std::vector<double> current_pos, desired_pos;
    current_pos.resize(6);
    desired_pos.resize(6);
    std::copy(std::begin(app_data_ptr->actual_position), std::end(app_data_ptr->actual_position), std::begin(current_pos));


    Eigen::Vector3d eef_pos_new;

    eef_pos_new[0] = eef_pos[0] + 2*(force_dim_ptr[0] - start_pos[0]);
    eef_pos_new[1] = eef_pos[1] + 2*(force_dim_ptr[1] - start_pos[1]);
    eef_pos_new[2] = eef_pos[2] + 2*(force_dim_ptr[2] - start_pos[2]);

    std::cout<<"updated eef_pos : \n"<<eef_pos_new<<std::endl;

    ik_solver.computeIK(eef_pos_new, eef_orient, current_pos, desired_pos);

    double command_pos[6];
    double command_vel[6] = {0, 0, 0, 0, 0, 0};

    for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        // std::cout<<"jnt _ctr : "<<jnt_ctr<<", current_pso : "<<current_pos[jnt_ctr]<<", desired_pos : "<<desired_pos[jnt_ctr]<<std::endl;
        command_pos[jnt_ctr] = desired_pos[jnt_ctr];
    }

    write_to_drive(command_pos, command_vel);
    usleep(1000);

    return 0;

}

int pt_to_pt_mvmt(double ini_pos[3], double final_pos[3])
{

    int num_joints = 3;

    double joint_vel[3] = {0.5, 0.5, 0.5};
    double joint_acc[3] = {0.5, 0.5, 0.5};

    // taking care of vel and acceleration sign
    for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {

        if (final_pos[jnt_ctr] < ini_pos[jnt_ctr])
        {
            joint_vel[jnt_ctr] = -joint_vel[jnt_ctr];
            joint_acc[jnt_ctr] = -joint_acc[jnt_ctr];
        }
    }

    // computing minimum time
    double max_time, global_acc_time, global_cruise_time;
    max_time = 0;

    for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double acc_dist, cruise_dist;

        // std::cout<<"joint_vel : "<<joint_vel[jnt_ctr]<<", joint_acc : "<<joint_acc[jnt_ctr]<<std::endl;
        // std::cout<<"final_pos : "<<final_pos[jnt_ctr]<<", ini_pos : "<<ini_pos[jnt_ctr]<<std::endl;

        double acc_time, cruise_time, local_time;

        if (fabs(final_pos[jnt_ctr] - ini_pos[jnt_ctr]) < fabs(joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (joint_acc[jnt_ctr])))
        {

            double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];

            if (fabs(joint_diff) > 1e-4)
            {
                joint_vel[jnt_ctr] = 0.99 * joint_vel[jnt_ctr] / fabs(joint_vel[jnt_ctr]) * sqrt(2 * fabs(joint_diff * joint_acc[jnt_ctr]));

                acc_dist = joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (2 * joint_acc[jnt_ctr]);
                acc_time = joint_vel[jnt_ctr] / joint_acc[jnt_ctr];
                cruise_time = (joint_diff - 2 * acc_dist) / joint_vel[jnt_ctr];

                local_time = 2 * acc_time + cruise_time;
            }
            else
            {
                local_time = 0;
                acc_time = 0;
                cruise_time = 0;
            }
        }
        else
        {

            double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];
            acc_dist = joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (2 * joint_acc[jnt_ctr]);
            acc_time = joint_vel[jnt_ctr] / joint_acc[jnt_ctr];
            cruise_time = (joint_diff - 2 * acc_dist) / joint_vel[jnt_ctr];
            local_time = 2 * acc_time + cruise_time;
        }

        // std::cout<<"local_time : "<<local_time<<std::endl;

        if (local_time > max_time)
        {
            max_time = local_time;
            global_acc_time = acc_time;
            global_cruise_time = cruise_time;
        }
    }

    // computing joint_vel and joint acc
    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];
        joint_acc[jnt_ctr] = joint_diff / (global_acc_time * global_acc_time + global_acc_time * global_cruise_time);
    }

    double t = 0;
    double current_pos[3] = {0};
    double current_vel[3] = {0};
    double current_acc[3] = {0};

    // std::cout << "max_time : " << max_time << std::endl;

    while (t < max_time)
    {
        /* int clock_gettime( clockid_t clock_id, struct
     timespec *tp ); The clock_gettime() function gets
     the current time of the clock specified by clock_id,
     and puts it into the buffer  pointed to by tp.tp
     parameter points to a structure containing
     atleast the following members:
     struct timespec {
               time_t   tv_sec;        // seconds
               long     tv_nsec;       // nanoseconds
           };
    clock id = CLOCK_REALTIME, CLOCK_PROCESS_CPUTIME_ID,
               CLOCK_MONOTONIC ...etc
    CLOCK_REALTIME : clock  that  measures real (i.e., wall-clock) time.
    CLOCK_PROCESS_CPUTIME_ID : High-resolution per-process timer
                               from the CPU.
    CLOCK_MONOTONIC : High resolution timer that is unaffected
                      by system date changes (e.g. NTP daemons).  */
        struct timespec start, end;

        clock_gettime(CLOCK_MONOTONIC, &start);

        // unsync the I/O of C and C++.
        ios_base::sync_with_stdio(false);

        t = t + 0.002;

        if ((fabs(t - max_time) < 1e-4) || t > max_time)
        {
            t = max_time;
        }

        // std::cout<<t<<",";

        for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
        {

            if (t < global_acc_time)
            {
                current_acc[jnt_ctr] = joint_acc[jnt_ctr];
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * t;
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + 0.5 * joint_acc[jnt_ctr] * t * t;
            }
            else if (t < global_acc_time + global_cruise_time)
            {
                current_acc[jnt_ctr] = 0;
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * global_acc_time;
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + 0.5 * joint_acc[jnt_ctr] * global_acc_time * global_acc_time + current_vel[jnt_ctr] * (t - global_acc_time);
            }
            else if (t < 2 * global_acc_time + global_cruise_time)
            {
                current_acc[jnt_ctr] = -joint_acc[jnt_ctr];
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * (max_time - t);
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + final_pos[jnt_ctr] - ini_pos[jnt_ctr] - 0.5 * joint_acc[jnt_ctr] * (max_time - t) * (max_time - t);
            }
            else
            {
                current_acc[jnt_ctr] = 0;
                current_vel[jnt_ctr] = 0;
                current_pos[jnt_ctr] = final_pos[jnt_ctr];
            }

            // if (jnt_ctr != 5)
            // {
            //     // std::cout<<current_pos[jnt_ctr]<<","<<current_vel[jnt_ctr]<<","<<current_acc[jnt_ctr]<<"," ;
            // }
            // else
            // {
            //     // std::cout<<current_pos[jnt_ctr]<<","<<current_vel[jnt_ctr]<<","<<current_acc[jnt_ctr]<<"\n";
            // }

            // if (jnt_ctr == 5){
            //     std::cout<<current_pos[jnt_ctr]<<","<<current_vel[jnt_ctr]<<","<<current_acc[jnt_ctr]<<"\n";
            // }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);

        // Calculating total time taken by the program.
        double time_taken;
        time_taken = (end.tv_sec - start.tv_sec) * 1e9;
        time_taken = (time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;

        // cout << "Time taken by program is : " << fixed
        //      << time_taken << setprecision(9);
        // cout << " sec" << endl;

        write_to_drive(current_pos, current_vel);

        usleep(2000);
    }

    return 0;
}
