#ifndef INSTRUMENT_JOG_H
#define INSTRUMENT_JOG_H

#include "pt_to_pt_planner.h"
#include "instrument_motion_planner.h"

/* pointer to shared memory object */
motion_planning::ForwardKinematics fk_solver;
motion_planning::IK6AxisInline ik_solver;
motion_planning::Jacobian jac_solver(1);

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

#endif