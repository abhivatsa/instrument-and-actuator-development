#pragma once

#include "instrument_motion_planner.h"

double InstrumentMotionPlanner::sterile_engagement()
{

    double ini_pos[NUM_JOINTS], final_pos[NUM_JOINTS];

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        ini_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
        final_pos[jnt_ctr] = ini_pos[jnt_ctr] + (4 * M_PI + 0.02);
    }

    bool start_homing = true;
    double sterile_time = 0;
    double command_pos[NUM_JOINTS];
    bool do_home[NUM_JOINTS] = {true, true, true, true};
    std::copy(std::begin(appDataPtr->actual_position), std::end(appDataPtr->actual_position), std::begin(command_pos));

    std::cout << "Outside start_homing : " << start_homing << ", !exitFlag : " << (!exitFlag) << std::endl;

    while (start_homing && !exitFlag)
    {

        sterile_time = sterile_time + 0.001;

        int homing_ctr = 0;

        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {

            // std::cout<<"do_home["<<jnt_ctr<<"]"<<do_home[jnt_ctr]<<std::endl;

            if (do_home[jnt_ctr])
            {
                command_pos[jnt_ctr] = command_pos[jnt_ctr] + 0.001;

                if ((fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5 && (fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0)))
                {
                    std::cout << "fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5 : " << (fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5) << std::endl;
                    std::cout << "(fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0 ) : " << (fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0) << std::endl;
                    std::cout << "(command_pos[jnt_ctr] > final_pos[jnt_ctr]) : " << (command_pos[jnt_ctr] > final_pos[jnt_ctr]) << std::endl;

                    do_home[jnt_ctr] = false;
                }
            }
            else
            {
                command_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
                homing_ctr++;
            }

            // std::cout<<"do_home["<<jnt_ctr<<"]"<<do_home[jnt_ctr]<<", command_pos: "<<command_pos[jnt_ctr]<<", actual_pos : "<<appDataPtr->actual_position[jnt_ctr]<<"actual torq : "<<appDataPtr->actual_torque[jnt_ctr]<<std::endl;
        }

        if (homing_ctr == NUM_JOINTS)
        {
            start_homing = false;
        }

        write_to_drive(command_pos);

        // std::cout<<"start_homing : "<<start_homing<<", !exitFlag : "<<(!exitFlag)<<", homing_ctr : "<<homing_ctr<<std::endl;

        usleep(1000);
    }

    commandDataPtr->type = CommandType::NONE;

    return 0;
}