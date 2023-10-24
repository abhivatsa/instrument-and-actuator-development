#ifndef STERILE_ENGAGEMENT_H
#define STERILE_ENGAGEMENT_H

#include "pt_to_pt_planner.h"
#include "instrument_motion_planner.h"

double sterile_engagement()
{

    double ini_pos[3], final_pos[3];

    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        ini_pos[jnt_ctr] = app_data_ptr->actual_position[jnt_ctr];
        final_pos[jnt_ctr] = ini_pos[jnt_ctr] + (2 * M_PI + 0.02);
    }

    pt_to_pt_mvmt(ini_pos, final_pos);
    
    return 0;
}

#endif