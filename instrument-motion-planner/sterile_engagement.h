#pragma once

#include "pt_to_pt_planner.h"


double sterile_engagement()
{

    double ini_pos[4], final_pos[4];

    for (unsigned int jnt_ctr = 0; jnt_ctr < 4; jnt_ctr++)
    {
        ini_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
        final_pos[jnt_ctr] = ini_pos[jnt_ctr] + (2 * M_PI + 0.02);
    }

    pt_to_pt_mvmt(ini_pos, final_pos);

    return 0;
}