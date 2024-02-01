#pragma once
#include "master.h"

void EthercatMaster::transitionToState(ControlWordValues value, int jnt_ctr)
{
    EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, value);
}

StatusWordValues EthercatMaster::readDriveState(int joint_num)
{
    uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[joint_num].statusword);

    // Check if the drive is in the Operation Enable state
    if (drive_status & 0x1000)
    {
        // Check if the drive is in the Quick Stop state
        if (drive_status & 0x0800)
        {
            return StatusWordValues::SW_QUICK_STOP_ACTIVE;
        }

        return StatusWordValues::SW_OPERATION_ENABLED;
    }

    // Check if the drive is in the Fault state
    else if (drive_status & 0x0400)
    {
        // Check if the Fault is Acknowledged
        if (drive_status & 0x0200)
        {
            return StatusWordValues::SW_FAULT_REACTION_ACTIVE;
        }

        return StatusWordValues::SW_FAULT;
    }

    // Check if the drive is in the Switched On state
    else if (drive_status & 0x0010)
    {

        // Check if the drive is in the Ready to Switch On state
        if (drive_status & 0x0020)
        {
            return StatusWordValues::SW_READY_TO_SWITCH_ON;
        }

        return StatusWordValues::SW_SWITCHED_ON;
    }

    // Check if the drive is in the Not Ready to Switch On state
    else if (drive_status & 0x0008)
    {
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }

    // Check if the drive is in the Switch On Disabled state
    else if (drive_status & 0x0007)
    {
        return StatusWordValues::SW_SWITCH_ON_DISABLED;
    }
    else
    {
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }
}