#pragma once
#include "master.h"

void EthercatMaster::transitionToState(ControlWordValues value, int jnt_ctr){
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, value);
}

uint16_t EthercatMaster::transitionToSwitchedOn(uint16_t status, int joint_num)
{
    // cout << "update_state" << endl;
    uint16_t command = 6;
    if (((status | 65456) ^ 65456) == 0)
    {
        // std::cout<<"Not ready to switch on, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65456) ^ 65520) == 0)
    {
        // std::cout<<"Switch on Disabled, joint num : "<<joint_num<<std::endl;
        command = 6;
    }
    else if (((status | 65424) ^ 65457) == 0)
    {
        // std::cout<<"Ready to Switch on, joint num : "<<joint_num<<std::endl;
        command = 7;
    }
    else if (((status | 65424) ^ 65459) == 0)
    {
        // std::cout<<"Switched On, joint num : "<<joint_num<<std::endl;
        // command = 15;
        driveSwitchedOn[joint_num] = true;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

uint16_t EthercatMaster::transitionToOperationEnabled(uint16_t status, int joint_num)
{
    uint16_t command = 6;
    if (((status | 65424) ^ 65459) == 0)
    {
        std::cout << "Switched On, joint num : " << joint_num << std::endl;
        command = 15;
    }
    else if (((status | 65424) ^ 65463) == 0)
    {
        // printf(" Operation Enabled \n");
        // Operation Enabled
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

uint16_t EthercatMaster::transitionToFaultState(uint16_t status, int joint_num)
{
    // cout << "update_state" << endl;
    uint16_t command = 6;
    if (((status | 65456) ^ 65471) == 0)
    {
        // Fault Reaction Active
        std::cout << "Fault reaction active" << std::endl;
    }
    else if (((status | 65456) ^ 65464) == 0)
    {
        // Fault
        std::cout << "Fault" << std::endl;
        command = 15;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}