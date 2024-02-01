#pragma once

#include "transistionState.h"

using namespace std;

uint8_t* EthercatMaster::domainPd = NULL;

void EthercatMaster::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void EthercatMaster::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void EthercatMaster::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void EthercatMaster::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (!exitFlag)
    {
        
        ecrt_master_application_time(master, ((uint64_t) pinfo.next_period.tv_sec*1000000000 + pinfo.next_period.tv_nsec));
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        checkDomainState();

        if (counter)
        {
            counter--;
        }
        else
        {
            counter = FREQUENCY;
            do_rt_task();
        }

        handleDriveStates();
        wait_rest_of_period(&pinfo);
    }

    struct timespec wakeupTime;
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

}

void EthercatMaster::do_rt_task()
{
    checkMasterState();
    handleDriveStates();
}

void EthercatMaster::handleDriveStates()
{
    switch (system_state_data_ptr->current_state)
    {
    case DriveState::INITIALIZE:
        initializeDrives();
        break;
    case DriveState::NOT_READY_TO_SWITCH_ON:
        enableDrivesIfInitialized();
        break;
    case DriveState::SWITCHED_ON:
        handleSwitchedOnState();
        break;
    case DriveState::SWITCH_TO_OPERATION:
        handleSwitchToOperationState();
        break;
    case DriveState::OPERATION_ENABLED:
        handleOperationEnabledState();
        break;
    case DriveState::ERROR:
        handleErrorState();
        break;
    default:
        break;
    }
}

void EthercatMaster::initializeDrives()
{
    system_state_data_ptr->current_state = DriveState::NOT_READY_TO_SWITCH_ON;
}

void EthercatMaster::enableDrivesIfInitialized()
{
    if (system_state_data_ptr->initialize_drives)
    {
        system_state_data_ptr->current_state = DriveState::SWITCHED_ON;
    }
}

void EthercatMaster::handleSwitchedOnState()
{
    for (size_t jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);
        readDriveState(drive_status, jnt_ctr);

        if ((((drive_status | 65456) ^ 65471) == 0) || (((drive_status | 65456) ^ 65464) == 0))
        {
            system_state_data_ptr->current_state = DriveState::ERROR;
            std::cout << "Drive inside error \n";
            break;
        }

        if (system_state_data_ptr->current_state == DriveState::ERROR)
        {
            std::cout << "Drive inside error break didn't work \n";
        }

        uint16_t domain_command = transitionToSwitchedOn(drive_status, jnt_ctr);
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, domain_command);
        // transitionToState(CW_SWITCH_ON, jnt_ctr);
    }

    if (!(system_state_data_ptr->current_state == DriveState::ERROR))
    {
        bool drivesInSwitchedOn = driveSwitchedOn[0] && driveSwitchedOn[1] && driveSwitchedOn[2];
        std::cout << "drivesInSwitchedOn : " << drivesInSwitchedOn << std::endl;

        if (drivesInSwitchedOn)
        {
            updateJointDataInSwitchedOnState();

            system_state_data_ptr->start_safety_check = true;

            if (system_state_data_ptr->safety_check_done)
            {
                handleSafetyCheckDone();
            }
        }
        else
        {
            system_state_data_ptr->safety_check_done = false;
            system_state_data_ptr->trigger_error_mode = false;
        }
    }
}

void EthercatMaster::updateJointDataInSwitchedOnState()
{
    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        joint_data_ptr->joint_position[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].position_actual_value);
        joint_data_ptr->joint_velocity[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].velocity_actual_value);
        joint_data_ptr->joint_torque[jnt_ctr] = 0;
    }
}

void EthercatMaster::handleSafetyCheckDone()
{
    if (system_state_data_ptr->trigger_error_mode)
    {
        run_ethercat_loop = false;
    }
    else
    {
        if (system_state_data_ptr->switch_to_operation)
        {
            system_state_data_ptr->current_state = DriveState::SWITCH_TO_OPERATION;
        }
    }
}

void EthercatMaster::handleSwitchToOperationState()
{
    if (system_state_data_ptr->trigger_error_mode)
    {
        run_ethercat_loop = false;
    }
    else
    {
        for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            // uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);
            StatusWordValues driveStatus = static_cast<StatusWordValues>(EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword));
            // readDriveState(drive_status, jnt_ctr);

            // if ((((drive_status | 65456) ^ 65471) == 0) || ((drive_status | 65456) ^ 65464) == 0)
            // {
            //     system_state_data_ptr->current_state = DriveState::ERROR;
            // }

            if ( driveStatus == StatusWordValues::SW_NOT_READY_TO_SWITCH_ON ){
                system_state_data_ptr->current_state = DriveState::ERROR;
            }

            joint_data_ptr->target_position[jnt_ctr] = joint_data_ptr->joint_position[jnt_ctr];

            EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, ControlWordValues::CW_ENABLE_OPERATION);
            EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, OperationModeState::POSITION_MODE);
            EC_WRITE_S32(domainPd + driveOffset[jnt_ctr].target_position, joint_data_ptr->joint_position[jnt_ctr]);



            if (driveStatus == StatusWordValues::SW_OPERATION_ENABLED)
            {
                system_state_data_ptr->drive_enable_for_operation[jnt_ctr] = true;
            }

        }

        if (!(system_state_data_ptr->current_state == DriveState::ERROR))
        {
            updateSwitchToOperationState();
        }
    }
}

void EthercatMaster::updateSwitchToOperationState()
{
    system_state_data_ptr->current_state = system_state_data_ptr->drive_enable_for_operation[0] &&
                                           system_state_data_ptr->drive_enable_for_operation[1] &&
                                           system_state_data_ptr->drive_enable_for_operation[2]
                                       ? DriveState::OPERATION_ENABLED
                                       : DriveState::SWITCH_TO_OPERATION;
}

void EthercatMaster::handleOperationEnabledState()
{
    system_state_data_ptr->status_operation_enabled = true;

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        joint_data_ptr->joint_position[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].position_actual_value);
        joint_data_ptr->joint_velocity[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].velocity_actual_value);
        joint_data_ptr->joint_torque[jnt_ctr] = EC_READ_S16(domainPd + driveOffset[jnt_ctr].torque_actual_value);
    }

    joint_data_ptr->sterile_detection_status = true;
    joint_data_ptr->instrument_detection_status = true;

    switch (system_state_data_ptr->drive_operation_mode)
    {
    case OperationModeState::POSITION_MODE:
        handlePositionMode();
        break;
    case OperationModeState::VELOCITY_MODE:
        handleVelocityMode();
        break;
    case OperationModeState::TORQUE_MODE:
        handleTorqueMode();
        break;
    }
}

void EthercatMaster::handlePositionMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);

        if ((((drive_status | 65456) ^ 65471) == 0) || ((drive_status | 65456) ^ 65464) == 0)
        {
            system_state_data_ptr->current_state = DriveState::ERROR;
        }

        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 8);
        EC_WRITE_S32(domainPd + driveOffset[jnt_ctr].target_position, joint_data_ptr->target_position[jnt_ctr]);
    }
}

void EthercatMaster::handleVelocityMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 9);
    }
}

void EthercatMaster::handleTorqueMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 10);
    }
}

void EthercatMaster::handleErrorState()
{
    DriveState localState = DriveState::SWITCHED_ON;
    system_state_data_ptr->state = SafetyStates::ERROR;

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[jnt_ctr].statusword);

        if ((((drive_status | 65456) ^ 65471) == 0) || ((drive_status | 65456) ^ 65464) == 0)
        {
            EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, 128);
            localState = DriveState::ERROR;
            std::cout << "Drive inside error : " << jnt_ctr << "\n";
        }
    }

    system_state_data_ptr->current_state = localState;
}