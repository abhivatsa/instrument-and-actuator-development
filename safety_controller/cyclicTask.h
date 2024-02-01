#pragma once

#include "safety_controller.h"

void SafetyController::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (!exitFlag)
    {
        do_rt_task();
        wait_rest_of_period(&pinfo);
    }
}

void SafetyController::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void SafetyController::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void SafetyController::do_rt_task()
{
    switch (system_state_data_ptr->state)
    {
    case SafetyStates::INITIALIZE:
        break;
    case SafetyStates::INITIALIZE_DRIVES:
        break;
    case SafetyStates::SAFETY_CHECK:
        break;
    case SafetyStates::READY_FOR_OPERATION:
        break;
    case SafetyStates::OPERATION:
        break;
    case SafetyStates::ERROR:
        break;
    }
    switch (system_state_data_ptr->state)
    {
    case SafetyStates::INITIALIZE:
    { /* code */
        
        if (system_state_data_ptr->current_state == DriveState::INITIALIZE)
        {
            // send signal to motion planner
            if (app_data_ptr->initialize_system) // motion planner initialized?
            {
                app_data_ptr->safety_process_status = true;
                if (app_data_ptr->initialize_drives) // initialize drive? from  motion planner
                {
                    // system_state_data_ptr->state = SafetyStates::INITIALIZE_DRIVES;
                    if (system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
                    {
                        app_data_ptr->trigger_error = false;
                        app_data_ptr->drive_initialized = true;
                        if (system_state_data_ptr->start_safety_check)
                        {
                            system_state_data_ptr->state = SafetyStates::SAFETY_CHECK;
                        }
                    }

                }
            }
        }
        break;
    }
    case SafetyStates::INITIALIZE_DRIVES:
    {
        std::cout << "system_state_data_ptr->start_safety_check : " << system_state_data_ptr->start_safety_check << std::endl;
        system_state_data_ptr->initialize_drives = true;
        if (system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
        {
            app_data_ptr->trigger_error = false;
            app_data_ptr->drive_initialized = true;
            if (system_state_data_ptr->start_safety_check)
            {
                system_state_data_ptr->state = SafetyStates::SAFETY_CHECK;
            }
        }

        if (system_state_data_ptr->current_state == DriveState::ERROR)
        {
            app_data_ptr->trigger_error = true;
            app_data_ptr->drive_initialized = true;
            // do someting
        }
        break;
    }
    case SafetyStates::SAFETY_CHECK:
    {
        if (check_limits())
        {
            system_state_data_ptr->state = SafetyStates::READY_FOR_OPERATION;
            app_data_ptr->trigger_error = false;
        }
        else
        {
            // safet check failed
            app_data_ptr->trigger_error = true;
        }
        app_data_ptr->safety_check_done = true;
        system_state_data_ptr->safety_check_done = true;

        break;
    }
    case SafetyStates::READY_FOR_OPERATION:
    {
        check_limits();
        read_data();

        for (int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            app_data_ptr->target_position[jnt_ctr] = joint_data_ptr->joint_position[jnt_ctr];
            app_data_ptr->target_torque[jnt_ctr] = joint_data_ptr->joint_torque[jnt_ctr];
        }

        if (app_data_ptr->switch_to_operation) // switch to operation? from motion planner
        {
            system_state_data_ptr->switch_to_operation = true;
            if (system_state_data_ptr->current_state == DriveState::OPERATION_ENABLED)
            {
                system_state_data_ptr->state = SafetyStates::OPERATION;
            }
        }
        break;
    }
    case SafetyStates::OPERATION:
    {
        if (system_state_data_ptr->current_state == DriveState::OPERATION_ENABLED)
        {
            app_data_ptr->operation_enable_status = true;
            // read write
            check_limits();
            read_data();
            write_data();
        }
        else if (system_state_data_ptr->current_state == DriveState::ERROR)
        {
            app_data_ptr->trigger_error = true;
            system_state_data_ptr->state = SafetyStates::ERROR;
            // send signal to motion planner
        }
        break;
    }
    case SafetyStates::ERROR:
    {
        app_data_ptr->setZero();
        if (system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
        {
            system_state_data_ptr->state = SafetyStates::READY_FOR_OPERATION;
        }
        break;
    }
    default:
        break;
    }
}

void SafetyController::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

bool SafetyController::check_limits()
{

    if (pos_limit_check(joint_data_ptr->joint_position) == 0)
    {
        system_state_data_ptr->trigger_error_mode = false;
    }
    else
    {
        system_state_data_ptr->trigger_error_mode = true;
    }

    return !system_state_data_ptr->trigger_error_mode;
}

int SafetyController::pos_limit_check(double *joint_pos)
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        if (fabs(joint_pos[jnt_ctr]) > 2 * M_PI / 3)
        {
            // return -1; //TODO
        }
    }
    return 0;
}