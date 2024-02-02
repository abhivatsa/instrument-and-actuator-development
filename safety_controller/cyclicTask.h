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
    switch (system_state_data_ptr->safety_state)
    {
    case SafetyStates::INITIALIZE:
        if (system_state_data_ptr->drive_state == DriveState::INITIALIZE)
        {
            // send signal to motion planner
            if (app_data_ptr->initialize_system) // motion planner initialized?
            {
                app_data_ptr->safety_process_status = true;
                if (app_data_ptr->initialize_drives) // initialize drive? from  motion planner
                {
                    // system_state_data_ptr->state = SafetyStates::INITIALIZE_DRIVES;
                    system_state_data_ptr->initialize_drives = true;
                    if (system_state_data_ptr->drive_state == DriveState::SWITCHED_ON)
                    {
                        app_data_ptr->trigger_error = false;
                        app_data_ptr->drive_initialized = true;
                        if (system_state_data_ptr->start_safety_check)
                        {
                            system_state_data_ptr->safety_state = SafetyStates::SAFETY_CHECK;

                            if (check_limits())
                            {
                                system_state_data_ptr->safety_state = SafetyStates::READY_FOR_OPERATION;
                                system_state_data_ptr->safety_check_done = true;
                            }
                            else
                            {
                                system_state_data_ptr->safety_check_done = false;
                            }
                        }
                    }

                    if (system_state_data_ptr->drive_state == DriveState::ERROR)
                    {
                        app_data_ptr->trigger_error = true;
                        app_data_ptr->drive_initialized = true;
                        // do someting
                    }
                }
            }
        }
        break;
    case SafetyStates::READY_FOR_OPERATION:
        check_limits();
        read_data();

        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            app_data_ptr->target_position[jnt_ctr] = app_data_ptr->actual_position[jnt_ctr];
            app_data_ptr->target_velocity[jnt_ctr] = app_data_ptr->actual_velocity[jnt_ctr];
            app_data_ptr->target_torque[jnt_ctr] = app_data_ptr->actual_torque[jnt_ctr];
        }

        if (app_data_ptr->switch_to_operation) // switch to operation? from motion planner
        {
            system_state_data_ptr->switch_to_operation = true;
            if (system_state_data_ptr->drive_state == DriveState::OPERATION_ENABLED)
            {
                system_state_data_ptr->safety_state = SafetyStates::OPERATION;
            }
        }
        break;
    case SafetyStates::OPERATION:
        if (system_state_data_ptr->drive_state == DriveState::OPERATION_ENABLED)
        {
            app_data_ptr->operation_enable_status = true;
            // read write
            read_data();
            if (check_limits()){
                write_data();
            }
            
        }
        else if (system_state_data_ptr->drive_state == DriveState::ERROR)
        {
            app_data_ptr->trigger_error = true;
            system_state_data_ptr->safety_state = SafetyStates::ERROR;
            // send signal to motion planner
        }
        break;
    case SafetyStates::ERROR:
        app_data_ptr->setZero();
        if (system_state_data_ptr->drive_state == DriveState::SWITCHED_ON)
        {
            system_state_data_ptr->safety_state = SafetyStates::READY_FOR_OPERATION;
        }
        break;
    case SafetyStates::RECOVERY:
        app_data_ptr->setZero();
        if (system_state_data_ptr->drive_state == DriveState::SWITCHED_ON)
        {
            system_state_data_ptr->safety_state = SafetyStates::READY_FOR_OPERATION;
        }
        break;
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

    joint_pos_limit_check();
    joint_vel_limit_check();
    joint_torq_limit_check();

    return !system_state_data_ptr->trigger_error_mode;
}

void SafetyController::joint_pos_limit_check()
{
    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_pos(joint_data_ptr->joint_position[jnt_ctr], jnt_ctr)) > pos_limit[jnt_ctr])
        {
            system_state_data_ptr->trigger_error_mode = true;
        }
    }
}

void SafetyController::joint_vel_limit_check()
{

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_velocity(joint_data_ptr->joint_velocity[jnt_ctr], jnt_ctr)) > vel_limit[jnt_ctr])
        {
            system_state_data_ptr->trigger_error_mode = true;
        }
    }
}

void SafetyController::joint_torq_limit_check()
{
    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_torque(joint_data_ptr->joint_torque[jnt_ctr], jnt_ctr)) > torque_limit[jnt_ctr])
        {
            system_state_data_ptr->trigger_error_mode = true;
        }
    }
}