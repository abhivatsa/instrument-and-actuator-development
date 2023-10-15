#include "safety_controller.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

int main()
{
    /* the size (in bytes) of shared memory object */
    const int SIZE_JointData = sizeof(JointData);
    const int SIZE_SystemStateData = sizeof(SystemStateData);
    const int SIZE_AppData = sizeof(AppData);

    int shm_fd_jointData;
    int shm_fd_systemStateData;
    int shm_fd_appData;

    /* open the shared memory object */
    shm_fd_jointData = shm_open("JointData", O_CREAT | O_RDWR, 0666);
    shm_fd_systemStateData = shm_open("SystemStateData", O_CREAT | O_RDWR, 0666);
    shm_fd_appData = shm_open("AppData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_jointData, SIZE_JointData);
    ftruncate(shm_fd_systemStateData, SIZE_SystemStateData);
    ftruncate(shm_fd_appData, SIZE_AppData);

    joint_data_ptr = static_cast<JointData *>(mmap(0, SIZE_JointData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_jointData, 0));
    system_state_data_ptr = static_cast<SystemStateData *>(mmap(0, SIZE_SystemStateData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_systemStateData, 0));
    app_data_ptr = static_cast<AppData *>(mmap(0, SIZE_AppData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_appData, 0));

    joint_data_ptr->setZero();
    system_state_data_ptr->setZero();
    // app_data_ptr->setZero();  //TODO: Why is it commented?

    // while (app_data_ptr->init_system == -1)
    // {
    //     usleep(1000);
    // }

    // system_state_data_ptr->safety_controller_enabled = true;

    // while (!system_state_data_ptr->start_safety_check)
    // {
    //     usleep(1000);
    // }

    // app_data_ptr->init_system = 1;

    // while (app_data_ptr->init_hardware_check == -1)
    // {
    //     usleep(1000);
    // }

    // while (system_state_data_ptr->current_state == DriveState::SAFETY_CONTROLLER_ENABLED &&
    //        system_state_data_ptr->current_state != DriveState::READY_FOR_OPERATION) //TODO: Redundent condition, second condition is not required.
    // {
    //     check_limits();

    //     system_state_data_ptr->safety_check_done = true;
    // }
    // //TODO: If safety check fails then it should go back to Line 46 (probably), 
    // //so that another saftey check can be performed if needed.

    // if (system_state_data_ptr->current_state != DriveState::READY_FOR_OPERATION) //TODO: Shouldn't this be an equality check? because after successful safety check `current_state` will automatically go to READY_FOR_OPERATION
    // {
    //     app_data_ptr->init_hardware_check = 1;
    // } //It will only go inside if current_state goes in ERROR

    // while (app_data_ptr->init_ready_for_operation == -1) //TODO: will it go inside? After init_hardware_check is set to 1, init_ready_for_operation will become 0 (motion planner Line 221 and 238) 
    // {
    //     usleep(1000);
    //     check_limits();
    //     read_data();
    // }

    // if (system_state_data_ptr->current_state == DriveState::OPERATION_ENALBLED)
    // {
    //     app_data_ptr->init_ready_for_operation = 1;

    //     check_limits();
    //     read_data();
    // }

    // while (system_state_data_ptr->current_state == DriveState::OPERATION_ENALBLED) //TODO: Instead of several if's and while's we can write switch inside a while, so that even if drive goes in error it can repeat the process and main function will not end.
    // {

    //     check_limits();
    //     read_data();
    //     write_data();
    // }// main function will end if ever drive goes in error


    SafetyStates state = SafetyStates::INITALIZE;

    while(1)
    {
        switch (state)
        {
            case SafetyStates::INITALIZE:
            {/* code */
                system_state_data_ptr->safety_controller_enabled = true;
                if(system_state_data_ptr->current_state == DriveState::NOT_READY_TO_SWITCH_ON)
                {
                    // send signal to motion planner
                    if(app_data_ptr->initialize_system) // motion planner initialized?
                    {
                        app_data_ptr->safety_process_status = true;
                        if(app_data_ptr->initialize_drives) // initialize drive? from  motion planner
                        {
                            state = SafetyStates::INITIALIZE_DRIVES;
                        }
                    }
                }
                break;
            }
            case SafetyStates::INITIALIZE_DRIVES:
            {
                system_state_data_ptr->initialize_drives = true;
                if(system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
                {
                    app_data_ptr->trigger_error = false;
                    app_data_ptr->drive_initialized = true;
                    if(system_state_data_ptr->start_safety_check)
                    {
                        state = SafetyStates::SAFETY_CHECK;
                    }
                }

                if(system_state_data_ptr->current_state == DriveState::ERROR)
                {
                    app_data_ptr->trigger_error = true;
                    app_data_ptr->drive_initialized = true;
                    // do someting
                }
                break;
            }
            case SafetyStates::SAFETY_CHECK:
            {
                if(check_limits())
                {
                    state = SafetyStates::READY_FOR_OPERATION;
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
                if(app_data_ptr->switch_to_operation)// switch to operation? from motion planner
                {   
                    system_state_data_ptr->switch_to_operation = true;
                    if(system_state_data_ptr.current_state == DriveState::OPERATION_ENALBLED)
                    {

                        state = SafetyStates::OPERATION;
                    }
                }
                break;
            }
            case SafetyStates::OPERATION:
            {
                if(system_state_data_ptr.current_state == DriveState::OPERATION_ENALBLED)
                {
                    app_data_ptr->operation_enalble_status = true;
                    // read write
                    check_limits();
                    read_data();
                    write_data();
                }
                else if(system_state_data_ptr.current_state == DriveState::ERROR)
                {
                    app_data_ptr->trigger_error = true;
                    state = SafetyStates::ERROR;
                    // send signal to motion planner
                }
                break;
            }
            case SafetyStates::ERROR:
            {
                if(system_state_data_ptr.current_state = DriveState::SWITCHED_ON)
                {
                    state = SafetyStates::READY_FOR_OPERATION;
                }
                break;
            }   
            default:
                break;
        }
        usleep(1000);
    }
}

void read_data()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        app_data_ptr->actual_position[jnt_ctr] = joint_data_ptr->joint_position[jnt_ctr];
        app_data_ptr->actual_velocity[jnt_ctr] = joint_data_ptr->joint_velocity[jnt_ctr];
        app_data_ptr->actual_torque[jnt_ctr] = joint_data_ptr->joint_torque[jnt_ctr];
    }

    app_data_ptr->sterile_detection = joint_data_ptr->sterile_detection_status;
    app_data_ptr->instrument_detection = joint_data_ptr->instrument_detection_status;
}

void write_data()
{

    if (!system_state_data_ptr->trigger_error_mode && system_state_data_ptr->status_operation_enabled)
    {
        for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
        {
            joint_data_ptr->target_position[jnt_ctr] = app_data_ptr->target_position[jnt_ctr];
            joint_data_ptr->target_velocity[jnt_ctr] = app_data_ptr->target_velocity[jnt_ctr];
            joint_data_ptr->target_torque[jnt_ctr] = app_data_ptr->target_torque[jnt_ctr];
        }
    }
}

bool check_limits()
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

int pos_limit_check(double *joint_pos)
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        if (fabs(joint_pos[jnt_ctr]) > 2 * M_PI / 3)
        {
            // return -1; //TODO
        }
        std::cout << "jnt_ctr : " << jnt_ctr << ", joint_pos[jnt_ctr] : " << joint_pos[jnt_ctr] << std::endl;
    }
    return 0;
}
