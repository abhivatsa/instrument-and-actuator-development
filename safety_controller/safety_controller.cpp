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
    // app_data_ptr->setZero();

    while (app_data_ptr->init_system == -1)
    {
        usleep(1000);
    }

    system_state_data_ptr->safety_controller_enabled = true;

    while (!system_state_data_ptr->start_safety_check)
    {
        usleep(1000);
    }

    app_data_ptr->init_system = 1;

    while (app_data_ptr->init_hardware_check == -1)
    {
        usleep(1000);
    }

    while (system_state_data_ptr->current_state == DriveState::SAFETY_CONTROLLER_ENABLED &&
           system_state_data_ptr->current_state != DriveState::READY_FOR_OPERATION)
    {
        check_limits();

        system_state_data_ptr->safety_check_done = true;
    }

    if (system_state_data_ptr->current_state != DriveState::READY_FOR_OPERATION)
    {
        app_data_ptr->init_hardware_check = 1;
    }

    while (app_data_ptr->init_ready_for_operation == -1)
    {
        usleep(1000);
        check_limits();
        read_data();
    }

    if (system_state_data_ptr->current_state == DriveState::OPERATION_ENALBLED)
    {
        app_data_ptr->init_ready_for_operation = 1;

        check_limits();
        read_data();
    }

    while (system_state_data_ptr->current_state == DriveState::OPERATION_ENALBLED)
    {

        check_limits();
        read_data();
        write_data();
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

void check_limits()
{

    if (pos_limit_check(joint_data_ptr->joint_position) == 0)
    {
        system_state_data_ptr->trigger_error_mode = false;
    }
    else
    {
        system_state_data_ptr->trigger_error_mode = true;
    }
}

int pos_limit_check(double *joint_pos)
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < 3; jnt_ctr++)
    {
        if (fabs(joint_pos[jnt_ctr]) > 2 * M_PI / 3)
        {
            // return -1;
        }
        std::cout << "jnt_ctr : " << jnt_ctr << ", joint_pos[jnt_ctr] : " << joint_pos[jnt_ctr] << std::endl;
    }
    return 0;
}
