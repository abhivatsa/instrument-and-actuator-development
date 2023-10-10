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

    int shm_fd_jointData;
    int shm_fd_systemStateData;

    /* open the shared memory object */
    shm_fd_jointData = shm_open("JointData", O_CREAT | O_RDWR, 0666);
    shm_fd_systemStateData = shm_open("SystemStateData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_jointData, SIZE_JointData);
    ftruncate(shm_fd_systemStateData, SIZE_SystemStateData);

    joint_data_ptr = static_cast<JointData *>(mmap(0, SIZE_JointData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_jointData, 0));
    system_state_data_ptr = static_cast<SystemStateData *>(mmap(0, SIZE_SystemStateData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_systemStateData, 0));
    
    joint_data_ptr->setZero();
    system_state_data_ptr->setZero();

    system_state_data_ptr->safety_controller_enabled = true;

    while (true){

        while(!system_state_data_ptr->start_safety_check){
            usleep(1000);
        }

        if(pos_limit_check(joint_data_ptr->joint_position) == 0)
        {
            system_state_data_ptr->trigger_error_mode = false;
        }
        else{
            system_state_data_ptr->trigger_error_mode = true;
        }
        system_state_data_ptr->safety_check_done = true;

        // while(system_state_data_ptr->safety_controller_enabled){
        //     if (pos_limit_check(joint_data_ptr->joint_position) == -1){
        //         system_state_data_ptr->trigger_error_mode = true;
        //         system_state_data_ptr->trigger_operation_enabled = false;
        //         break;
        //     }
        //     system_state_data_ptr->trigger_operation_enabled = true;
        // }

        // while (system_state_data_ptr->trigger_error_mode == true){

        // }

    }

    

}

int pos_limit_check(double* joint_pos){

    for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++){
        if (fabs(joint_pos[jnt_ctr]) > 2*M_PI/3)
        {
            // return -1;
        }
        std::cout<<"jnt_ctr : "<<jnt_ctr<<", joint_pos[jnt_ctr] : "<<joint_pos[jnt_ctr]<<std::endl;
    }
    return 0;
}

