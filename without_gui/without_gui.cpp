#include "without_gui.h"
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
    const int SIZE_SysData = sizeof(SystemData);
    const int SIZE_AppData = sizeof(AppData);
    const int SIZE_ComData = sizeof(CommandData);

    int shm_fd_SysData;
    double shm_fd_AppData;
    double shm_fd_ComData;

    /* open the shared memory object */
    shm_fd_SysData = shm_open("SysData", O_CREAT | O_RDWR, 0666);
    shm_fd_AppData = shm_open("AppData", O_CREAT | O_RDWR, 0666);
    shm_fd_ComData = shm_open("ComData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_SysData, SIZE_SysData);
    ftruncate(shm_fd_AppData, SIZE_AppData);
    ftruncate(shm_fd_ComData, SIZE_ComData);

    system_data_ptr = static_cast<SystemData *>(mmap(0, SIZE_SysData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_SysData, 0));
    app_data_ptr = static_cast<AppData *>(mmap(0, SIZE_AppData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_AppData, 0));
    commmand_data_ptr = static_cast<CommandData *>(mmap(0, SIZE_ComData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ComData, 0));


    system_data_ptr->setSystemState(SystemState::POWER_OFF);
    system_data_ptr->request = 0;
    app_data_ptr->setZero();
    commmand_data_ptr->type = CommandType::NONE;

    sleep(2);

    system_data_ptr->request = 1;

    while (!(app_data_ptr->operation_enable_status)){
        sleep(1);
    }

    commmand_data_ptr->setSterileEngagement();

}
