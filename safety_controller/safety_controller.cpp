#include "cyclicTask.h"

int main()
{
    SafetyController safety_ctrl;
    safety_ctrl.run();
    return 0;
}


SafetyController::SafetyController(){
    configureSharedMemory();
}

SafetyController::~SafetyController(){

}

void SafetyController::run(){

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Set to the desired CPU core

    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
    {
        perror("Error setting CPU affinity");
        // Handle the error appropriately based on your application's requirements
    }

    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
        // Handle the error appropriately based on your application's requirements
    }

    stackPrefault();

    // Register signal handler to gracefully stop the program
    signal(SIGINT, SafetyController::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    system_state_data_ptr->safety_controller_enabled = true;

    cyclicTask();

}

void SafetyController::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void SafetyController::signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        std::cout << "Signal received: " << signum << std::endl;
        exitFlag = 1; // Set the flag to indicate the signal was received
    }
}


void SafetyController::configureSharedMemory()
{
    int shm_fd_jointData;
    int shm_fd_systemStateData;
    int shm_fd_appData;

    createSharedMemory(shm_fd_jointData, "JointData", sizeof(JointData));
    createSharedMemory(shm_fd_systemStateData, "SystemStateData", sizeof(SystemStateData));
    createSharedMemory(shm_fd_appData, "AppData", sizeof(AppData));

    mapSharedMemory((void *&)jointDataPtr, shm_fd_jointData, sizeof(JointData));
    mapSharedMemory((void *&)systemStateDataPtr, shm_fd_systemStateData, sizeof(SystemStateData));
    mapSharedMemory((void *&)appDataPtr, shm_fd_appData, sizeof(AppData));

    initializeSharedData();
}

void SafetyController::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void SafetyController::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void SafetyController::initializeSharedData()
{
    jointDataPtr->setZero();
    systemStateDataPtr->setZero();
    app_data_ptr->setZero();
}


void SafetyController::read_data()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        app_data_ptr->actual_position[jnt_ctr] = joint_data_ptr->joint_position[jnt_ctr];
        app_data_ptr->actual_velocity[jnt_ctr] = joint_data_ptr->joint_velocity[jnt_ctr];
        app_data_ptr->actual_torque[jnt_ctr] = joint_data_ptr->joint_torque[jnt_ctr];
    }

    app_data_ptr->sterile_detection = joint_data_ptr->sterile_detection_status;
    app_data_ptr->instrument_detection = joint_data_ptr->instrument_detection_status;
}

void SafetyController::write_data()
{

    if (!system_state_data_ptr->trigger_error_mode && system_state_data_ptr->status_operation_enabled)
    {
        for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            joint_data_ptr->target_position[jnt_ctr] = app_data_ptr->target_position[jnt_ctr];
            joint_data_ptr->target_velocity[jnt_ctr] = app_data_ptr->target_velocity[jnt_ctr];
            joint_data_ptr->target_torque[jnt_ctr] = app_data_ptr->target_torque[jnt_ctr];
        }

    }
}


