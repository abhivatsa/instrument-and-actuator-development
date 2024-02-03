#include "cyclicTask.h"
using namespace std;

int main(int argc, char **argv)
{
    // Create an instance of the Master class
    EthercatMaster ecat_master;

    // Run the main functionality of your program
    ecat_master.run();

    return 0; // Indicate successful program execution
}

EthercatMaster::EthercatMaster()
{
    master = ecrt_request_master(0);
    if (!master)
    {
        throw std::runtime_error("Failed to retrieve Master.");
    }

    /** Creates a new process data domain.
     *
     * For process data exchange, at least one process data domain is needed.
     * This method creates a new process data domain and returns a pointer to the
     * new domain object. This object can be used for registering PDOs and
     * exchanging them in cyclic operation.
     *
     * This method allocates memory and should be called in non-realtime context
     * before ecrt_master_activate().
     *
     * \return Pointer to the new domain on success, else NULL.
     */

    domain = ecrt_master_create_domain(master);
    if (!domain)
    {
        throw std::runtime_error("Failed to create process data domain.");
    }

    for (uint16_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        ec_slave_config_t *sc;

        if (!(sc = ecrt_master_slave_config(master, 0, jnt_ctr, ingeniaDenalliXcr)))
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return;
        }

        pdoMapping(sc);

        ec_pdo_entry_reg_t domain_regs[] = {

            {0, jnt_ctr, ingeniaDenalliXcr, 0x6041, 0, &driveOffset[jnt_ctr].statusword},                // 6041 0 statusword
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6061, 0, &driveOffset[jnt_ctr].mode_of_operation_display}, // 6061 0 mode_of_operation_display
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6064, 0, &driveOffset[jnt_ctr].position_actual_value},     // 6064 0 pos_act_val
            {0, jnt_ctr, ingeniaDenalliXcr, 0x606C, 0, &driveOffset[jnt_ctr].velocity_actual_value},     // 606C 0 vel_act_val
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6077, 0, &driveOffset[jnt_ctr].torque_actual_value},       // 6077 0 torq_act_val
            {0, jnt_ctr, ingeniaDenalliXcr, 0x2600, 0, &driveOffset[jnt_ctr].digital_input_value},       // 60FD 0 digital_input_value
            {0, jnt_ctr, ingeniaDenalliXcr, 0x603F, 0, &driveOffset[jnt_ctr].error_code},                // 603F 0 digital_input_value
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6040, 0, &driveOffset[jnt_ctr].controlword},               // 6040 0 control word
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6060, 0, &driveOffset[jnt_ctr].modes_of_operation},        // 6060 0 mode_of_operation
            {0, jnt_ctr, ingeniaDenalliXcr, 0x6071, 0, &driveOffset[jnt_ctr].target_torque},             // 6071 0 target torque
            {0, jnt_ctr, ingeniaDenalliXcr, 0x607A, 0, &driveOffset[jnt_ctr].target_position},           // 607A 0 target position
            {0, jnt_ctr, ingeniaDenalliXcr, 0x60FF, 0, &driveOffset[jnt_ctr].target_velocity},           // 60FF 0 target velocity
            {0, jnt_ctr, ingeniaDenalliXcr, 0x60B2, 0, &driveOffset[jnt_ctr].torque_offset},
            {0, jnt_ctr, ingeniaDenalliXcr, 0x60B1, 0, &driveOffset[jnt_ctr].velocity_offset}, // 60B2 0 torque offset
            {}};

        ecrt_slave_config_dc(sc, 0x0300, 1000000, 0, 0, 0);

        /** Registers a bunch of PDO entries for a domain.
         *
         * This method has to be called in non-realtime context before
         * ecrt_master_activate().
         *
         * \see ecrt_slave_config_reg_pdo_entry()
         *
         * \attention The registration array has to be terminated with an empty
         *            structure, or one with the \a index field set to zero!
         * \return 0 on success, else non-zero.
         */

        if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs))
        {
            fprintf(stderr, "PDO entry registration failed!\n");
            return;
        }

        // ecrt_slave_config_dc for assignActivate/sync0,1 cycle and shift values for each drive/slave....
    }

    configureSharedMemory();
}

EthercatMaster::~EthercatMaster()
{

    // Release EtherCAT master resources
    if (master)
    {
        ecrt_master_deactivate(master);
        ecrt_release_master(master);
    }

    // Release shared memory
    if (jointDataPtr != nullptr)
    {
        munmap(jointDataPtr, sizeof(JointData));
    }
    if (systemStateDataPtr != nullptr)
    {
        munmap(systemStateDataPtr, sizeof(SystemStateData));
    }
}

void EthercatMaster::run()
{

    // Activate the master
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        perror("Error activating master");
        // Handle the error appropriately based on your application's requirements
    }

    /** Returns the domain's process data.
     *
     * - In kernel context: If external memory was provided with
     * ecrt_domain_external_memory(), the returned pointer will contain the
     * address of that memory. Otherwise it will point to the internally allocated
     * memory. In the latter case, this method may not be called before
     * ecrt_master_activate().
     *
     * - In userspace context: This method has to be called after
     * ecrt_master_activate() to get the mapped domain process data memory.
     *
     * \return Pointer to the process data memory.
     */

    if (!(domainPd = ecrt_domain_data(domain)))
    {
        return;
    }

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset); // Set to the desired CPU core

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
    signal(SIGINT, EthercatMaster::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    // Set real-time interval for the master
    ecrt_master_set_send_interval(master, 1000);

    cyclicTask();
}

void EthercatMaster::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void EthercatMaster::pdoMapping(ec_slave_config_t *sc)
{
    /* Define RxPdo */
    ecrt_slave_config_sync_manager(sc, 2, EC_DIR_OUTPUT, EC_WD_ENABLE);

    ecrt_slave_config_pdo_assign_clear(sc, 2);

    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1600);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1601);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1602);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1602);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16); /* 0x6040:0/16bits, control word */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8);  /* 0x6060:0/8bits, mode_of_operation */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6071, 0, 16); /* 0x6071:0/16bits, target torque */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607A, 0, 32); /* 0x607a:0/32bits, target position */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x60FF, 0, 32); /* 0x60FF:0/32bits, target velocity */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x60B2, 0, 16); /* 0x60B2:0/16bits, torque offset */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x60B1, 0, 32); /* 0x60B1:0/32bits, velocity offset */

    /* Define TxPdo */

    ecrt_slave_config_sync_manager(sc, 3, EC_DIR_INPUT, EC_WD_ENABLE);

    ecrt_slave_config_pdo_assign_clear(sc, 3);

    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A00);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A01);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A02);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A01);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A02);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16); /* 0x6041:0/16bits, Statusword */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6061, 0, 8);  /* 0x6061:0/8bits, Modes of operation display */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32); /* 0x6064:0/32bits, Position Actual Value */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x606C, 0, 32); /* 0x606C:0/32bits, velocity_actual_value */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6077, 0, 16); /* 0x6077:0/16bits, Torque Actual Value */

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x2600, 0, 32); /* 0x60FD:0/32bits, Digital Inputs */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x603F, 0, 16); /* 0x603F:0/16bits, Error Code */
}

void EthercatMaster::signalHandler(int signum)
{

    if (signum == SIGINT)
    {
        std::cout << "Signal received: " << signum << std::endl;
        exitFlag = 1; // Set the flag to indicate the signal was received
    }
}

void EthercatMaster::configureSharedMemory()
{
    int shm_fd_jointData;
    int shm_fd_systemStateData;

    createSharedMemory(shm_fd_jointData, "JointData", sizeof(JointData));
    createSharedMemory(shm_fd_systemStateData, "SystemStateData", sizeof(SystemStateData));

    mapSharedMemory((void *&)jointDataPtr, shm_fd_jointData, sizeof(JointData));
    mapSharedMemory((void *&)systemStateDataPtr, shm_fd_systemStateData, sizeof(SystemStateData));

    initializeSharedData();
}

void EthercatMaster::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void EthercatMaster::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void EthercatMaster::initializeSharedData()
{
    jointDataPtr->setZero();
    systemStateDataPtr->setZero();

    printf("Waiting for Safety Controller to get Started ...\n");
    while (!systemStateDataPtr->safety_controller_enabled)
    {
    }

    printf("Safety Controller Started \n");
}

void EthercatMaster::checkDomainState()
{
    // cout << "check_domain_state" << endl;
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds); // to do - do for all domains

    if (ds.working_counter != domainState.working_counter)
    {
        // printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainState.wc_state)
    {
        // printf("Domain1: State %u.\n", ds.wc_state);
    }

    domainState = ds;
}

void EthercatMaster::checkMasterState()
{
    // cout << "check_master_state" << endl;
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != masterState.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != masterState.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != masterState.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    masterState = ms;
}
