#include "master.h"
using namespace std;
int *ptrRxPDO, *ptrTxPDO;

/*****************************************************************************/

void check_domain_state(void)
{
    // cout << "check_domain_state" << endl;
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds); // to do - do for all domains

    if (ds.working_counter != domain1_state.working_counter)
    {
        // printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        // printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    // cout << "check_master_state" << endl;
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;

#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0,
             exec_ns = 0,
             latency_ns = 0,
             latency_min_ns = 0,
             latency_max_ns = 0,
             period_min_ns = 0,
             period_max_ns = 0,
             exec_min_ns = 0,
             exec_max_ns = 0;
#endif

    // cout << "cyclic_task" << endl;

    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while (1)
    {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns)
        {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns)
        {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns)
        {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns)
        {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns)
        {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns)
        {
            exec_min_ns = exec_ns;
        }
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // check process data state (optional)
        check_domain_state();

        if (counter)
        {
            counter--;
        }
        else
        { // do this at 1 Hz
            counter = FREQUENCY;
            // check for master state (optional)
            check_master_state();

#ifdef MEASURE_TIMING
            // output timing stats
            printf("period     %10u ... %10u\n",
                   period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                   exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                   latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif
        }

        // printf("domain1 %lu \n", ecrt_domain_size(domain1));
        // printf("%lu \n", ecrt_domain_size(domain2));

        // ************ For Domain 1 **************

        int domain1_slave_cnt = 6;
        static uint16_t domain1_command[6] = {0};

        std::cout<<"system_state_data_ptr->current_state: "<<system_state_data_ptr->current_state<<std::endl;

        switch (system_state_data_ptr->current_state)
        {
        case DriveState::DISABLED:
        {
            /* code */
            break;    
        }
        case DriveState::SWITCHED_ON: // Automatic switch to safety controller when finished
        {
            for (size_t jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
            {
                uint16_t drive_status = EC_READ_U16(domain1_pd + drive_offset[jnt_ctr].statusword);
                
                if ( (((drive_status | 65456) ^ 65471) == 0) || (((drive_status | 65456) ^ 65464) == 0))
                {
                    std::cout<<"Drive "<<jnt_ctr<<" in fault state \n";
                    transition_to_fault_state(drive_status, domain1_command[jnt_ctr], jnt_ctr);
                }

                domain1_command[jnt_ctr] = transition_to_switched_on(drive_status, domain1_command[jnt_ctr], jnt_ctr);

                EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].controlword, domain1_command[jnt_ctr]);

                EC_WRITE_U8(domain1_pd + drive_offset[1].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[1].led_colour, 65280);
                EC_WRITE_U8(domain1_pd + drive_offset[2].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[2].led_colour, 65280);
                EC_WRITE_U8(domain1_pd + drive_offset[4].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[4].led_colour, 65280);
                /* code */
            }

            std::cout<<"drive_switched_on[0] "<<drive_switched_on[0]<<std::endl;
            std::cout<<"drive_switched_on[1] "<<drive_switched_on[1]<<std::endl;
            std::cout<<"drive_switched_on[2] "<<drive_switched_on[2]<<std::endl;
            std::cout<<"drive_switched_on[3] "<<drive_switched_on[3]<<std::endl;
            std::cout<<"drive_switched_on[4] "<<drive_switched_on[4]<<std::endl;
            std::cout<<"drive_switched_on[5] "<<drive_switched_on[5]<<std::endl;
            

            system_state_data_ptr->current_state = drive_switched_on[0] &&
                                                            drive_switched_on[1] &&
                                                            drive_switched_on[2] &&
                                                            drive_switched_on[3] &&
                                                            drive_switched_on[4] &&
                                                            drive_switched_on[5] ? DriveState::SAFETY_CONTROLLER_ENABLED : DriveState::SWITCHED_ON;

            system_state_data_ptr->safety_check_done = false;
            system_state_data_ptr->trigger_error_mode = false;
            break;
        }
        case DriveState::SAFETY_CONTROLLER_ENABLED: // Automatic switch to Ready for operation/Error (safety check)
        {
            for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
            {
                joint_data_ptr->joint_position[jnt_ctr] = conv_count_to_rad(EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].position_actual_value), jnt_ctr);
                joint_data_ptr->joint_velocity[jnt_ctr] = conv_rpm_to_rad_sec(EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].velocity_actual_value), jnt_ctr);
                joint_data_ptr->joint_torque[jnt_ctr] = 0;
            }
            system_state_data_ptr->start_safety_check = true;

            if(system_state_data_ptr->safety_check_done)
            {
                if(system_state_data_ptr->trigger_error_mode)
                    system_state_data_ptr->current_state = DriveState::ERROR;
                else
                {
                    system_state_data_ptr->current_state = DriveState::READY_FOR_OPERATION;
                    system_state_data_ptr->start_safety_check = false;
                    system_state_data_ptr->safety_check_done = false;
                }
            }

            break;
        }
        case DriveState::READY_FOR_OPERATION:  // Wait for acknowledgement from user to swich to operation mode 
        {
            system_state_data_ptr->ready_for_operation = true;
            if(system_state_data_ptr->safety_check_done && !system_state_data_ptr->trigger_error_mode)
            {
                system_state_data_ptr->current_state = DriveState::SWITCH_TO_OPERATION;
            }
            break;
        }
        case DriveState::SWITCH_TO_OPERATION: // Automatic switch to operation enabled after enabling all motors
        {
            for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
            {
                uint16_t drive_status = EC_READ_U16(domain1_pd + drive_offset[jnt_ctr].statusword);
                EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].controlword, 15);
                EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].modes_of_operation, 8);
                EC_WRITE_S32(domain1_pd + drive_offset[jnt_ctr].target_position, joint_data_ptr->joint_position[jnt_ctr]);

                std::cout<<"jnt_ctr : "<<jnt_ctr<<", (((drive_status | 65424) ^ 65459) == 0) : "<<(((drive_status | 65424) ^ 65459) == 0)<<", ((drive_status | 65424) ^ 65463) == 0 : "<<(((drive_status | 65424) ^ 65463) == 0)<<std::endl;

                if (((drive_status | 65424) ^ 65463) == 0){
                    system_state_data_ptr->drive_enable_for_operation[jnt_ctr] = true;
                }
            }
            
            system_state_data_ptr->current_state = system_state_data_ptr->drive_enable_for_operation[0] &&
                                                        system_state_data_ptr->drive_enable_for_operation[1] &&
                                                        system_state_data_ptr->drive_enable_for_operation[2] &&
                                                        system_state_data_ptr->drive_enable_for_operation[3] &&
                                                        system_state_data_ptr->drive_enable_for_operation[4] &&
                                                        system_state_data_ptr->drive_enable_for_operation[5] ? DriveState::OPERATION_ENALBLED : DriveState::SWITCH_TO_OPERATION;
            
            std::cout<<"drive_enable_for_operation[0] "<<system_state_data_ptr->drive_enable_for_operation[0]<<std::endl;
            std::cout<<"drive_enable_for_operation[1] "<<system_state_data_ptr->drive_enable_for_operation[1]<<std::endl;
            std::cout<<"drive_enable_for_operation[2] "<<system_state_data_ptr->drive_enable_for_operation[2]<<std::endl;
            std::cout<<"drive_enable_for_operation[3] "<<system_state_data_ptr->drive_enable_for_operation[3]<<std::endl;
            std::cout<<"drive_enable_for_operation[4] "<<system_state_data_ptr->drive_enable_for_operation[4]<<std::endl;
            std::cout<<"drive_enable_for_operation[5] "<<system_state_data_ptr->drive_enable_for_operation[5]<<std::endl;

            break;
        }
        case DriveState::OPERATION_ENALBLED:
        {
            system_state_data_ptr->status_operation_enabled = true;
            // read write data to pdos
            break;
        }
        case DriveState::ERROR:
        {
            system_state_data_ptr->current_state = DriveState::DISABLED; // or swichted on state
            break;
        }
        default:
            break;
        }

        /*
        for (int jnt_ctr = 0; jnt_ctr < domain1_slave_cnt; jnt_ctr++)
        {
            
            uint16_t drive_status = EC_READ_U16(domain1_pd + drive_offset[jnt_ctr].statusword);

            if (!system_state_data_ptr->status_switched_on){

                domain1_command[jnt_ctr] = transition_to_switched_on(drive_status, domain1_command[jnt_ctr], jnt_ctr);

                EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].controlword, domain1_command[jnt_ctr]);

                EC_WRITE_U8(domain1_pd + drive_offset[1].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[1].led_colour, 65280);
                EC_WRITE_U8(domain1_pd + drive_offset[2].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[2].led_colour, 65280);
                EC_WRITE_U8(domain1_pd + drive_offset[4].gpio_global_option, 2);
                EC_WRITE_U32(domain1_pd + drive_offset[4].led_colour, 65280);

                system_state_data_ptr->status_switched_on = drive_switched_on[0] &&
                                                            drive_switched_on[1] &&
                                                            drive_switched_on[2] &&
                                                            drive_switched_on[3] &&
                                                            drive_switched_on[4] &&
                                                            drive_switched_on[5];

            }
            else{

                for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
                {
                    joint_data_ptr->joint_position[jnt_ctr] = conv_count_to_rad(EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].position_actual_value), jnt_ctr);
                    joint_data_ptr->joint_velocity[jnt_ctr] = conv_rpm_to_rad_sec(EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].velocity_actual_value), jnt_ctr);
                    joint_data_ptr->joint_torque[jnt_ctr] = 0;

                    // ptrTxPDO[jnt_ctr + 1] = drive_status;
                    // ptrTxPDO[6 + jnt_ctr + 1] = EC_READ_S8(domain1_pd + drive_offset[jnt_ctr].mode_of_operation_display);
                    // ptrTxPDO[12 + jnt_ctr + 1] = EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].position_actual_value);
                    // ptrTxPDO[18 + jnt_ctr + 1] = EC_READ_S32(domain1_pd + drive_offset[jnt_ctr].velocity_actual_value);
                    // ptrTxPDO[24 + jnt_ctr + 1] = EC_READ_S16(domain1_pd + drive_offset[jnt_ctr].torque_actual_value);
                    // ptrTxPDO[30 + jnt_ctr + 1] = EC_READ_U32(domain1_pd + drive_offset[jnt_ctr].digital_input_value);
                }

                if (!system_state_data_ptr->trigger_error_mode == false){

                }

                if (!system_state_data_ptr->trigger_operation_enabled){

                }

                else if(!system_state_data_ptr->ready_for_operation){

                    for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
                    {
                        EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].controlword, 15);
                        EC_WRITE_U16(domain1_pd + drive_offset[jnt_ctr].modes_of_operation, 8);
                        EC_WRITE_S32(domain1_pd + drive_offset[jnt_ctr].target_position, joint_data_ptr->joint_position[jnt_ctr]);

                        if (((drive_status | 65424) ^ 65463) == 0){
                            system_state_data_ptr->drive_enable_for_operation[jnt_ctr] = true;
                        }
                    }

                    system_state_data_ptr->ready_for_operation = system_state_data_ptr->drive_enable_for_operation[0] &&
                                                                system_state_data_ptr->drive_enable_for_operation[1] &&
                                                                system_state_data_ptr->drive_enable_for_operation[2] &&
                                                                system_state_data_ptr->drive_enable_for_operation[3] &&
                                                                system_state_data_ptr->drive_enable_for_operation[4] &&
                                                                system_state_data_ptr->drive_enable_for_operation[5];

                }
                else{

                    // EC_WRITE_S32(domain1_pd + drive_offset[jnt_ctr].target_position, ptrRxPDO[18 + jnt_ctr]);
                    // EC_WRITE_S32(domain1_pd + drive_offset[jnt_ctr].velocity_offset, ptrRxPDO[36 + jnt_ctr]);
                    // EC_WRITE_U8(domain1_pd + drive_offset[1].gpio_global_option, 2);
                    // EC_WRITE_U32(domain1_pd + drive_offset[1].led_colour, 65280);
                    // EC_WRITE_U8(domain1_pd + drive_offset[2].gpio_global_option, 2);
                    // EC_WRITE_U32(domain1_pd + drive_offset[2].led_colour, 65280);
                    // EC_WRITE_U8(domain1_pd + drive_offset[4].gpio_global_option, 2);
                    // EC_WRITE_U32(domain1_pd + drive_offset[4].led_colour, 65280);

                }
            }

        }
        */

        if (sync_ref_counter)
        {
            sync_ref_counter--;
        }
        else
        {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }

        ecrt_master_sync_slave_clocks(master);

        ecrt_domain_queue(domain1);

        // todo - sync distributed clock
        /*
        ecrt_master_reference_clock_time
        ecrt_master_sync_slave_clocks
        ecrt_master_application_time
        */

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif

        ecrt_master_send(master);
    }
}

/****************************************************************************/

void stack_prefault(void)
{
    // cout << "stack_prefault" << endl;
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void sdo_mapping(ec_slave_config_t *sc, int jnt_ctr)
{
    // required sdo's needed to be added to slave config here...

    //     /** Create an SDO request to exchange SDOs during realtime operation.
    //  *
    //  * The created SDO request object is freed automatically when the master is
    //  * released.
    //  *
    //  * This method has to be called in non-realtime context before
    //  * ecrt_master_activate().
    //  *
    //  * \return New SDO request, or NULL on error.
    //  */
    // ec_sdo_request_t *ecrt_slave_config_create_sdo_request(
    //         ec_slave_config_t *sc, /**< Slave configuration. */
    //         uint16_t index, /**< SDO index. */
    //         uint8_t subindex, /**< SDO subindex. */
    //         size_t size /**< Data size to reserve. */
    //         );

    // gpio_global_option[jnt_ctr] = ecrt_slave_config_create_sdo_request(sc, 0x2214, 1, 1);
    // led_colour[jnt_ctr] = ecrt_slave_config_create_sdo_request(sc, 0x2215, 1, 4);
}

/****************************************************************************/
void pdo_mapping(ec_slave_config_t *sc)
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

    ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x60FE, 1, 32); /* 0x60FE:1/32bits, Digital Physical Outputs */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x60FE, 2, 32); /* 0x60FE:2/32bits, Digital Output bit Mask */

    ecrt_slave_config_pdo_mapping_add(sc, 0x1602, 0x2214, 1, 8);  /* 0x2214:1/8bits, gpio_global_option */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1602, 0x2215, 1, 32); /* 0x2215:1/32bits, led_colour*/

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

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x60FD, 0, 32); /* 0x60FD:0/32bits, Digital Inputs */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x603F, 0, 16); /* 0x603F:0/16bits, Error Code */
}

/****************************************************************************/

int main(int argc, char **argv)
{

    struct timespec wakeup_time;
    int ret = 0;

    // Retrieving Master
    master = ecrt_request_master(0);

    if (!master)
    {
        return -1;
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

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        return -1;
    }

    int local_pos = 0;

    for (uint16_t jnt_ctr = DOMAIN1_START; jnt_ctr < DOMAIN1_END; jnt_ctr++)
    {
        ec_slave_config_t *sc;

        if (!(sc = ecrt_master_slave_config(master, 0, jnt_ctr, synapticon_circulo)))
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }

        pdo_mapping(sc);

        ec_pdo_entry_reg_t domain_regs[] = {

            {0, jnt_ctr, synapticon_circulo, 0x6041, 0, &drive_offset[jnt_ctr - local_pos].statusword},                // 6041 0 statusword
            {0, jnt_ctr, synapticon_circulo, 0x6061, 0, &drive_offset[jnt_ctr - local_pos].mode_of_operation_display}, // 6061 0 mode_of_operation_display
            {0, jnt_ctr, synapticon_circulo, 0x6064, 0, &drive_offset[jnt_ctr - local_pos].position_actual_value},     // 6064 0 pos_act_val
            {0, jnt_ctr, synapticon_circulo, 0x606C, 0, &drive_offset[jnt_ctr - local_pos].velocity_actual_value},     // 606C 0 vel_act_val
            {0, jnt_ctr, synapticon_circulo, 0x6077, 0, &drive_offset[jnt_ctr - local_pos].torque_actual_value},       // 6077 0 torq_act_val
            {0, jnt_ctr, synapticon_circulo, 0x60FD, 0, &drive_offset[jnt_ctr - local_pos].digital_input_value},       // 60FD 0 digital_input_value
            {0, jnt_ctr, synapticon_circulo, 0x603F, 0, &drive_offset[jnt_ctr - local_pos].error_code},                // 603F 0 digital_input_value
            {0, jnt_ctr, synapticon_circulo, 0x6040, 0, &drive_offset[jnt_ctr - local_pos].controlword},               // 6040 0 control word
            {0, jnt_ctr, synapticon_circulo, 0x6060, 0, &drive_offset[jnt_ctr - local_pos].modes_of_operation},        // 6060 0 mode_of_operation
            {0, jnt_ctr, synapticon_circulo, 0x6071, 0, &drive_offset[jnt_ctr - local_pos].target_torque},             // 6071 0 target torque
            {0, jnt_ctr, synapticon_circulo, 0x607A, 0, &drive_offset[jnt_ctr - local_pos].target_position},           // 607A 0 target position
            {0, jnt_ctr, synapticon_circulo, 0x60FF, 0, &drive_offset[jnt_ctr - local_pos].target_velocity},           // 60FF 0 target velocity
            {0, jnt_ctr, synapticon_circulo, 0x60B2, 0, &drive_offset[jnt_ctr - local_pos].torque_offset},
            {0, jnt_ctr, synapticon_circulo, 0x60B1, 0, &drive_offset[jnt_ctr - local_pos].velocity_offset},         // 60B2 0 torque offset
            {0, jnt_ctr, synapticon_circulo, 0x60FE, 1, &drive_offset[jnt_ctr - local_pos].digital_physical_output}, // 60FE 1 digital_output_value
            {0, jnt_ctr, synapticon_circulo, 0x60FE, 2, &drive_offset[jnt_ctr - local_pos].digital_output_bit_mask}, // 60FE 2 digital_output_bit_mask
            {0, jnt_ctr, synapticon_circulo, 0x2214, 1, &drive_offset[jnt_ctr - local_pos].gpio_global_option},      // 60FE 1 digital_output_value
            {0, jnt_ctr, synapticon_circulo, 0x2215, 1, &drive_offset[jnt_ctr - local_pos].led_colour},              // 60FE 2 digital_output_bit_mask
            {}

        };

        ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, 0, 0, 0);

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

        if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs))
        {
            fprintf(stderr, "PDO entry registration failed!\n");
            return -1;
        }

        // ecrt_slave_config_dc for assignActivate/sync0,1 cycle and shift values for each drive/slave....
    }

    // ecrt_master_application_time to set application time for master and distributed clock.

    // ecrt_master_select_reference_clock usually set first slave as a reference clock for distributed clocks...

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        return -1;
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

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        return -1;
    }

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

    while (!system_state_data_ptr->safety_controller_enabled){
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    ecrt_master_set_send_interval(master, 1000);

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1)
    {
        // printf("line 697 \n");
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                              &wakeup_time, NULL);
        if (ret)
        {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC)
        {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}
