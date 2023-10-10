/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>     /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h>    /* sched_setscheduler() */
#include <stdbool.h>
#include <iostream>
#include "SharedObject.h"
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/


#define DOMAIN1_POSITION 0
#define DOMAIN1_START 0
#define DOMAIN1_END 6

/****************************************************************************/
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING
/** Task period in ns. */
#define PERIOD_NS (2000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

bool operation_enable_status[6] = {true, true, false, false, true, true};

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// process data
static uint8_t *domain1_pd = NULL;

static unsigned int counter = 1;

static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

#define synapticon_circulo 0x000022d2, 0x00000301 // Vendor Id, Product Code

bool drive_switched_on[6] = {false, false, false, false, false, false};
bool all_drive_enabled = false;

struct joint_pdos
{
    unsigned int statusword;
    unsigned int mode_of_operation_display;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_input_value;
    unsigned int error_code;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int torque_offset;
    unsigned int velocity_offset;
    unsigned int digital_physical_output;
    unsigned int digital_output_bit_mask;
    unsigned int gpio_global_option;
    unsigned int led_colour;
} drive_offset[6];

int conv_radians_to_count(double rad, int joint_num)
{
    if (joint_num < 3)
    {
        return (int)(524288 * rad / (2 * M_PI));
    }
    else
    {
        return (int)(262144 * rad / (2 * M_PI));
    }
}

double conv_count_to_rad(int count, int joint_num)
{
    if (joint_num < 3)
    {
        return (count / 524288 * (2 * M_PI));
    }
    else
    {
        return (count / 262144 * (2 * M_PI));
    }
}

int conv_rad_sec_to_rpm(double rad_sec, int joint_num)
{
    return (rad_sec * 60 / (2 * M_PI));
}

double conv_rpm_to_rad_sec(int rpm, int joint_num)
{
    return (2 * M_PI * rpm / 60);
}


/****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

uint16_t transition_to_switched_on(uint16_t status, uint16_t command, int joint_num)
{
    // cout << "update_state" << endl;
    if (((status | 65456) ^ 65456) == 0)
    {
        // std::cout<<"Not ready to switch on, joint num : "<<joint_num<<std::endl;
    }
    else if (((status | 65456) ^ 65520) == 0)
    {
        // std::cout<<"Switch on Disabled, joint num : "<<joint_num<<std::endl;
        command = 6;
    }
    else if (((status | 65424) ^ 65457) == 0)
    {
        // std::cout<<"Ready to Switch on, joint num : "<<joint_num<<std::endl;
        command = 7;
    }
    else if (((status | 65424) ^ 65459) == 0)
    {
        // std::cout<<"Switched On, joint num : "<<joint_num<<std::endl;
        // command = 15;
        drive_switched_on[joint_num] = true;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

/*****************************************************************************/

uint16_t transition_to_operation_enabled(uint16_t status, uint16_t command, int joint_num)
{
    if (((status | 65424) ^ 65459) == 0)
    {
        std::cout<<"Switched On, joint num : "<<joint_num<<std::endl;
        command = 15;
    }
    else if (((status | 65424) ^ 65463) == 0)
    {
        // printf(" Operation Enabled \n");
        // Operation Enabled
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

/*****************************************************************************/

uint16_t transition_to_fault_state(uint16_t status, uint16_t command, int joint_num)
{
    // cout << "update_state" << endl;
    if (((status | 65456) ^ 65471) == 0)
    {
        // Fault Reaction Active
        std::cout<<"Fault reaction active"<<std::endl;
    }
    else if (((status | 65456) ^ 65464) == 0)
    {
        // Fault
        std::cout<<"Fault"<<std::endl;
        command = 15;
    }
    else
    {
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

