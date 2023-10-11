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
#define DOMAIN1_START 1
#define DOMAIN1_END 4

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

bool operation_enable_status[3] = {true, true, false};

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

#define ingenia_denalli_xcr 0x0000029c, 0x03831002 // Vendor Id, Product Code

bool drive_switched_on[3] = {false, false, false};
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
} drive_offset[3];

double gear_ratio = 50;
double rated_torque = 0.02;
double enc_count = 4096;

int conv_radians_to_count(double rad)
{
    return (int) (enc_count * gear_ratio *rad / (2 * M_PI));
}

double conv_count_to_rad(int count)
{
    return (count / (enc_count * gear_ratio) * (2 * M_PI));
}

// Needs to be checked from Drive Side for Unit and Formula also not correct
int conv_rad_sec_to_mrev_sec(double rad_sec)
{
    return (rad_sec  / (2 * M_PI));
}

double conv_mrev_sec_to_rad_sec(int mrev_sec)
{
    return (2 * M_PI * mrev_sec / 1000);
}

double conv_to_actual_torque(int torq_val)
{
    return (torq_val / 1000 * rated_torque * gear_ratio);
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

