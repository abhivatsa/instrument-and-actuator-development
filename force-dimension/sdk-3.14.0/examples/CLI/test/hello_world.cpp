////////////////////////////////////////////////////////////////////////////////
//
// This example implements a simple gravity compensation loop for a single
// haptic device.
//
////////////////////////////////////////////////////////////////////////////////
// C++ library headers
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
// project headers
#include "dhdc.h"
#include "drdc.h"
////////////////////////////////////////////////////////////////////////////////

struct ForceDimData{

    void setZero(){
        
        gripper_pos = 0;
        gripper_vel = 0;

        for (int ctr = 0; ctr < 3; ctr++){
            cart_pos[ctr] = 0;
            cart_linear_vel[ctr] = 0;
            cart_angular_vel[ctr] = 0;
        }

        for (int ctr = 0; ctr < 9; ctr++){
            cart_orient[ctr] = 0;
        }

    }

    double cart_pos[3];
    double cart_linear_vel[3];
    double cart_orient[9];
    double cart_angular_vel[3];
    double gripper_pos;
    double gripper_vel;
};

/* pointer to shared memory object */
ForceDimData *force_dim_ptr;

int main(int argc, char *argv[])
{

    const int SIZE_ForceDimData = sizeof(ForceDimData);
    double shm_fd_ForceDimData;
    shm_fd_ForceDimData = shm_open("ForceDimData", O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_ForceDimData, SIZE_ForceDimData);
    force_dim_ptr = static_cast<ForceDimData *>(mmap(0, SIZE_ForceDimData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ForceDimData, 0));

    // get device count
    if (dhdGetDeviceCount() <= 0)
    {
        std::cout << "error: " << dhdErrorGetLastStr() << std::endl;
        return -1;
    }
    // open the first available device
    if (dhdOpen() < 0)
    {
        std::cout << "error: " << dhdErrorGetLastStr() << std::endl;
        return -1;
    }
    // haptic loop

    // open the first available device
    if (drdOpen() < 0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return -1;
    }

    double px, py, pz;

    double wx, wy, wz;

    double gripper_vel;

    double orient_mat[3][3];

    // center of workspace
    double nullPose[DHD_MAX_DOF] = {0.0, 0.0, 0.0, // base  (translations)
                                    0.0, 0.0, 0.0, // wrist (rotations)
                                    0.0};          // gripper

    // perform auto-initialization
    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return -1;
    }
    else if (drdStart() < 0)
    {
        printf("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return -1;
    }

    // move to center
    drdMoveTo(nullPose);

    // stop regulation thread (but leaves forces on)
    drdStop (true);

    // dhdEnableForce(DHD_ON);

    while (true)
    {

        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }

        if (dhdGetOrientationFrame(orient_mat) < 0)
        {
            std::cout << "Some_error \n";
        }
        else
        {
            force_dim_ptr->cart_orient[0] = orient_mat[0][0];
            force_dim_ptr->cart_orient[1] = orient_mat[0][1];
            force_dim_ptr->cart_orient[2] = orient_mat[0][2];
            force_dim_ptr->cart_orient[3] = orient_mat[1][0];
            force_dim_ptr->cart_orient[4] = orient_mat[1][1];
            force_dim_ptr->cart_orient[5] = orient_mat[1][2];
            force_dim_ptr->cart_orient[6] = orient_mat[2][0];
            force_dim_ptr->cart_orient[7] = orient_mat[2][1];
            force_dim_ptr->cart_orient[8] = orient_mat[2][2];
        }

        if (dhdGetPosition(&px, &py, &pz) < 0)
        {
            std::cout << "Some_error in getting XYZ values \n";
        }
        else
        {
            force_dim_ptr->cart_pos[0] = px;
            force_dim_ptr->cart_pos[1] = py;
            force_dim_ptr->cart_pos[2] = pz;
        }

        if (dhdGetAngularVelocityRad(&wx, &wy, &wz) < 0)
        {
            std::cout << "Some_error in getting XYZ values \n";
        }
        else
        {
            force_dim_ptr->cart_angular_vel[0] = wx;
            force_dim_ptr->cart_angular_vel[1] = wy;
            force_dim_ptr->cart_angular_vel[2] = wz;
        }

        if (dhdGetGripperAngularVelocityRad(&gripper_vel) < 0)
        {
            std::cout << "Unable to get values \n";
        }
        else{
            force_dim_ptr->gripper_vel = gripper_vel;
        }

        // sleep(1);

        // exit the haptic loop on button press
        // if (dhdGetButton(0))
        // {
        //     break;
        // }

        if (dhdKbHit ()){
            if (dhdKbGet() == 'q'){
                break;
            }
        }
    }
    // close the connection to the device
    if (dhdClose() < 0)
    {
        std::cout << "error: " << dhdErrorGetLastStr() << std::endl;
    }

    drdClose ();
    return 0;
}
