#pragma once

#include "SharedObject.h"

class InstrumentMotionPlanner
{
public:
    InstrumentMotionPlanner();
    ~InstrumentMotionPlanner();

private:
    SystemData *systemDataPtr;
    AppData *appDataPtr;
    CommandData *commandDataPtr;
    ForceDimData *forceDataPtr;

    void stackPrefault();
    void cyclicTask();
    static void signalHandler(int signum);

    int changeSystemState();
    int write_to_drive(double joint_pos[NUM_JOINTS], double joint_vel[NUM_JOINTS]);
    double sterile_engagement();
    double jog(int index, int dir, int type);
    void Jog();
    int write_to_drive(double joint_pos[4]);
    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);
    void initializeSharedData();

    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    void do_rt_task();
    static void wait_rest_of_period(struct period_info *pinfo);

};
