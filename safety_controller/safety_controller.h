#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include "SharedObject.h"

#define PERIOD_NS (2000000)
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;

class SafetyController
{
public:
    SafetyController();
    ~SafetyController();
    void run();

private:
    JointData *jointDataPtr;
    SystemStateData *systemStateDataPtr;
    AppData *appDataPtr;
    void stackPrefault();
    void cyclicTask();
    static void signalHandler(int signum);

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);
    void initializeSharedData();

    bool check_limits();
    int pos_limit_check(double *joint_pos);
    void write_data();
    void read_data();

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