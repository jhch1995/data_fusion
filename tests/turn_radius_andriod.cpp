#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h> // time

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

//#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace imu;
int main(int argc, char *argv[])
{   
    TimeUtils f_time_counter;
    int64_t t_1, t_2;
    int r_state = -1;
    double R_cur;
    ImuModule::Instance().StartDataFusionTask();
    
    while(1){
        t_1 = f_time_counter.Microseconds();
        usleep(50000); // 50ms
        r_state = ImuModule::Instance().GetTurnRadius( t_1, &R_cur);
        printf("state: r_state,  R = %f\n", R_cur);
    }
    return 0;
}

