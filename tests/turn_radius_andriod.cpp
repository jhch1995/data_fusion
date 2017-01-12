#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h> // time
#include <stdio.h>
#include "common/hal/halio.h"
#include "common/hal/timeop.h"
#include "common/concurrency/this_thread.h"


#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

//#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"


void speed_callback(struct timeval *tv, int type, float speed) {
    
}


using namespace imu;
int main(int argc, char *argv[])
{   
    ROD_DATA rod_data[20];
    HalIO &halio = HalIO::Instance();
    bool res = halio.Init(NULL, 1);
    if (!res) {
        std::cerr << "HALIO init fail" << std::endl;
        return -1;
    }    

    halio.EnableFMU();
    
    while(1){
        int read_rod_state = halio.read_rod_data(rod_data, 20);
        if(read_rod_state){
            for(int i=0; i<read_rod_state; i++){
                printf("rod data: %ld %ld %d %d %d num: %d\n", rod_data[i].tv.tv_sec, rod_data[i].tv.tv_usec, rod_data[i].acc[0], rod_data[i].acc[1], rod_data[i].acc[2], read_rod_state);
            }
        }else{
//            printf("no rod data, read state = %d\n", read_rod_state);
        }
     
        usleep(10000); // 10ms
    }
    return 0;
}

