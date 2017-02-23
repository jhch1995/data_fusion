#include <dirent.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/hal/halio.h"
#include "common/hal/camctl.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include <Eigen/Dense>  
#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"
#include "turnlamp_detector.h"

using namespace imu;
using namespace std; 

void GetTimeS(double* timestamp);

void init();

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    // 初始化
    init();

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();

    usleep(500000);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    while(1){
        double timestamp;
        GetTimeS(&timestamp);
        int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.02, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            double acc_angle[2];
            acc_angle[0] = (atan2f(-acc_camera[1], -acc_camera[2]));       // Calculating pitch ACC angle
            acc_angle[1] = (atan2f(acc_camera[0], sqrtf(acc_camera[2]*acc_camera[2] + acc_camera[1]*acc_camera[1])));   //Calculating roll ACC angle  
            printf("attitude %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D);
            break;
        }else{
            printf("state: %d \n", search_state);
        }
        usleep(20000);
    }
    
    data_fusion.StopDataFusionTask();
    return 0;
}

void init()
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = 0; //VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

}


// 获取当前循环需要读取数据的时间
void GetTimeS(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}


