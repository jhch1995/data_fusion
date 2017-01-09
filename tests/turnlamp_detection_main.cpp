#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "turnlamp_detector.h"

using namespace imu;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";
    #if defined(USE_GLOG)
        FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
//        FLAGS_v = 0;
    #endif

    double turnlamp_detect_delay_num = 10; // 当第一次检测到拨杆可能被拨动，然后就持续检测后面的Ｎ个数据，就行比较判断当前拨杆的绝对位置
    double turnlamp_detect_delay_counter = turnlamp_detect_delay_num;
    double diff_acc_max = 2.0*1.5; // 最大的acc差值是2,大于这个数据不能用于进行拨杆绝对位置判断
    double average_num_counter = 0.0; // 有效的用于计算均值的acc_diff

    // 进行数据融合的类
    DataFusion data_fusion;
    data_fusion.StartDataFusionTask();

    TurnlampDetector m_turnlamp_detector;
    double acc_fmu0[3] = {-2.42, -1.92, -9.25};
    double acc_camera0[3] = {-1, -0.36, -10.05};
    m_turnlamp_detector.CalculateRotationFmu2Camera(acc_fmu0, acc_camera0);

    double diff_acc_average[3] = {0.0, 0.0, 0.0}; // 检测到可能的波动后进行绝对位置判断
    bool if_new_acc_diff_mean = true; // 是否是新的diff_acc_average数据
    double diff_acc[3], diff_gyro[3];
    while(1){
        m_turnlamp_detector.RunDetectTurnlamp();
        double R_cur;
        double timestamp_search = m_turnlamp_detector.m_fmu_imu_data.timestamp;
        int64_t image_timestamp_cur_int = (int64_t)(timestamp_search*1000);
        data_fusion.GetTurnRadius( image_timestamp_cur_int, &R_cur);

        if(m_turnlamp_detector.m_rod_shift_state){            
            // 拨杆可能被重新波动，重新计算均值
            turnlamp_detect_delay_counter = 0;
            average_num_counter = 0.0;
            memset(diff_acc_average, 0 , sizeof(diff_acc_average));
            if_new_acc_diff_mean = true;
        }

        // 分析检测到可能的波动之后 对拨杆的绝对位置进行判断
        if(turnlamp_detect_delay_counter<turnlamp_detect_delay_num){
            double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];
            int search_state = data_fusion.GetTimestampData(timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
                    
            for(int i=0; i<3; i++ ){
                diff_acc[i] = m_turnlamp_detector.m_fmu_imu_data.acc[i] - acc_camera[i];
                diff_gyro[i] = m_turnlamp_detector.m_fmu_imu_data.gyro[i] - gyro_camera[i];               
            } 
            
            if(fabs(diff_acc[1]) < diff_acc_max){
                 for(int i=0; i<3; i++ ){
                    diff_acc_average[i] += diff_acc[i];// 计算均值
                 }
                 average_num_counter += 1;
            }
            VLOG(VLOG_INFO)<<"Main--"<<"Fmu&Camera_diff_acc = "<<diff_acc[0]<<", "<<diff_acc[1]<<", "<<diff_acc[2]<<", "<<endl;
            turnlamp_detect_delay_counter += 1;
        }else if(if_new_acc_diff_mean){
            // 打印均值
            if(average_num_counter>0){
                for(int i=0; i<3; i++ ){
                     diff_acc_average[i] = diff_acc_average[i]/average_num_counter;
                }
            }
            
            VLOG(VLOG_INFO)<<"Main--"<<"average num: "<<average_num_counter<<endl;
            VLOG(VLOG_INFO)<<"Main--"<<"mean Fmu&Camera_diff_acc = "<<diff_acc_average[0]<<", "<<diff_acc_average[1]<<", "<<diff_acc_average[2]<<endl;
            if_new_acc_diff_mean = false; // 一次数据只打印一次
        }

        usleep(5);
    }
    printf("over!!\n");

    return 0;
}

