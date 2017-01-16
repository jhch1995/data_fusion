#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>
#include <signal.h> 

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

#define MACH_INIT 1
#define ROD_ACC_THRESHOLD 2

#define L_LAMP 4
#define R_LAMP 5
#define M_LAMP 6
 
#define CAMERA_ACC_QUEUE_SIZE 40
#pragma pack(1)
    typedef struct {
    	unsigned short rd_index;
    	unsigned short wr_index;
    	AccData	acc_data[CAMERA_ACC_QUEUE_SIZE];
    }AccDataQueue;
#pragma pack()


bool g_is_start_init_match = false;
bool g_set_rod_init_match_start = false;
bool g_start_d_rod_acc_calculate = false; // 开始rod自检测的阈值计算


void set_left_turnlamp(int sig)  
{  
    printf("\n state = 1 %d\n", sig);    
}  

void set_right_turnlamp(int sig)  
{  
    printf("\n state = -1 %d\n", sig);    
}  

void set_middle_turnlamp(int sig)  
{  
    printf("\n state = 0 %d\n", sig);    
}  

void set_rod_camera_trans_match(int sig)  
{  
    g_is_start_init_match = true;
    g_set_rod_init_match_start = true;
    printf("\n state = 0 %d, start rod init match!!\n", sig);    
} 


void start_d_rod_acc_calculate(int sig)  
{  
    g_start_d_rod_acc_calculate = true;
    printf("\n state = 0 %d, start rod acc calculate!!\n", sig);    
} 


#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 1, " ");
#endif

int main(int argc, char *argv[])
{
    // init
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";
    
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
        #endif
    #endif

    HalIO &halio = HalIO::Instance();
    bool res = halio.Init(NULL, 1);
    if (!res) {
        std::cerr << "HALIO init fail" << std::endl;
        return -1;
    }
    halio.EnableFMU(); // 使能CAN对应的filter

    double turnlamp_detect_delay_num = 20; // 当第一次检测到拨杆可能被拨动，然后就持续检测后面的Ｎ个数据，就行比较判断当前拨杆的绝对位置
    double turnlamp_detect_delay_counter = turnlamp_detect_delay_num;
    double diff_acc_max = 1.5; // 最大的acc差值是2,大于这个数据不能用于进行拨杆绝对位置判断
    double average_num_counter = 0.0; // 有效的用于计算均值的acc_diff

    int camera_init_match_acc_data_counter = 0;

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();

    TurnlampDetector &m_turnlamp_detector = TurnlampDetector::Instance();
    #if !defined(DATA_FROM_LOG)
        m_turnlamp_detector.StartTurnlampDetectTaskOnline();
        printf("!!! Start Turnlamp Detect Task Online\n");
    #endif

    // match
    int mean_init_camera_acc_counter = 0; // 进行初始化校正的时候camera的数据计数 
    double mean_init_camera_acc[3] = {0, 0, 0};
    bool is_camera_init_acc_data_ok = false; // camera acc的初始化数据收集是否已经ok
//    double acc_fmu0[3] = {-2.42, -1.92, -9.25};
//    double acc_camera0[3] = {-1, -0.36, -10.05};
//    m_turnlamp_detector.CalculateRotationFmu2Camera(acc_fmu0, acc_camera0);

    double diff_acc_average[3] = {0.0, 0.0, 0.0}; // 检测到可能的波动后进行绝对位置判断
    bool if_new_acc_diff_mean = true; // 是否是新的diff_acc_average数据
    double diff_acc[3], diff_gyro[3];

    // 设置PC端控制信号
    (void) signal(MACH_INIT, set_rod_camera_trans_match); 
    (void) signal(ROD_ACC_THRESHOLD, start_d_rod_acc_calculate); 
    
    (void) signal(L_LAMP, set_left_turnlamp);  
//    (void) signal(R_LAMP, set_right_turnlamp); 
//    (void) signal(M_LAMP, set_middle_turnlamp); 
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];
    while(1){
        // 读取camera imu数据
        #if defined(DATA_FROM_LOG)
            m_turnlamp_detector.RunDetectTurnlampOffline();
            // 在读取log日志的时候 推进data_fusion的时间戳
            double R_cur;
            double timestamp_search = m_turnlamp_detector.m_fmu_imu_data.timestamp;
            int64_t image_timestamp_cur_int = (int64_t)(timestamp_search*1000);
            data_fusion.GetTurnRadius( image_timestamp_cur_int, &R_cur);
        #else
            struct timeval time_imu;
            gettimeofday(&time_imu, NULL);
            double timestamp_search = time_imu.tv_sec + time_imu.tv_usec*1e-6;
        #endif

        // 初始化对准
        // 开启rod init match数据采集
        if(g_set_rod_init_match_start){
            m_turnlamp_detector.SartRotationInitDataCollect();
            g_set_rod_init_match_start = false;
        }

        // g_start_d_rod_acc_calculate
        if(g_start_d_rod_acc_calculate){
            m_turnlamp_detector.SartCalculateRodAccThreshold();
            g_start_d_rod_acc_calculate = false;
        }
            
        if(g_is_start_init_match){
            if(!is_camera_init_acc_data_ok){ // 收集camera acc的数据
                usleep(20000);
                int search_state = data_fusion.GetTimestampData(timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
                if(search_state == 1){
                    if(camera_init_match_acc_data_counter++ < CAMERA_ACC_QUEUE_SIZE){
                        for(int k=0; k<3; k++)
                            mean_init_camera_acc[k] += acc_camera[k];
                        printf("index: %d amera acc data: %f %f %f\n", camera_init_match_acc_data_counter, acc_camera[0], acc_camera[1], acc_camera[2]);
                    }else{
                        for(int k=0; k<3; k++)
                            mean_init_camera_acc[k] = mean_init_camera_acc[k]/CAMERA_ACC_QUEUE_SIZE;
                        is_camera_init_acc_data_ok = true;
                        
                    }
                }else{
                    continue;
                }
            }else{
                // camera acc数据已经ok，判断rod的数据
                bool is_rod_init_acc_data_ok = m_turnlamp_detector.m_is_init_road_acc_collect_ok;
                if(is_rod_init_acc_data_ok){
                    printf("rod acc int data = %f %f %f\n", m_turnlamp_detector.m_mean_init_rod_acc[0], m_turnlamp_detector.m_mean_init_rod_acc[1], m_turnlamp_detector.m_mean_init_rod_acc[2]);
                    printf("camera acc int data = %f %f %f\n", mean_init_camera_acc[0], mean_init_camera_acc[1], mean_init_camera_acc[2]);
                    m_turnlamp_detector.CalculateRotationFmu2Camera(m_turnlamp_detector.m_mean_init_rod_acc, mean_init_camera_acc);
                    g_is_start_init_match = false;

                    // reset state for next run
                    is_camera_init_acc_data_ok = false;
                    camera_init_match_acc_data_counter = 0;
                    memset(mean_init_camera_acc, 0 ,sizeof(mean_init_camera_acc));
                    m_turnlamp_detector.ResetInitMatchState();
                }else{
                    usleep(100000);
                }
            }
        }


        if(m_turnlamp_detector.m_rod_shift_state){            
            // 拨杆可能被重新波动，重新计算均值
            turnlamp_detect_delay_counter = 0;
            average_num_counter = 0.0;
            memset(diff_acc_average, 0 , sizeof(diff_acc_average));
            if_new_acc_diff_mean = true;
        }else{
            // 分析检测到可能的波动之后 对拨杆的绝对位置进行判断
            if(turnlamp_detect_delay_counter < turnlamp_detect_delay_num){
                
                int search_state = data_fusion.GetTimestampData(timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
                for(int i=0; i<3; i++ ){
                    diff_acc[i] = m_turnlamp_detector.m_fmu_imu_data.acc[i] - acc_camera[i];
                    diff_gyro[i] = m_turnlamp_detector.m_fmu_imu_data.gyro[i] - gyro_camera[i];               
                } 
                
                if(fabs(diff_acc[0]) < diff_acc_max && fabs(diff_acc[1]) < diff_acc_max && fabs(diff_acc[2]) < diff_acc_max){
                     for(int i=0; i<3; i++ ){
                        diff_acc_average[i] += diff_acc[i]; // 计算均值
                     }
                     average_num_counter += 1;
                }
                std::cout<<"Main--"<<"Fmu&Camera_diff_acc = "<<diff_acc[0]<<", "<<diff_acc[1]<<", "<<diff_acc[2]<<", "<<endl;
                turnlamp_detect_delay_counter += 1;
            }else if(if_new_acc_diff_mean){
                if(average_num_counter>0){ // 打印均值
                    for(int i=0; i<3; i++ )
                        diff_acc_average[i] = diff_acc_average[i]/average_num_counter;
                }
                
                VLOG(VLOG_INFO)<<"Main--"<<"average num: "<<average_num_counter<<endl;
                VLOG(VLOG_INFO)<<"Main--"<<"mean Fmu&Camera_diff_acc = "<<diff_acc_average[0]<<", "<<diff_acc_average[1]<<", "<<diff_acc_average[2]<<endl;
                printf("rod acc = %f %f %f\n", m_turnlamp_detector.m_fmu_imu_data.acc[0], m_turnlamp_detector.m_fmu_imu_data.acc[1], m_turnlamp_detector.m_fmu_imu_data.acc[2]);
                printf("camera acc = %f %f %f\n", acc_camera[0], acc_camera[1], acc_camera[2]);
                printf("mean nums = %.0f, diff_acc = %f %f %f\n", average_num_counter, diff_acc_average[0], diff_acc_average[1], diff_acc_average[2]);
                if_new_acc_diff_mean = false; // 一次数据只打印一次
            }
            usleep(40000);
        }
    }
    printf("over!!\n");

    return 0;
}

