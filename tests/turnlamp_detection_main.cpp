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
#include <stdlib.h>


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

#define L_LAMP 3
#define R_LAMP 4
#define M_LAMP 5
 
#define CAMERA_ACC_QUEUE_SIZE 40
#pragma pack(1)
    typedef struct {
    	unsigned short rd_index;
    	unsigned short wr_index;
    	AccData	acc_data[CAMERA_ACC_QUEUE_SIZE];
    }AccDataQueue;
#pragma pack()

//----------------------------------------------------------------------------//
void init();

// 拨杆初始化坐标对准
int CalculateRodRotationInitMatch();

void DoCommondSwitch();

int CalculateDiffAcc();

// 获取当前循环需要读取数据的时间
void GetReadAccDataTime();

// 计算左右转向灯阈值
void CalculateTurnlampThreshold();

// 判断转向灯的位置
void DiagnoseLeftRightTurnlamp();


void CollectLeftTurnlampAccData(int sig);  

void CollectRightTurnlampAccData(int sig);  

void CalculateRodCameraRotation(int sig);  

void CalculateRodSelfShiftThreshold(int sig);


//----------------------------------------------------------------------------//
bool g_is_start_init_match = false;
bool g_set_rod_init_match_start = false;
bool g_start_d_rod_acc_calculate = false; // 开始rod自检测的阈值计算
int g_left_turnlamp_init_state = 0; // 开始左转向灯标定 0: 没有数据  1:  开始采集数据 2: 数据采集完毕，计算阈值
int g_right_turnlamp_init_state = 0; // 开始右转向灯标定  0: 没有数据  1:  开始采集数据 2: 数据采集完毕，计算阈值

bool g_is_camera_init_acc_data_collect_ok = false; // camera acc的初始化数据收集是否已经ok
double g_timestamp_search = 0;
int g_camera_init_match_acc_data_counter = 0;
double g_mean_init_camera_acc[3] = {0, 0, 0};

// init match
bool g_rotation_init_ok = false; // 初始化的坐标对准是否OK
int g_mean_init_camera_acc_counter = 0; // 进行初始化校正的时候camera的数据计数 
double g_diff_acc_average[3] = {0.0, 0.0, 0.0}; // 检测到可能的波动后进行绝对位置判断
bool g_calculate_new_acc_diff_mean = true; // 是否是新的diff_acc_average数据
double g_diff_acc[3];

// rod self shif setect
bool g_rod_self_shift_threshold_ok = false; // 拨杆自身的阈值计算是否ok

// 左右转向灯检测阈值计算
int g_turnlamp_detect_delay_num = 10; // 当第一次检测到拨杆可能被拨动，然后就持续检测后面的Ｎ个数据，就行比较判断当前拨杆的绝对位置
int g_turnlamp_detect_delay_counter = g_turnlamp_detect_delay_num;
double g_diff_acc_max = 100; // 最大的acc差值是2,大于这个数据不能用于进行拨杆绝对位置判断
int g_average_num_counter = 0; // 有效的用于计算均值的acc_diff

bool g_turnlamp_threshold_ok = false; // 左右拨杆的阈值是否计算ok
double g_acc_diff_left_threshold[3] = {0, 0, 0};
double g_acc_diff_right_threshold[3] = {0, 0, 0};
double g_acc_diff_middle_threshold[3] = {0, 0, 0};
double g_acc_diff_weight[3] = {0, 0, 0};
int g_acc_diff_key_index = -1; // 变化最大的值
bool g_is_left_threshold_uodate = false;
bool g_is_right_threshold_uodate = false;
#define DIFF_ACC_ERROR_RATIO 0.5 // 判断左右转向灯阈值的缩放系数



#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    // init
    google::InitGoogleLogging(argv[0]);
    init(); 

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();

    TurnlampDetector &m_turnlamp_detector = TurnlampDetector::Instance();
    #if !defined(DATA_FROM_LOG)
        m_turnlamp_detector.StartTurnlampDetectTaskOnline();
        printf("!!! Start Turnlamp Detect Task Online\n");
    #endif

    
    while(1){
        GetReadAccDataTime();
        DoCommondSwitch();
        int rod_shift_state = TurnlampDetector::Instance().m_rod_shift_state;
        if(TurnlampDetector::Instance().m_rod_shift_state){
            int cal_state = CalculateDiffAcc();
            if(cal_state){
                DiagnoseLeftRightTurnlamp();
            }
        }
        CalculateTurnlampThreshold();

    }
    printf("over!!\n");

    return 0;
}

void init()
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

    #if !defined(DATA_FROM_LOG)
        HalIO &halio = HalIO::Instance();
        bool res = halio.Init(NULL, 1);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            exit(0);
        }
        halio.EnableFMU(); // 使能CAN对应的filter 
    #endif

    // 设置PC端控制信号
    (void) signal(MACH_INIT, CalculateRodCameraRotation); 
    (void) signal(ROD_ACC_THRESHOLD, CalculateRodSelfShiftThreshold);     
    (void) signal(L_LAMP, CollectLeftTurnlampAccData);  
    (void) signal(R_LAMP, CollectRightTurnlampAccData); 
    
}

// 拨杆初始化坐标对准
// 1:对准结束
// 0: 正在对准
int CalculateRodRotationInitMatch()
{
    if(!g_is_camera_init_acc_data_collect_ok){ // 收集camera acc的数据
        usleep(20000);
        double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];
        int search_state = DataFusion::Instance().GetTimestampData(g_timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            if(g_camera_init_match_acc_data_counter++ < CAMERA_ACC_QUEUE_SIZE){
                for(int k=0; k<3; k++)
                    g_mean_init_camera_acc[k] += acc_camera[k];
                printf("index: %d amera acc data: %f %f %f\n", g_camera_init_match_acc_data_counter, acc_camera[0], acc_camera[1], acc_camera[2]);
            }else{
                for(int k=0; k<3; k++)
                    g_mean_init_camera_acc[k] = g_mean_init_camera_acc[k]/CAMERA_ACC_QUEUE_SIZE;
                g_is_camera_init_acc_data_collect_ok = true;
            }
        }else{
            return -1; 
        }
    }else{
        // camera acc数据已经ok，判断rod的数据
        bool is_rod_init_acc_data_ok = TurnlampDetector::Instance().m_is_init_road_acc_collect_ok;
        if(is_rod_init_acc_data_ok){
            TurnlampDetector::Instance().CalculateRotationFmu2Camera(TurnlampDetector::Instance().m_mean_init_rod_acc, g_mean_init_camera_acc);
            g_is_start_init_match = false;
            g_rotation_init_ok = true;
            printf("!!!!rod acc init data = %f %f %f\n", TurnlampDetector::Instance().m_mean_init_rod_acc[0], TurnlampDetector::Instance().m_mean_init_rod_acc[1], TurnlampDetector::Instance().m_mean_init_rod_acc[2]);
            printf("!!!!camera acc init data = %f %f %f\n", g_mean_init_camera_acc[0], g_mean_init_camera_acc[1], g_mean_init_camera_acc[2]);

            // reset state for next run
            g_is_camera_init_acc_data_collect_ok = false;
            g_camera_init_match_acc_data_counter = 0;
            memset(g_mean_init_camera_acc, 0 ,sizeof(*g_mean_init_camera_acc));
            TurnlampDetector::Instance().ResetInitMatchState();

            return 1;
        }else{
            usleep(100000);
        }
    }
    return 0; 
}

// 执行PC发出的指令
void DoCommondSwitch()
{
     // 初始化对准
    // 开启rod init match数据采集
    if(g_set_rod_init_match_start){
        TurnlampDetector::Instance().SartRotationInitDataCollect();
        g_set_rod_init_match_start = false;
    }

    // 开始拨杆拨动自检测
    if(g_start_d_rod_acc_calculate){
        TurnlampDetector::Instance().SartCalculateRodAccThreshold();
        g_start_d_rod_acc_calculate = false;
    }

    if(g_is_start_init_match){
        CalculateRodRotationInitMatch( );   
    }
}

// 计算rod和camera的diff acc
int CalculateDiffAcc()
{
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];
    // 拨杆可能被重新波动，重新计算均值
    g_turnlamp_detect_delay_counter = 0;
    g_average_num_counter = 0;
    memset(g_diff_acc_average, 0 , sizeof(g_diff_acc_average));
    g_calculate_new_acc_diff_mean = true;

    // 为了第一个能读到数据
    usleep(10000);
    
    while(g_calculate_new_acc_diff_mean){
        // 在计算diff均值的过程中，一旦拨杆检测到可能被拨动，则清零重新判断
        if(TurnlampDetector::Instance().m_rod_shift_state){
             // 拨杆可能被重新波动，重新计算均值
            g_turnlamp_detect_delay_counter = 0;
            g_average_num_counter = 0;
            memset(g_diff_acc_average, 0 , sizeof(g_diff_acc_average));
            g_calculate_new_acc_diff_mean = true;
    
        }else{
            // 分析检测到可能的波动之后 对拨杆的绝对位置进行判断
            if(g_average_num_counter < g_turnlamp_detect_delay_num){
                int search_state = DataFusion::Instance().GetTimestampData(g_timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
                if(search_state == 1){
                    for(int i=0; i<3; i++ )
                        g_diff_acc[i] = TurnlampDetector::Instance().m_fmu_imu_data.acc[i] - acc_camera[i];
                    
                    if(fabs(g_diff_acc[0]) < g_diff_acc_max && fabs(g_diff_acc[1]) < g_diff_acc_max && fabs(g_diff_acc[2]) < g_diff_acc_max){
                         for(int i=0; i<3; i++ )
                            g_diff_acc_average[i] += g_diff_acc[i]; // 计算均值
                            
                         g_average_num_counter++;
                         printf("Rod&Camera_diff_acc--loop index: %03d, value index: %03d, data: %f %f %f\n", g_turnlamp_detect_delay_counter, g_average_num_counter, g_diff_acc[0], g_diff_acc[1], g_diff_acc[2] );
                    }                    
                }else{
                    printf("!!! no new camera acc data\n");
                }
                g_turnlamp_detect_delay_counter += 1;
            }else {
                if(g_average_num_counter>0){ // 打印均值
                    for(int i=0; i<3; i++ )
                        g_diff_acc_average[i] = g_diff_acc_average[i]/g_average_num_counter;
                }
                g_calculate_new_acc_diff_mean = false; // 结束这次的计算

                VLOG(VLOG_INFO)<<"Main--"<<"average num: "<<g_average_num_counter<<endl;
                VLOG(VLOG_INFO)<<"Main--"<<"mean Fmu&Camera_diff_acc = "<<g_diff_acc_average[0]<<", "<<g_diff_acc_average[1]<<", "<<g_diff_acc_average[2]<<endl;
                printf("mean nums = %02d, g_diff_acc = %f %f %f\n", g_average_num_counter, g_diff_acc_average[0], g_diff_acc_average[1], g_diff_acc_average[2]);
                
                return 1;
            }

            // 为了防止无线循环
            if(g_turnlamp_detect_delay_counter>30){
                g_calculate_new_acc_diff_mean = false;
            }
            usleep(20000);
        }        
    }
    return 0;
}

// 获取当前循环需要读取数据的时间
void GetReadAccDataTime()
{
       // 读取camera imu数据
    #if defined(DATA_FROM_LOG)
        m_turnlamp_detector.RunDetectTurnlampOffline();
        double R_cur; // 在读取log日志的时候 推进data_fusion的时间戳;
        g_timestamp_search = m_turnlamp_detector.m_fmu_imu_data.timestamp;
        data_fusion.GetTurnRadius( (int64_t)(g_timestamp_search*1000), &R_cur);
    #else
        struct timeval time_imu;
        gettimeofday(&time_imu, NULL);
        g_timestamp_search = time_imu.tv_sec + time_imu.tv_usec*1e-6;
    #endif
}

void CalculateTurnlampThreshold()
{
    // 计算左右转向灯的阈值
    if(g_left_turnlamp_init_state == 1 || g_right_turnlamp_init_state == 1){
        int cal_state = CalculateDiffAcc();
        printf("run CalculateTurnlampThreshold, cal_state=%d\n", cal_state);
        
        if(cal_state){
            if(g_left_turnlamp_init_state == 1){
                memcpy(g_acc_diff_left_threshold, g_diff_acc_average, sizeof(g_acc_diff_left_threshold));
                g_left_turnlamp_init_state = 2;
                printf("temp g_acc_diff_left_threshold: %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            }else if(g_right_turnlamp_init_state == 1){
                memcpy(g_acc_diff_right_threshold, g_diff_acc_average, sizeof(g_acc_diff_right_threshold));
                g_right_turnlamp_init_state = 2;
                printf("temp g_acc_diff_right_threshold: %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            }
        }
    
        if(g_left_turnlamp_init_state ==2 && g_right_turnlamp_init_state != 2){
            printf("!!!!!! please do the right turnlamp threshold init\n");
        }else if(g_left_turnlamp_init_state !=2 && g_right_turnlamp_init_state == 2){
            printf("!!!!!! please do the left turnlamp threshold init\n");
        }else if(g_left_turnlamp_init_state ==2 && g_right_turnlamp_init_state == 2){
            double dacc_abs_t[3];
            for(int i=0; i<3; i++)
                dacc_abs_t[i] = fabs(g_acc_diff_left_threshold[i] - g_acc_diff_right_threshold[i]);
    
            // 挑选最大的权值对应的轴
            double max_threshold = max(dacc_abs_t[0], max(dacc_abs_t[1], dacc_abs_t[2]));
            for(int i=0; i<3; i++){
                if(max_threshold == dacc_abs_t[i]){
                    g_acc_diff_weight[i] = 1;
                    g_acc_diff_key_index = i;
                }else{
                    g_acc_diff_weight[i] = 0;
                }
            }

            // 更新diff_max
            g_diff_acc_max = fabs(g_acc_diff_left_threshold[g_acc_diff_key_index]) + fabs(g_acc_diff_right_threshold[g_acc_diff_key_index]);
            g_turnlamp_threshold_ok = true;
            // reset state
            g_left_turnlamp_init_state = 0;
            g_right_turnlamp_init_state = 0;

            printf("!!!!---new g_diff_acc_max: %f\n", g_diff_acc_max);
            printf("!!!!---new g_acc_diff_left_threshold: %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            printf("!!!!---new g_acc_diff_right_threshold: %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            printf("!!!!---new g_acc_diff_weight: %f %f %f\n", g_acc_diff_weight[0], g_acc_diff_weight[1], g_acc_diff_weight[2]);
        }
    }
    
}

// 判断是左还是右转向
void DiagnoseLeftRightTurnlamp()
{
    if(g_rotation_init_ok && g_rod_self_shift_threshold_ok && g_turnlamp_threshold_ok){

        // 判断left or right
        double d_left[3], d_right[3], d_middle[3];
        double d_left_key, d_right_key;

        d_left_key = g_diff_acc_average[g_acc_diff_key_index] - g_acc_diff_left_threshold[g_acc_diff_key_index];
        d_right_key = g_diff_acc_average[g_acc_diff_key_index] - g_acc_diff_right_threshold[g_acc_diff_key_index];

        printf("d_left_key = %f, d_right_key = %f\n", d_left_key, d_right_key);
        if(fabs(d_left_key) <= DIFF_ACC_ERROR_RATIO*fabs(g_acc_diff_left_threshold[g_acc_diff_key_index])){
            printf("d_left_key: %f\n", d_left_key);
            printf("!!!!------LEFT------!!!!!\n");
        }else if(fabs(d_right_key) <= DIFF_ACC_ERROR_RATIO*fabs(g_acc_diff_right_threshold[g_acc_diff_key_index])){
            printf("d_right_key: %f\n", d_right_key);
            printf("!!!!------RIGHT------!!!!!\n");
        }else{
            printf("!!!!------Middle------!!!!!\n");
        }
    }
}

void CollectLeftTurnlampAccData(int sig)  
{  
    g_left_turnlamp_init_state = 1;
    printf("\n state = %d, start left turnlamp collect !!!!\n", sig);    
}  

void CollectRightTurnlampAccData(int sig)  
{  
    g_right_turnlamp_init_state = 1;
    printf("\n state = %d, start right turnlamp collect !!!!\n", sig);    
}  

void CalculateRodCameraRotation(int sig)  
{  
    g_is_start_init_match = true;
    g_set_rod_init_match_start = true;
    printf("\n state = 0 %d, start rod init match!!\n", sig);    
} 


void CalculateRodSelfShiftThreshold(int sig)  
{  
    g_start_d_rod_acc_calculate = true;
    g_rod_self_shift_threshold_ok = true;
    printf("\n state = 0 %d, start rod acc calculate!!\n", sig);    
} 

