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

// 6个面的顺序: 1:-Y 2:-Z 3:+X 4:+Y 5:-X 6:+Z
#define NEGATIVE_Y 1
#define NEGATIVE_Z 2
#define POSITIVE_X 3
#define POSITIVE_Y 4
#define NEGATIVE_X 5
#define POSITIVE_Z 6

using namespace imu;
using namespace Eigen;  
using namespace std; 

//---------------------variable--------------------------------//
string g_file_addr = "/storage/sdcard0/imu/imu.flag";  // = "./gflags.flag";

bool g_is_print_imu_data = false;
RowVectorXd g_acc_average_save(3), g_gyro_average_save(3);// 6个面的均值
//g_acc_average_save.setZero();
MatrixXd g_acc_A0(3, 1), g_acc_A1(3,3);

// 6个面的顺序: 1:-Y 2:-Z 3:+X 4:+Y 5:-X 6:+Z
int g_acc_calibrate_index = 0 ;// 用于表示正在校正哪个面  0表示刚开始

bool g_acc_average_data_ready = false; // 是否收集完数据

//----------------------function-------------------------------//
// 获取当前循环需要读取数据的时间
void GetReadAccDataTime(double* timestamp);

int GetIMUAverageData(const int data_num, RowVectorXd &acc_average_save, RowVectorXd &gyro_average_save);

void TryPrintImuData(int sig);

void PrintImuData();

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif
// 获取当前循环需要读取数据的时间
void GetReadAccDataTime(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}

// 0: 错误
// 1: 当前面数据采集结束
// 2: 6个面数据都采集结束
// 获取平均数据,并判断数据的有效性
int GetIMUAverageData(const int data_num, RowVectorXd &acc_average_save, RowVectorXd &gyro_average_save)
{
    if(data_num <=0 )
        return 0;
    double timestamp;
    GetReadAccDataTime(&timestamp);

    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3],  gyro_camera[3];
    bool read_acc_data = true;
    int read_acc_data_index = 0;
    RowVectorXd acc_vector_tmp(3), acc_vector_sum_tmp(3);
    acc_vector_sum_tmp.setZero();
    RowVectorXd gyro_vector_tmp(3), gyro_vector_sum_tmp(3);
    gyro_vector_sum_tmp.setZero();
    
    RowVectorXd acc_standard(3);// 当前面下acc标准值
    acc_standard << 1, 0, 0;  
    
    bool is_first_acc_data = true;
    while(read_acc_data){
        if(is_first_acc_data){
            usleep(10000);
            is_first_acc_data = false;
        }
        
        int search_state = DataFusion::Instance().GetTimestampData(timestamp, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            // 判断数据的有效性
            for(int j = 0; j<3; j++){
                acc_vector_tmp(j) = acc_camera[j]/ONE_G;
                gyro_vector_tmp(j) = gyro_camera[j];
            }
            RowVectorXd acc_diff(3);
            acc_diff = acc_vector_tmp - acc_standard;
            double acc_diff_normal = acc_diff.norm();
            // 对平面就行判断
            if(acc_diff_normal < 0.2){
                // 满足条件，则进行累加求和
                acc_vector_sum_tmp += acc_vector_tmp;
                gyro_vector_sum_tmp += gyro_vector_tmp;
            }else{
                // 不满足条件 退出这次采集数据
                read_acc_data_index = 0;
                acc_vector_sum_tmp.setZero();
                gyro_vector_sum_tmp.setZero();
                printf("Make sure the module is placed level，and place the camerad to sky，then press next!!!\n");
                fflush(stdout);
                return 0;
            }
            read_acc_data_index++;
        }else{
            printf("no new acc data!!\n");
            fflush(stdout);
        }
        
        if(read_acc_data_index >= data_num){
            acc_average_save = acc_vector_sum_tmp/read_acc_data_index;
            gyro_average_save = gyro_vector_sum_tmp/read_acc_data_index;
            read_acc_data = false;
            return 1;
        }
        GetReadAccDataTime(&timestamp);
        usleep(20000);
    }
    return 0;    
}


void TryPrintImuData(int sig)
{ 
    g_is_print_imu_data = !g_is_print_imu_data;
}

void PrintImuData()
{
    double timestamp;
    GetReadAccDataTime(&timestamp);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3],  gyro_camera[3];
    int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
    if(search_state == 1){
        printf("acc: %0.2f %0.2f %0.2f gyro: %0.2f %0.2f %0.2f\n", acc_camera[0], acc_camera[1], acc_camera[2], gyro_camera[0]*R2D,  gyro_camera[1]*R2D,  gyro_camera[2]*R2D);
        fflush(stdout);
    }
}

int main(int argc, char *argv[])
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif
            
    (void) signal(4, TryPrintImuData);  

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.Init();
    data_fusion.SetGyroAutoCalibrateState(0); // 停止自动校准
    data_fusion.ResetImuParameter();
    data_fusion.SetImuVersionMode(2); // 设置摄像头模组为新镜头
    
    if(g_acc_calibrate_index == 0){
         printf("Place camera up to sky，then press next  !!!\n");
         fflush(stdout);
    }

    #if !defined(DATA_FROM_LOG)
        bool is_imu_test_over = false;
        while(!is_imu_test_over){        
            char buf[80];
            scanf("%s", buf);
            if (strcmp(buf, "n") == 0){
                int get_acc_average_state = GetIMUAverageData(50, g_acc_average_save, g_gyro_average_save);
                if(abs(g_acc_average_save(0)-1)<0.2 && abs(g_acc_average_save(1))<0.2 && abs(g_acc_average_save(2))<0.2 &&
                    abs(g_gyro_average_save(1))<0.1 && abs(g_gyro_average_save(1))<0.1 && abs(g_gyro_average_save(1))<0.1 ){
                    printf("the imu is OK !!!\n");
                    fflush(stdout);
                }else{
                    printf("the imu is error !!!\n");
                    fflush(stdout);
                }
                is_imu_test_over = true;
            }else{
                usleep(5000);
            }
        }
        // print imu data
        while(g_is_print_imu_data){
            PrintImuData();
            usleep(100000);
        }
            
    #endif

    data_fusion.Destory();
    printf("exe over\n");
    fflush(stdout);
    return 0;
}

