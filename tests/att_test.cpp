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
using namespace Eigen;  
using namespace std; 

void GetTimeS(double* timestamp);

void TryPrintData(int sig);

void SaveImuAttData();

void SaveVn300Data( );

void SaveExData( );


void init();


//---------------------variable--------------------------------//
string g_att_log = "/storage/sdcard1/att.txt";  // = "./gflags.flag";
fstream g_file_log;

bool g_is_print_data = false;

MatrixXd g_acc_calibrate_sequence(6,3);// 校准的acc的哪个面

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    // 初始化
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    init();

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();

    #if !defined(DATA_FROM_LOG)
        while(1){
            SaveVn300Data();
            SaveExData();
            SaveImuAttData();
            usleep(20000);
        }
    #endif
    
    g_file_log.close();
    printf("exe over\n");
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

    // SIGTSTP: Ctrl+Z   SIGTINI: Ctrl+C
    (void) signal(SIGTSTP, TryPrintData);  
    
    // log
    g_file_log.open(g_att_log.c_str());
    if (!g_file_log.is_open() || g_file_log.fail())
    {
        g_file_log.close();
        printf("Error: failed to open log file !\n");
        exit(0);
    }

    
    HalIO &halio = HalIO::Instance();
    // string config_file_path = "./golf.json";
    //const char config_file_path[20] = "./golf.json";
    //HalioInitInfo info;
    //int state = halio.LoadCANSignalConfig(config_file_path, &info);
    //bool res = halio.Init(&info, 1);
    bool res = halio.Init(NULL, 1);
    if (!res) {
        std::cerr << "HALIO init fail" << std::endl;
        exit(0);
    }
    halio.EnableFMU(); // 使能CAN对应的filter 
}


// 获取当前循环需要读取数据的时间
void GetTimeS(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}

void TryPrintData(int sig)
{ 
    g_is_print_data = !g_is_print_data;
        
}

void SaveImuAttData()
{
    double timestamp;
    GetTimeS(&timestamp);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
    if(search_state == 1){
        // ACC to angle
        double acc_angle[3];
        acc_angle[0] = (atan2f(-acc_camera[1], -acc_camera[2]));       // Calculating pitch ACC angle
        acc_angle[1] = (atan2f(acc_camera[0], sqrtf(acc_camera[2]*acc_camera[2] + acc_camera[1]*acc_camera[1])));   //Calculating roll ACC angle    
//        printf("gyro: %5.2f %5.2f %5.2f ", gyro_camera[0]*R2D,  gyro_camera[1]*R2D,  gyro_camera[2]*R2D);
//        printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);
//        printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);
        if (g_file_log.is_open()) {
           char buffer[100];
           sprintf(buffer, "att %f %5.2f %5.2f %5.2f\n", timestamp-0.01, att[0]*R2D, att[1]*R2D, att[2]*R2D);
           g_file_log << buffer;
           
           memset(buffer, 0, sizeof(buffer));
           sprintf(buffer, "imu %f %f %f %f %f %f %f\n", timestamp-0.01, acc_camera[0], acc_camera[1], acc_camera[2], gyro_camera[0], gyro_camera[1], gyro_camera[2]);
           g_file_log << buffer;
           
        }

        if(g_is_print_data)
            printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);

    }
}


void SaveVn300Data( )
{
    ROD_DATA rod_data[20];
    double VN300_att[3];
    int read_rod_state = HalIO::Instance().ReadRodData(rod_data, 20);
    if(read_rod_state){
        memcpy(VN300_att, rod_data[read_rod_state-1].acc, sizeof(VN300_att));
        for(int i = 0; i < 3; i++)
            VN300_att[i] = rod_data[read_rod_state-1].acc[i]/100.0;
        if (g_file_log.is_open()) {
            char buffer[100];
            struct timeval time_t;
            time_t = rod_data[read_rod_state-1].tv;
            double time_rod = time_t.tv_sec + time_t.tv_usec*1e-6;
            sprintf(buffer, "VN300 %f %5.2f %5.2f %5.2f\n",time_rod, VN300_att[0], VN300_att[1], VN300_att[2]);
            g_file_log << buffer;
        }

        if(g_is_print_data)
            printf("VN300 att: %5.2f %5.2f %5.2f\n", VN300_att[0], VN300_att[1], VN300_att[2]);
    }
}

void SaveExData( )
{
    EX_DATA ex_data[20];
    double att[3];
    int read_state = HalIO::Instance().ReadExData(ex_data, 20);
    if(read_state){
        memcpy(att, ex_data[read_state-1].data, sizeof(att));
        for(int i = 0; i < 3; i++)
            att[i] = ex_data[read_state-1].data[i]/10.0;
        if (g_file_log.is_open()) {
            char buffer[100];
            struct timeval time_t;
            time_t = ex_data[read_state-1].tv;
            double time_rod = time_t.tv_sec + time_t.tv_usec*1e-6;
            sprintf(buffer, "fmu %f %5.2f %5.2f %5.2f\n",time_rod, att[0], att[1], att[2]);
            g_file_log << buffer;
        }

        if(g_is_print_data)
            printf("fmu: %5.2f %5.2f %5.2f\n", att[0], att[1], att[2]);
    }
}

