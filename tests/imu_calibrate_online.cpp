#include <dirent.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/hal/halio.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include <Eigen/Dense>  
#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace imu;
using namespace Eigen;  
using namespace std; 

//---------------------variable--------------------------------//
string g_file_addr = "/storage/sdcard0/imu/imu.flag";  // = "./gflags.flag";

MatrixXd g_acc_calibrate_sequence(6,3);// 校准的acc的哪个面
int g_acc_calibrate_state = 0; // 用于表示正在校正哪个面


//----------------------function-------------------------------//

// 获取当前循环需要读取数据的时间
void GetReadAccDataTime(double* timestamp);

int GetAccAverageData(const int data_num);

// 继续读取acc数据 用于校正
void ContinueReadAccAverageData(int sig);  



int WiteImuCalibationParameter(const StructImuParameter &imu_parameter);
int ReadImuCalibationParameter( StructImuParameter *imu_parameter);



#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    // 初始化
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

    (void) signal(1, ContinueReadAccAverageData); 

    g_acc_calibrate_sequence<<0,-1,0,
                             0,0,-1,
                             1,0,0,
                             0,1,0,
                            -1,0,0,
                             0,0,1;
    cout<<g_acc_calibrate_sequence;
    // imu的acc参数
    double A0[3] = {0.03312, 0.00830, 0.02780};
    double A1[3][3] = {1.00094091021825, -0.00363380068824405, -0.00990621587281586, 0.00634451729910714,
                        1.00072829473586, -0.0320083650978663, 0.0142462642609127, 0.0270694986979167,1.00604358303931};
    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();

    #if !defined(DATA_FROM_LOG)

    #endif
//    data_fusion.StartDataFusionTask( );

    #if defined(DATA_FROM_LOG)

    #else
        TimeUtils f_time_counter;
        int64_t t_1;
        int r_state = -1;
        double R_cur;
//        data_fusion.PrintImuData(1);
        while(1){
            t_1 = f_time_counter.Microseconds();
            usleep(50000); // 50ms
            r_state = data_fusion.GetTurnRadius( t_1, &R_cur);
        }
    #endif

    return 0;
}


// 获取当前循环需要读取数据的时间
void GetReadAccDataTime(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}

// 获取平均数据
int GetAccAverageData(const int data_num)
{
    if(data_num <=0)
        return 0;
    double timestamp;
    GetReadAccDataTime(&timestamp);

    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    double acc_sum[3] = {0, 0, 0};
    bool read_acc_data = true;
    bool is_firat_acc_data = true;
    int read_acc_data_index = 0;
    while(read_acc_data){
        int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            // 判断数据的有效性
            read_acc_data_index++;
        }
        
        // 读取足都的数据后退出
        if(read_acc_data_index > data_num)
            read_acc_data = false;
        usleep(30000);
    }
    return 0;
    
}


int ReadImuCalibationParameter( StructImuParameter *imu_parameter)
{
    string buffer_log;
    stringstream ss_log;
    string data_flag;
    double gyro_bias[3];

    ifstream file_imu (g_file_addr.c_str());
    if(file_imu.is_open()){
        getline(file_imu, buffer_log);
        ss_log.clear();
        ss_log.str(buffer_log);
        ss_log>>data_flag>>gyro_bias[0]>>gyro_bias[1]>>gyro_bias[2];

        int len = GET_ARRAY_LEN(gyro_bias);
        memcpy(&(imu_parameter->gyro_bias[0]), gyro_bias, GET_ARRAY_LEN(gyro_bias)); // 为什么有问题
        imu_parameter->gyro_bias[0] = gyro_bias[0];
        imu_parameter->gyro_bias[1] = gyro_bias[1];
        imu_parameter->gyro_bias[2] = gyro_bias[2];
        printf("read imu parameter %s, %f %f %f\n", data_flag.c_str(), imu_parameter->gyro_bias[0], imu_parameter->gyro_bias[1], imu_parameter->gyro_bias[2]);
    }else{
        printf("open file error!!!\n");
    }
    file_imu.close();
    return 1;
}


int WiteImuCalibationParameter(const StructImuParameter &imu_parameter)
{
     //文件格式: gyro_bias gyro_bias[0] gyro_bias[1] gyro_bias[2]
    char buffer[100];
    //clear content
    fstream file_imu;
    file_imu.open(g_file_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
    if (!file_imu.is_open() || file_imu.fail())
    {
        file_imu.close();
        printf("Error : failed to erase file content !");
    }
    file_imu.close();

    file_imu.open(g_file_addr.c_str());
    if (file_imu.is_open()) {
        sprintf(buffer, "--gyro_bias_x=%f\n--gyro_bias_y=%f\n--gyro_bias_z=%f\n",
                imu_parameter.gyro_bias[0],  imu_parameter.gyro_bias[1],  imu_parameter.gyro_bias[2]);
          sprintf(buffer, "gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0],  imu_parameter.gyro_bias[1],  imu_parameter.gyro_bias[2]);
        file_imu << buffer;
        printf("write new gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0], imu_parameter.gyro_bias[1], imu_parameter.gyro_bias[2] );
    }else{
        printf("write new gyro_bias failed!!\n");
    }

    file_imu.close();
    return 1;
}


void ContinueReadAccAverageData(int sig)
{
    if(g_acc_calibrate_state == 0)
        printf("你好\n");
    
}

