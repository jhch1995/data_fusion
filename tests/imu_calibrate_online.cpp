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

#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace imu;

struct StructImuParameter
{
   double gyro_bias[3];
//   double acc_A0[3];
//   double acc_A0[3][3];
};

#pragma pack(1)
struct StructImuData
{
    double timestamp;
    double acc_raw[3];
    double gyro_raw[3];
    double acc[3];
    double gyro[3];
    double temp;
};
#pragma pack()

//int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter);
//int read_imu_calibation_parameter( StructImuParameter *imu_parameter);

int main(int argc, char *argv[])
{
    // 初始化
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

//    #if defined(USE_GLOG)
//        FLAGS_log_dir = "./log/";
//        FLAGS_v = 0; // VLOG_DEBUG;
//    #endif

    // imu的acc参数
    double A0[3] = {0.03312, 0.00830, 0.02780};
    double A1[3][3] = {1.00094091021825, -0.00363380068824405, -0.00990621587281586, 0.00634451729910714,
                        1.00072829473586, -0.0320083650978663, 0.0142462642609127, 0.0270694986979167,1.00604358303931};
    // 进行数据融合的类
    DataFusion data_fusion;
    data_fusion.SetAccCalibationParam(A0, A1);
    data_fusion.StartDataFusionTask( );
    TimeUtils f_time_counter;
    int64_t t_ms;
    int r_state = -1;
    double R_cur;
    double pos[2], att[3], gyro_z, gyro_att[3];
    while(1){
        t_ms = f_time_counter.Milliseconds();
        double  t_1 = (double)t_ms/1000.0;
        usleep(50000); // 50ms
        r_state = data_fusion.GetTurnRadius( t_ms, &R_cur);
        data_fusion.GetTimestampData(t_1, pos, att, &gyro_z, gyro_att);
//        printf("time: %f %lld att: %f %f %f gyro_att: %f %f\n",t_1, t_ms, att[0], att[1], att[2], gyro_att[0], gyro_z);
        printf("att: %f %f %f \n", att[0]*R2D-0.4, att[1]*R2D+1.8, att[2]*R2D);
    }

    return 0;
}


//int read_imu_calibation_parameter( StructImuParameter *imu_parameter)
//{
//    string buffer_log;
//    stringstream ss_log;
//    string data_flag;
//    double gyro_bias[3];
//
//    ifstream file_imu (g_file_addr.c_str());
//    if(file_imu.is_open()){
//        getline(file_imu, buffer_log);
//        ss_log.clear();
//        ss_log.str(buffer_log);
//        ss_log>>data_flag>>gyro_bias[0]>>gyro_bias[1]>>gyro_bias[2];
//
//        int len = GET_ARRAY_LEN(gyro_bias);
//        memcpy(&(imu_parameter->gyro_bias[0]), gyro_bias, GET_ARRAY_LEN(gyro_bias)); // 为什么有问题
//        imu_parameter->gyro_bias[0] = gyro_bias[0];
//        imu_parameter->gyro_bias[1] = gyro_bias[1];
//        imu_parameter->gyro_bias[2] = gyro_bias[2];
//        printf("read imu parameter %s, %f %f %f\n", data_flag.c_str(), imu_parameter->gyro_bias[0], imu_parameter->gyro_bias[1], imu_parameter->gyro_bias[2]);
//    }else{
//        printf("open file error!!!\n");
//    }
//    file_imu.close();
//    return 1;
//}
//
//
//int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter)
//{
//     文件格式: gyro_bias gyro_bias[0] gyro_bias[1] gyro_bias[2]
//    char buffer[100];
//     clear content
//    fstream file_imu;
//    file_imu.open(g_file_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
//    if (!file_imu.is_open() || file_imu.fail())
//    {
//        file_imu.close();
//        printf("Error : failed to erase file content !");
//    }
//    file_imu.close();
//
//    file_imu.open(g_file_addr.c_str());
//    if (file_imu.is_open()) {
//        sprintf(buffer, "--gyro_bias_x=%f\n--gyro_bias_y=%f\n--gyro_bias_z=%f\n",
//                imu_parameter.gyro_bias[0],  imu_parameter.gyro_bias[1],  imu_parameter.gyro_bias[2]);
//          sprintf(buffer, "gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0],  imu_parameter.gyro_bias[1],  imu_parameter.gyro_bias[2]);
//        file_imu << buffer;
//        printf("write new gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0], imu_parameter.gyro_bias[1], imu_parameter.gyro_bias[2] );
//    }else{
//        printf("write new gyro_bias failed!!\n");
//    }
//
//    file_imu.close();
//    return 1;
//}

