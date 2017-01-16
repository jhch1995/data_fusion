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

//struct StructImuParameter
//{
//   double gyro_bias[3];
//   double acc_A0[3];
//   double acc_A0[3][3];
//};

string g_file_addr = "/storage/sdcard0/imu/imu.flag";  // = "./gflags.flag";

//#pragma pack(1)    
//struct StructImuData
//{
//    double timestamp;
//    double acc_raw[3];
//    double gyro_raw[3];
//    double acc[3];
//    double gyro[3];
//    double temp;
//};
//#pragma pack()

int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter);
int read_imu_calibation_parameter( StructImuParameter *imu_parameter);
//DEFINE_string(fileflag, "./imu1.flag", "imu gyro bias z ");
int main(int argc, char *argv[])
{
    // 初始化
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
//    FLAGS_log_dir = "./log/";
//    #if defined(USE_GLOG)
//        FLAGS_v = VLOG_DEBUG; // VLOG_DEBUG;
//    #endif

    // 进行数据融合的类
//    DataFusion data_fusion;

    #if defined(DATA_FROM_LOG)
//        data_fusion.StartDataFusionTask( );
        ImuModule::Instance().StartDataFusionTask();
    #else
        //读取车速测试
//        HalIO &halio = HalIO::Instance();
//        bool res = halio.Init(NULL, MOBILEEYE);
//        if (!res) {
//            std::cerr << "HALIO init fail" << std::endl;
//            return -1;
//        }
        // 测试gflags读取配置并修改   
//        data_fusion.StartDataFusionTask( );
        ImuModule::Instance().StartDataFusionTask();
    #endif

    #if defined(DATA_FROM_LOG)
    {
        // 读入log
        ifstream infile_log("data/doing/log.txt");  // 指定log的路径
        string buffer_log;
        string data_flag;
        stringstream ss_log;
        stringstream ss_tmp;
        double R_cur;
        double image_timestamp;

        string log_data_t[2];
        while(!infile_log.eof()){
            getline(infile_log, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>log_data_t[0]>>log_data_t[1]>>data_flag;
            ss_log.clear();
            ss_log.str(buffer_log);

            if(data_flag == "cam_frame"){
                int camera_raw_timestamp[2];
                string camera_flag, camera_add, image_index_str, image_name;
                ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>image_index_str;
                image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;

                // 执行查询转弯半径
                int64_t image_timestamp_cur_int = (int64_t)(image_timestamp*1000);
                int r_1  = ImuModule::Instance().GetTurnRadius( image_timestamp_cur_int*1000, &R_cur);
//                int r_1 = data_fusion.GetTurnRadius( image_timestamp_cur_int, &R_cur);
                if(r_1 <= 0){
                    printf("main timestamp dismatch, match state= %d, so sleep\n", r_1);
                    sleep(1);
                }else{
//                    printf("state: r_state,  R = %f\n", R_cur);
                }
            }
            usleep(10);
        }
        printf("radius calculate over!!\n");
        infile_log.close();
    }
    #else
    {
        TimeUtils f_time_counter;
        int64_t t_1;
        int r_state = -1;
        double R_cur;
        while(1){
            t_1 = f_time_counter.Microseconds();
            usleep(50000); // 50ms
            r_state = ImuModule::Instance().GetTurnRadius( t_1*1000, &R_cur);
//            r_state = data_fusion.GetTurnRadius( t_1, &R_cur);
        }
    }
    #endif

    return 0;
}


int read_imu_calibation_parameter( StructImuParameter *imu_parameter)
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

        //int len = GET_ARRAY_LEN(gyro_bias);
        //memcpy(&(imu_parameter->gyro_bias[0]), gyro_bias, GET_ARRAY_LEN(gyro_bias)); // 为什么有问题
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


int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter)
{
    // 文件格式: gyro_bias gyro_bias[0] gyro_bias[1] gyro_bias[2]
    char buffer[100]; 
    // clear content
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
//          sprintf(buffer, "gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0],  imu_parameter.gyro_bias[1],  imu_parameter.gyro_bias[2]);
        file_imu << buffer; 
        printf("write new gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0], imu_parameter.gyro_bias[1], imu_parameter.gyro_bias[2] );
    }else{
        printf("write new gyro_bias failed!!\n");
    }

    file_imu.close();
    return 1;
}

