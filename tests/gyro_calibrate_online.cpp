#include <dirent.h>
#include <getopt.h> 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/types.h>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/hal/halio.h"

#include "data_fusion.h"
#include "datafusion_math.h"

using namespace imu;

string g_file_addr = "/storage/sdcard0/imu/imu.flag";  // = "./gflags.flag";

// gflog

int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter);

int read_imu_calibation_parameter( StructImuParameter *imu_parameter);

int main(int argc, char *argv[])
{       
    // 进行数据融合的类
    DataFusion data_fusion;    
    data_fusion.Init();
    data_fusion.SetImuVersionMode(2);

    #if defined(ANDROID)
        //读取车速测试
//         HalIO &halio = HalIO::Instance();
//         bool res = halio.Init(NULL, 1);
//         if (!res) {
//             std::cerr << "HALIO init fail" << std::endl;
//             return -1;
//         } 
    #endif

    // 校正
    StructImuParameter imu_parameter_pre, imu_parameter_new;
    #if defined(ANDROID)
        double gyro_A0[3];
        data_fusion.CalibrateGyroBias(gyro_A0);        
        //read_imu_calibation_parameter(&imu_parameter_pre); 
//         memcpy(imu_parameter_new.gyro_A0, gyro_A0, sizeof(gyro_A0));
//         wite_imu_calibation_parameter(imu_parameter_new);

        // 测试gflags读取配置并修改   
//         google::ParseCommandLineFlags(&argc, &argv, true);
        printf("gyro_A0= %f, %f, %f\n", gyro_A0[0], gyro_A0[1], gyro_A0[2]);         
    #endif

    while(1){
        printf("imu calibrate success!!\n");
        sleep(1); 
    }
    return 0;
}


int read_imu_calibation_parameter( StructImuParameter *imu_parameter)
{
    string buffer_log;
    stringstream ss_log;
    string data_flag;  
    double gyro_A0[3];      

    ifstream file_imu (g_file_addr.c_str());
    if(file_imu.is_open()){
        getline(file_imu, buffer_log);
        ss_log.clear();
        ss_log.str(buffer_log);
        ss_log>>data_flag>>gyro_A0[0]>>gyro_A0[1]>>gyro_A0[2]; 

        //int len = GET_ARRAY_LEN(gyro_A0);
        //memcpy(&(imu_parameter->gyro_A0[0]), gyro_A0, GET_ARRAY_LEN(gyro_A0)); // 为什么有问题
        imu_parameter->gyro_A0[0] = gyro_A0[0];
        imu_parameter->gyro_A0[1] = gyro_A0[1];
        imu_parameter->gyro_A0[2] = gyro_A0[2];
        printf("read imu parameter %s, %f %f %f\n", data_flag.c_str(), imu_parameter->gyro_A0[0], imu_parameter->gyro_A0[1], imu_parameter->gyro_A0[2]);
    }else{
        printf("open file error!!!\n");
    }
    file_imu.close();
    return 1;
}


int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter)
{
    // 文件格式: gyro_A0 gyro_A0[0] gyro_A0[1] gyro_A0[2]
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
                imu_parameter.gyro_A0[0],  imu_parameter.gyro_A0[1],  imu_parameter.gyro_A0[2]);
//          sprintf(buffer, "gyro_A0 %f %f %f\n", imu_parameter.gyro_A0[0],  imu_parameter.gyro_A0[1],  imu_parameter.gyro_A0[2]);
        file_imu << buffer; 
        printf("write new gyro_A0 %f %f %f\n", imu_parameter.gyro_A0[0], imu_parameter.gyro_A0[1], imu_parameter.gyro_A0[2] );
    }else{
        printf("write new gyro_A0 failed!!\n");
    }

    file_imu.close();
    return 1;
}

