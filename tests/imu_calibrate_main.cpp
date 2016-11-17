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

struct StructImuParameter
{
   double gyro_bias[3];
//   double acc_A0[3];
//   double acc_A0[3][3];
};

string g_file_addr = "/storage/sdcard0/imu/imu.flag";  // = "./gflags.flag";

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

// gflog
DEFINE_double(gyro_bias_x, 0.00897, "imu gyro bias x ");
DEFINE_double(gyro_bias_y, -0.0322, "imu gyro bias y ");
DEFINE_double(gyro_bias_z, -0.0214, "imu gyro bias z ");
DEFINE_string(imu_parameter_addr, "./imu.flag", "imu parameter address ");


int wite_imu_calibation_parameter(const StructImuParameter &imu_parameter);

int read_imu_calibation_parameter( StructImuParameter *imu_parameter);

int main(int argc, char *argv[])
{       
    // 进行数据融合的类
    DataFusion data_fusion;    

    #if defined(ANDROID)
    {
        //读取车速测试
        HalIO &halio = HalIO::Instance();
        bool res = halio.Init(NULL, MOBILEEYE);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            return -1;
        } 
    }          
    #endif

    // 校正
    StructImuParameter imu_parameter_pre, imu_parameter_new;    
    #if defined(ANDROID)
    {
        double gyro_bias[3];
        data_fusion.CalibrateGyroBias(gyro_bias);        
        //read_imu_calibation_parameter(&imu_parameter_pre); 
        memcpy(imu_parameter_new.gyro_bias, gyro_bias, sizeof(gyro_bias));
        wite_imu_calibation_parameter(imu_parameter_new);

        // 测试gflags读取配置并修改   
        google::ParseCommandLineFlags(&argc, &argv, true);
        printf("FLAG: gyro_bias= %f, %f, %f\n", FLAGS_gyro_bias_x, FLAGS_gyro_bias_y, FLAGS_gyro_bias_z);         
    }
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
}

