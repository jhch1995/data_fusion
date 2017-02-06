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


// global variables
static const char* program = "data_fudion_test";
static const char* version = "0.1.0";
static const char* usage = "Usage: %s [options]\n"
"\n"
"Options:\n"
"  -h, --help                   Print this help message and exit\n"
"  -v, --version                Print version message and exit\n"
"  -i, --imu                    Print the imu data\n"
"  -s, --speed                  Print the speed data\n"

"\n";

//struct StructImuParameter
//{
//   double gyro_bias[3];
   //double acc_A0[3];
   //double acc_A1[3][3];
//};

string g_file_addr = "/mnt/media_rw/sdcard1/imu_calibulation_paramer.ini";


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



void speed_callback(struct timeval *tv, int type, float speed)
{

}

using namespace imu;

int main(int argc, char *argv[])
{
    // 进行数据融合的类
    DataFusion data_fusion;    

    #if defined(ANDROID)
    {
        int show_help = 0;
        int show_version = 0;
        int is_print_imu = 0;
        int is_print_speed = 0;

        static struct option long_options[] = {
            {"help", no_argument, &show_help, 'h'},
            {"version", no_argument, &show_version, 'v'},
            {"imu", no_argument, &is_print_imu, 'i'},
            {"speed", no_argument, &is_print_speed, 's'},
            {0, 0, 0, 0}
        };

        while (true){
            // "hvi:"  :表示i后面要跟参数
            int opt = getopt_long(argc, argv, "hvis", long_options, NULL);
            if (opt == -1){
                break;
            }else if (opt == 'h'){
                printf(usage, program);
                exit(EXIT_SUCCESS);
            }else if (opt == 'v'){
                printf("%s version %s\n", program, version);
                exit(EXIT_SUCCESS);
            } else if (opt == 'i') {
                is_print_imu = 1;
            } else if (opt == 's'){            
                is_print_speed = 1;
            } else{  // 'h'
                fprintf(stderr, usage, program);
                exit(EXIT_FAILURE);
            }
        }

        //读取车速
        HalIO &halio = HalIO::Instance();
        bool res = halio.Init(speed_callback, MOBILEEYE);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            return -1;
        } 

        if(is_print_imu){
            printf("set is_print_imu = %d\n", is_print_imu);
            data_fusion.PrintImuData(is_print_imu);
        }

        if(is_print_speed){
            printf("set is_print_speed = %d\n", is_print_speed);
            data_fusion.PrintSpeedData(is_print_speed);
        }        
    }
    #else
    {
        #if defined(USE_GLOG)
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
            google::InitGoogleLogging(argv[0]);
        #endif
    }            
    #endif


    // data_fusion.StartDataFusionTask();  
    // 校正
    StructImuParameter imu_parameter_pre, imu_parameter_new;
    
    #if defined(ANDROID)
    {
        double gyro_bias[3];
        data_fusion.GyroParameterCalibration(gyro_bias);
        printf("gyro_bias= %f %f %f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        
        //read_imu_calibation_parameter(&imu_parameter_pre); 
        memcpy(imu_parameter_new.gyro_bias, gyro_bias, sizeof(gyro_bias));
        wite_imu_calibation_parameter(imu_parameter_new);
    }
    #endif

    while(1){
        printf(" loop\n");
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
    sprintf(buffer, "gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0], imu_parameter.gyro_bias[1], imu_parameter.gyro_bias[2] );

    if (file_imu.is_open()) {
        file_imu << buffer; 
        printf("write new gyro_bias %f %f %f\n", imu_parameter.gyro_bias[0], imu_parameter.gyro_bias[1], imu_parameter.gyro_bias[2] );
    }else{
        printf("write new gyro_bias failed!!\n");
    }

    file_imu.close();
    return 1;
}

