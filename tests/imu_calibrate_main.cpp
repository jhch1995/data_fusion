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
MatrixXd g_acc_calibrate_sequence(6,3);// 校准的acc的哪个面
MatrixXd g_acc_average_save(6,3);// 6个面的均值
//g_acc_average_save.setZero();
MatrixXd g_acc_A0(3, 1), g_acc_A1(3,3);

// 6个面的顺序: 1:-Y 2:-Z 3:+X 4:+Y 5:-X 6:+Z
int g_acc_calibrate_index = 0 ;// 用于表示正在校正哪个面  0表示刚开始

bool g_acc_average_data_ready = false; // 是否收集完数据

//----------------------function-------------------------------//
// 获取当前循环需要读取数据的时间
void GetReadAccDataTime(double* timestamp);

int GetAccAverageData(const int data_num, const int acc_sequence_index, MatrixXd &acc_average_save);

// 继续读取acc数据 用于校正
void ReadAccAverageData( );  

void TryCalGyroParameter(int sig);

void TryPrintImuData(int sig);

void PrintImuData();

// 计算acc校正参数
int CalculateAccParameter(const MatrixXd acc_average_save, MatrixXd &A0, MatrixXd &A1);

int WiteImuCalibationParameter(const StructImuParameter &imu_parameter);

int ReadImuCalibationParameter( StructImuParameter *imu_parameter);

int CalculatGyroParameter(double gyro_bias[3]);

int JudgeImuParameteIstLegal(StructImuParameter new_imu_parameter);

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int ReadImuCalibationParameter( StructImuParameter *imu_parameter)
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

        int len = GET_ARRAY_LEN(gyro_A0);
        memcpy(&(imu_parameter->gyro_A0[0]), gyro_A0, GET_ARRAY_LEN(gyro_A0)); // 为什么有问题
        imu_parameter->gyro_A0[0] = gyro_A0[0];
        imu_parameter->gyro_A0[1] = gyro_A0[1];
        imu_parameter->gyro_A0[2] = gyro_A0[2];
        printf("read imu parameter %s, %f %f %f\n", data_flag.c_str(), imu_parameter->gyro_A0[0], imu_parameter->gyro_A0[1], imu_parameter->gyro_A0[2]);
        fflush(stdout);
    }else{
        printf("open file error!!!\n");
        fflush(stdout);
    }
    file_imu.close();
    return 1;
}

int WiteImuCalibationParameter(const StructImuParameter &imu_parameter)
{
     //文件格式: gyro_A0 gyro_A0[0] gyro_A0[1] gyro_A0[2]
    char buffer[100];
    //clear content
    fstream file_imu;
    file_imu.open(g_file_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
    if (!file_imu.is_open() || file_imu.fail())
    {
        file_imu.close();
        printf("Error : failed to erase file content !");
        fflush(stdout);
    }
    file_imu.close();

    file_imu.open(g_file_addr.c_str());
    if (file_imu.is_open()) {
        sprintf(buffer, "--gyro_bias_x=%f\n--gyro_bias_y=%f\n--gyro_bias_z=%f\n",
                imu_parameter.gyro_A0[0],  imu_parameter.gyro_A0[1],  imu_parameter.gyro_A0[2]);
          sprintf(buffer, "gyro_A0 %f %f %f\n", imu_parameter.gyro_A0[0],  imu_parameter.gyro_A0[1],  imu_parameter.gyro_A0[2]);
        file_imu << buffer;
        printf("write new gyro_A0 %f %f %f\n", imu_parameter.gyro_A0[0], imu_parameter.gyro_A0[1], imu_parameter.gyro_A0[2] );
        fflush(stdout);
    }else{
        printf("write new gyro_A0 failed!!\n");
        fflush(stdout);
    }

    file_imu.close();
    return 1;
}

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
int GetAccAverageData(const int data_num, const int acc_sequence_index, MatrixXd &acc_average_save)
{
    if(data_num <=0 || acc_sequence_index<0 || acc_sequence_index>6)
        return 0;
    double timestamp;
    GetReadAccDataTime(&timestamp);

    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3],  gyro_camera[3];
    bool read_acc_data = true;
    int read_acc_data_index = 0;
    MatrixXd acc_average_tmp_save(3,3); // 每个面都要取三次均值，三次均值一致才能通过校正
    RowVectorXd acc_average_tmp(3);
    RowVectorXd acc_vector_tmp(3), acc_vector_sum_tmp(3);
    acc_vector_sum_tmp.setZero();
    RowVectorXd acc_standard(3); // 当前面下acc标准值
    acc_standard = g_acc_calibrate_sequence.row(acc_sequence_index);
    bool is_first_acc_data = true;
    while(read_acc_data){
        if(is_first_acc_data){
            usleep(10000);
            is_first_acc_data = false;
        }
        
        int search_state = DataFusion::Instance().GetTimestampData(timestamp, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            // 判断数据的有效性
            for(int j = 0; j<3; j++)
                acc_vector_tmp(j) = acc_camera[j]/ONE_G;
            RowVectorXd acc_diff(3);
            acc_diff = acc_vector_tmp - acc_standard;
            double acc_diff_normal = acc_diff.norm();
            // 对平面就行判断
            if(acc_diff_normal < 0.15){
                // 满足条件，则进行累加求和
                acc_vector_sum_tmp += acc_vector_tmp;
            }else{
                // 不满足条件 退出这次采集数据
                read_acc_data_index = 0;
                acc_vector_sum_tmp.setZero();
//                 printf("acc_diff: %.3f\n", acc_diff_normal);
//                 printf("请确认校准模块是否放置平整，并且 %d 号面朝天，然后按 n !!!\n", acc_sequence_index+1);
                printf("Make sure the module is placed level，and place the index %d to sky，then press next!!!\n", acc_sequence_index+1);
                fflush(stdout);
                return 0;
            }
            read_acc_data_index++;
        }else{
            printf("no new acc data!!\n");
            fflush(stdout);
        }
        // 这个面数据读取完毕
        if(read_acc_data_index >= data_num){
            acc_average_save.row(acc_sequence_index) = acc_vector_sum_tmp/read_acc_data_index;
            read_acc_data = false;
//             printf("\nindex: %d , average value: %.3f %.3f %.3f\n", acc_sequence_index+1, acc_average_save(acc_sequence_index, 0), 
//                         acc_average_save(acc_sequence_index, 1), acc_average_save(acc_sequence_index, 2));
//             fflush(stdout);
            if(acc_sequence_index < 5){
//                 printf("请将 %d 号面朝天放平，然后按 n  !!!\n", acc_sequence_index+2);
                printf("Place index %d up to sky，then press next  !!!\n", acc_sequence_index+2);
                fflush(stdout);
            }else{
                printf("try to calculate the acc parameter!!\n");
                fflush(stdout);
                return 2;
            }
            return 1;
        }
        GetReadAccDataTime(&timestamp);
        usleep(20000);
    }
    return 0;    
}

void ReadAccAverageData()
{   
    if(g_acc_calibrate_index < 6 ){
        usleep(100000);
        int get_acc_average_state = GetAccAverageData(50, g_acc_calibrate_index, g_acc_average_save);
        if(get_acc_average_state == 1)
            g_acc_calibrate_index++;
        else if(get_acc_average_state == 2){
            g_acc_average_data_ready = true;
            g_acc_calibrate_index++;
        }
    }else{
        g_is_print_imu_data = !g_is_print_imu_data;
        printf("acc data collect over!!!\n");
        fflush(stdout);
    }
}

void TryCalGyroParameter(int sig)
{   
    double gyro_bias[3];
    CalculatGyroParameter(gyro_bias);
    printf("gyro_bias: %f %f %f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    fflush(stdout);
}

void TryPrintImuData(int sig)
{ 
    g_is_print_imu_data = !g_is_print_imu_data;
}

void PrintImuData()
{
    double timestamp;
    GetReadAccDataTime(&timestamp);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
    if(search_state == 1){
        printf("acc: %0.2f %0.2f %0.2f gyro: %0.2f %0.2f %0.2f\n", acc_camera[0], acc_camera[1], acc_camera[2], gyro_camera[0]*R2D,  gyro_camera[1]*R2D,  gyro_camera[2]*R2D);
        fflush(stdout);
    }
}

// 计算结果
// acc_average_save: 6*3
int CalculateAccParameter(const MatrixXd acc_average_save, MatrixXd &A0, MatrixXd &A1_inv)
{
    MatrixXd Acc(6, 3);
    MatrixXd A1(3,3);
    Acc = acc_average_save;
    A0(0,0) = (Acc(0,0) + Acc(1,0) + Acc(3,0) + Acc(5,0))/4;
    A1(0,0) = (Acc(2,0) - Acc(4,0))/2;
    A1(0,1) = (Acc(3,0) - Acc(0,0))/2;
    A1(0,2) = (Acc(5,0) - Acc(1,0))/2;

    A0(1,0) = (Acc(1,1) + Acc(2,1) + Acc(4,1) + Acc(5,1))/4;
    A1(1,0) = (Acc(2,1) - Acc(4,1))/2;
    A1(1,1) = (Acc(3,1) - Acc(0,1))/2;
    A1(1,2) = (Acc(5,1) - Acc(1,1))/2;

    A0(2,0) = (Acc(0,2) + Acc(2,2) + Acc(3,2) + Acc(4,2))/4;
    A1(2,0) = (Acc(2,2) - Acc(4,2))/2;
    A1(2,1) = (Acc(3,2) - Acc(0,2))/2;
    A1(2,2) = (Acc(5,2) - Acc(1,2))/2;

    A1_inv = A1.inverse();
    return 1;
}


int CalculatGyroParameter(double gyro_bias[3])
{
    // 校正陀螺仪
//     printf("请放置校正模块静止不动，开始校正陀螺仪\n");
    printf("Keep the module still, try to calibrate gyro!\n");
    fflush(stdout);
    DataFusion::Instance().SetGyroAutoCalibrateState(1); // 开启陀螺仪校准

    double gyro_bias_tmp[3];
    int loop_coubter = 0;
    while(1){
        if(1 == DataFusion::Instance().GetGyroCurrentBias(gyro_bias_tmp)){
            memcpy(gyro_bias, gyro_bias_tmp, sizeof(gyro_bias_tmp));
            return 1;                
        }else{
//             printf("请保持校正模块禁止不动，正在校正陀螺仪\n");
            printf("Keep the module still, calibrating gyro now!\n");
            fflush(stdout);
            sleep(2);
            if(loop_coubter++ > 20){
                printf("gyro calibrate failed!!!\n");
                fflush(stdout);
                return -1;
            }
        }
    }
}

// -1: the imu parameter is illegal
// 1: legal
int JudgeImuParameteIstLegal(StructImuParameter new_imu_parameter)
{
    // default imu parameter 
    StructImuParameter imu_parameter_default;
    memset(&imu_parameter_default, 0, sizeof(imu_parameter_default));
    imu_parameter_default.acc_A1[0][0] = 1;
    imu_parameter_default.acc_A1[1][1] = 1;
    imu_parameter_default.acc_A1[2][2] = 1;
    imu_parameter_default.gyro_A1[0][0] = 1;
    imu_parameter_default.gyro_A1[1][1] = 1;
    imu_parameter_default.gyro_A1[2][2] = 1;
    
    double diff_imu_parameter[4] = {0, 0, 0, 0};
    for(int i=0; i<3; i++){
        diff_imu_parameter[0] += abs(new_imu_parameter.gyro_A0[i] - imu_parameter_default.gyro_A0[i]);
    }    
    
    for(int i=0; i<3; i++){
        for(int j=0; i<3; i++){
            diff_imu_parameter[2] += abs(new_imu_parameter.gyro_A1[i][j] - imu_parameter_default.gyro_A1[i][j]);
        }
    }  

    // acc parameter compare
        for(int i=0; i<3; i++){
        diff_imu_parameter[3] += abs(new_imu_parameter.acc_A0[i] - imu_parameter_default.acc_A0[i]);
    }
    for(int i=0; i<3; i++){
        for(int j=0; i<3; i++){
            diff_imu_parameter[2] += abs(new_imu_parameter.acc_A1[i][j] - imu_parameter_default.acc_A1[i][j]);
        }
    } 
    
    if(diff_imu_parameter[0] > 0.1 || diff_imu_parameter[0] > 0.1 || diff_imu_parameter[0] > 0.1 || diff_imu_parameter[0] > 0.1 ){
        printf("imu parameter diff: gyro_A0= %f, gyro_A1= %f, acc_A0=%f, acc_A1=%f !!!\n", diff_imu_parameter[0], diff_imu_parameter[1], diff_imu_parameter[2], diff_imu_parameter[3]);
        // 计算的结果是非法的，误差太大
        fflush(stdout);
        return -1;
    }else{
        return  1;
    }
}

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
    // SIGTSTP:  n    SIGTINI: Ctrl+C
//     (void) signal(SIGTSTP, ReadAccAverageData);  // SIGTSTP: ctrl+Z
    (void) signal(3, TryCalGyroParameter);  
    (void) signal(4, TryPrintImuData);  

    g_acc_calibrate_sequence<<0,-1,0, 0,0,-1, 1,0,0,0,1,0,-1,0,0, 0,0,1;

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.Init();
    data_fusion.SetGyroAutoCalibrateState(0); // 停止自动校准
    data_fusion.ResetImuParameter();
    data_fusion.SetImuVersionMode(2); // 设置摄像头模组为新镜头
    
    if(g_acc_calibrate_index == 0){
//         printf("请将 1 号面朝天放平，然后按 n  !!!\n");
         printf("Place index 1 up to sky，then press next  !!!\n");
         fflush(stdout);
    }

    #if !defined(DATA_FROM_LOG)
        while(1){        
            if(!g_acc_average_data_ready){
                // 采集6个面的数据
                char buf[80];
                scanf("%s", buf);
                if (strcmp(buf, "n") == 0) 
                    ReadAccAverageData();
            }else{
                // 如果acc数据收集完毕，则进行校准参数计算
                int cal_acc_parameter = CalculateAccParameter(g_acc_average_save, g_acc_A0, g_acc_A1);
                g_acc_average_data_ready = false;

                double gyro_bias[3];
                int cal_gyro_parameter = CalculatGyroParameter(gyro_bias);

                if(cal_acc_parameter == 1 && cal_gyro_parameter == 1){
                    printf("try to write imu parameter to camera !!\n");
                    fflush(stdout);
                    StructImuParameter imu_parameter_write;
                    
                    for(int i=0; i<3; i++){
                        imu_parameter_write.gyro_A0[i] = gyro_bias[i];
                        imu_parameter_write.acc_A0[i] = g_acc_A0(i, 0);
                        for(int j=0; j<3; j++){
                            imu_parameter_write.acc_A1[i][j] = g_acc_A1(i, j);
                            imu_parameter_write.gyro_A1[i][j] = 0;
                        }
                    }
                    int is_imu_parameter_legal = JudgeImuParameteIstLegal(imu_parameter_write);
                    if(is_imu_parameter_legal == -1){
                        printf("the caluate imu parameter is illegal !!!\n");
                        printf("make sure the calibrate operation is right, and do it again !!!\n");
                        fflush(stdout);
                        return -1;
                    }

                    //  WR camera register
                    int write_state = camera_write_flash_file(CAMERA_FLASH_FILE_IMU_CAL, &imu_parameter_write, sizeof(imu_parameter_write));
                    if (0 != write_state) {
                        printf("camera_write_flash_file failed %d\n", write_state);
                        fflush(stdout);
                        return write_state;
                    }
                    
                    // read imu parameter test
                    StructImuParameter imu_parameter_read;
                    int read_state = camera_read_flash_file(CAMERA_FLASH_FILE_IMU_CAL, &imu_parameter_read, sizeof(imu_parameter_read));
                    if (0 != read_state) {
                        printf("camera_read_flash_file failed %d\n", read_state);
                        fflush(stdout);
                        return read_state;
                    }else{
                        // 读取摄像头imu参数，确认是否写入正确
                        // gyro parameter compare
                        int cmp_state = memcmp(&imu_parameter_read, &imu_parameter_write, sizeof(imu_parameter_read));
                        if(cmp_state == 0){
                            printf("imu parameter calibrate over, save the parameter success !!!\n");
                            fflush(stdout);
                        }else{
                            printf("write and read imu parameter is not the same , write imu parameter failed !!!\n");
                            fflush(stdout);
                        }
                    }
                    break;
                }
            }
            usleep(5000);
            if(g_is_print_imu_data){
                PrintImuData();
            }
        }
    #endif

    data_fusion.Destory();
    printf("exe over\n");
    fflush(stdout);
    return 0;
}

