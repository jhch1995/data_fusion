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
using namespace std; 

void GetTimeS(double* timestamp);

void init();

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
	 //解析
    google::ParseCommandLineFlags(&argc, &argv, true);
	
    // 初始化
    init();
	
	FLAGS_log_dir = "./log/";
//     FLAGS_v = 30;
//     google::SetStderrLogging(0); //设置级别高于 google::INFO 的日志同时输出到屏幕
//     FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
//     FLAGS_logbufsecs = 0;        //缓冲日志输出，默认为30秒，此处改为立即输出
//     google::InstallFailureSignalHandler();      //捕捉 core dumped
	
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.Init();

    usleep(500000);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    int acc_mean_length = 35; // 对加速度计的值进行平滑数据长度
    int acc_data_mean_index = 0;
    double acc_mean_sum[3] = {0, 0 ,0};
	
	// 
	bool is_first_imu_data = true;
	double imu_timestamp_start = 0;
	double g_speed_can = 0;
	
	// 保存数据
	// save results
    string save_result_addr = "./data/imu_trans.ini";
    ofstream fid_save_result_out(save_result_addr.c_str(), ios::out); 
	
	 // 读入log
    string buffer_log;
    stringstream ss_log;
    stringstream ss_tmp;
    ifstream infile_log;
	infile_log.open(FLAGS_log_data_addr.c_str()); // ifstream
    while(infile_log.is_open() && !infile_log.eof() ){
		
		double log_data[2], timestamp_raw[2];
		string data_flag;
		struct StructImuData imu_data;
		getline(infile_log, buffer_log);
		ss_tmp.clear();
		ss_tmp.str(buffer_log);
		ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
		ss_log.clear();
		ss_log.str(buffer_log);
		if(data_flag == "Gsensor"){
			ss_log>>timestamp_raw[0]>>timestamp_raw[1];
			double imu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
			
			int search_state = DataFusion::Instance().GetTimestampData(imu_timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
			
			double g_R = -1;
			int64 time_int64 = (int64)(imu_timestamp*1e3);
			int radius_state = DataFusion::Instance().GetTurnRadius(time_int64, &g_R);
			if(is_first_imu_data){
				imu_timestamp_start = imu_timestamp;
				is_first_imu_data = false;
			}
			
			if(search_state == 1 && radius_state == 1){
				 // save data
                if(fid_save_result_out ){
                    char buffer_save[100];
                    sprintf(buffer_save, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.1f %.1f", imu_timestamp - imu_timestamp_start, acc_camera[0], acc_camera[1], 
							acc_camera[2], gyro_camera[0], gyro_camera[1], gyro_camera[2], g_speed_can, g_R);
                    fid_save_result_out<<buffer_save<<endl;
					
// 					printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.1f  %.1f\n", imu_timestamp - imu_timestamp_start, acc_camera[0], acc_camera[1], 
// 							acc_camera[2], gyro_camera[0], gyro_camera[1], gyro_camera[2],g_speed_can, g_R);
                }
			}
		}else if(data_flag == "speed"){
			string str_t[10],str_speed;
			double raw_timestamp[2];
			double speed_can, speed_timestamp;
			ss_log>>raw_timestamp[0]>>raw_timestamp[1]>>str_t[0]>>g_speed_can;

			g_speed_can = g_speed_can/3.6;// km/h-->m/s
		}
		usleep(10);
	
    }
    
    printf("exe over !\n");
    data_fusion.Destory();
    return 0;
}

void init()
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = 0; //VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

}


// 获取当前循环需要读取数据的时间
void GetTimeS(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}


