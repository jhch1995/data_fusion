#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>   

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"

using namespace imu;

int main(int argc, char *argv[])
{   
    // 读入log
    ifstream infile_log("data/radius/log.txt");       // 指定log的路径
    string buffer_log;
    string data_flag;    
    stringstream ss_log;
    stringstream ss_tmp;

    // 写R
    ofstream  offile_log;
    string of_log_addr = "./data/radius/radius.txt"; 
    bool is_save_R = false; //true; // 是否将R保存为txt

    TimeUtils f_time_counter;
    double g_R_cur;
    double image_timestamp;
    
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";  
    #if defined(USE_GLOG)
        //FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
        FLAGS_v = 0;
    #endif

    // 进行数据融合的类
    DataFusion data_fusion;
    data_fusion.StartDataFusionTask();  

    // 清空文件内容
    offile_log.open(of_log_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
    if (!offile_log.is_open() || offile_log.fail())
    {
        offile_log.close();
        printf("Error : failed to erase file content !");
    }
    offile_log.close();
    
    offile_log.open(of_log_addr.c_str());  
    string log_data_t[2];
    while(!infile_log.eof())
    {
        getline(infile_log, buffer_log);
        ss_tmp.clear();
        ss_tmp.str(buffer_log);
        ss_tmp>>log_data_t[0]>>log_data_t[1]>>data_flag;
        ss_log.clear();
        ss_log.str(buffer_log);

        if(data_flag == "cam_frame")
        {
            int camera_raw_timestamp[2];
            string camera_flag, camera_add, image_index_str, image_name;
            ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>image_index_str;
            image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;                     

            // 执行查询转弯半径
            int64_t t_1, t_2;
            int64_t image_timestamp_cur_int = (int64_t)(image_timestamp*1000);
            int r_1 = -1;
            int main_sleep_counter = 0; //  一次外部调用，main sleep的次数
            
            while(r_1<0)
            {
                // 测试运行时间
                t_1 = f_time_counter.Microseconds();
                r_1 = data_fusion.GetTurnRadius( image_timestamp_cur_int, &g_R_cur);        
                t_2 = f_time_counter.Microseconds();

                int64_t R_cal_dt = (t_2 - t_1) ;
                //printf("R_cal_dt(us) = %f\n", R_cal_dt);
                
                if(main_sleep_counter > 0)
                {            
                    printf("main timestamp diamatch conunter:%d, match state= %d, so sleep\n", main_sleep_counter, r_1);
                    sleep(1);
                }
                main_sleep_counter++;       
            }

            if(is_save_R){
                char buffer[100];
                if (offile_log.is_open()) {
                    sprintf(buffer, "%d %d %s %s %f\n", camera_raw_timestamp[0], camera_raw_timestamp[1], camera_add.c_str(), image_index_str.c_str(), g_R_cur);
                    offile_log << buffer; 
                }else{
                    printf("write log file is not opened!");                
                }
            }
            printf("R = %f\n", g_R_cur); 
        } 
        usleep(10000); // 40ms 模拟计算的板子上最快25hz的计算时间， 如果只是为了计算R，可以缩小这个限制
    }

    offile_log.close();
    infile_log.close();
    
    return 0;
}

