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
#include "common/math/polyfit.h"
#include "common/base/log_level.h"
#include "common/relative_locate/relative_locate.h"
#include "common/relative_locate/bird_perspective_mapping.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"

using namespace std;
using namespace imu;


// 读入log和图片路径
ifstream infile_log("data/radius/log.txt");       // 指定log的路径
string buffer_log;
string data_flag;    
stringstream ss_log;
stringstream ss_tmp;

void LoadImage(cv::Mat* image, string image_name);


// 获取指定路径下所有文件
string get_file_name(string file_path);

// 获取图片文件夹中所有的图片的最大最小序号
bool get_max_min_image_index(int &max_index, int &min_index, string file_path);

// 在IPM图上画车道线
void mark_IPM_radius(const IPMPara ipm_para, const double R, const float val,  cv::Mat &ipm_image );

void do_get_turn_radius();


// 进行数据融合的类
DataFusion data_fusion;
TimeUtils f_time_counter;
double g_R_cur;
double image_timestamp;

int main(int argc, char *argv[])
{      
    // 初始化
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";    
  
    // 设置VLOG打印等级
    #if defined(USE_GLOG)
        FLAGS_v = VLOG_DEBUG;
    #endif  
    

// 初始化融合函数
    data_fusion.StartDataFusionTask();    

    // 外部循环控制
    int image_index_start = 1;
    int image_cal_step = 1;// 每隔多少帧计算一次  
    bool is_camera_index_mached = 0; // 是否已经从log中寻找到当前图像的匹配的时间戳

    double log_data_t[2];
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
            double camera_raw_timestamp[2];
            string camera_flag, camera_add, image_index_str;
            string image_name;
            int log_image_index;
            ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>log_image_index;
            image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;                     
            // 执行查询转弯半径
            do_get_turn_radius();

        }   
        usleep(1000);
    }     

    
    return 0;
}


void LoadImage(cv::Mat* image, string image_name)
{
    cv::Mat org_image = cv::imread(image_name, 1);
    cv::Mat channels[3];
    cv::split(org_image, &channels[0]);
    image->create(org_image.rows, org_image.cols, CV_32FC1);
    channels[0].convertTo(*image, CV_32FC1);
    *image = *image * (1.0 / 255);
}


// 获取指定路径下的文件名(主要用于获取图片所在文件夹名字)
string get_file_name(string file_path)
{
    DIR *dp;
    struct dirent *dirp;
    char *file_name_t;
    int n=0;
    const char *filePath = file_path.data();
    if((dp=opendir(filePath))==NULL)
        printf("can't open %s", filePath);
    
    while(((dirp=readdir(dp))!=NULL))
    {
         if((strcmp(dirp->d_name,".")==0)||(strcmp(dirp->d_name,"..")==0))
            continue;
        file_name_t = dirp->d_name;
        printf("%d: %s\n ",++n, file_name_t);
    }

    if(n == 1)
    {
        string str_file_name(file_name_t);
        return str_file_name;
    }else
    {
        printf("error: too many files!!! \n");
        return 0;
    }  
}

// 读取图片文件夹中所有文件最小和最大的index
bool get_max_min_image_index(int &max_index, int &min_index, string file_path)
{
    DIR *dp;
    struct dirent *dirp;
    char *file_name_t;
    bool is_first_time = 1;
    const char *filePath = file_path.data();
    if((dp=opendir(filePath))==NULL)
    {
        printf("can't open %s", filePath); 
        return 0;
    }       

    while(((dirp=readdir(dp))!=NULL))
    {
        if((strcmp(dirp->d_name,".")==0)||(strcmp(dirp->d_name,"..")==0))
           continue;
        
        file_name_t = dirp->d_name;
        string str_name(file_name_t);
        int number = std::atoi( str_name.c_str());
        if(is_first_time)
        {
            is_first_time = 0;
            max_index = number;
            min_index = number;
        }else
        {
            if(number > max_index)
                max_index = number;
            if(number < min_index)
                min_index = number;
        }
    }

    printf("max:%d, min:%d\n", max_index, min_index);
    return 1;    
}


// 图片IPM
void image_IPM(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para)
{
    for (int i = 0; i < ipm_para.height; ++i) 
    {
        int base = i * ipm_para.width;
        for (int j = 0; j < ipm_para.width; ++j) 
        {
            int offset = base + j;
            float ui = ipm_para.uv_grid.at<float>(0, offset);
            float vi = ipm_para.uv_grid.at<float>(1, offset);
            if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] || vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
                continue;
            int x1 = int(ui), x2 = int(ui + 1);
            int y1 = int(vi), y2 = int(vi + 1);
            float x = ui - x1, y = vi - y1;
            float val = org_image.at<float>(y1, x1) * (1 - x) * (1-y) + org_image.at<float>(y1, x2) * x * (1-y) +
                        org_image.at<float>(y2, x1) * (1-x) * y + org_image.at<float>(y2, x2) * x * y;
            ipm_image.at<float>(i, j) = static_cast<float>(val);
        }
    }
}

//// 标记 IPM 转弯半径
void mark_IPM_radius(const IPMPara ipm_para, const double R, const float val,  cv::Mat &ipm_image )
{
    /// 在IPM中标注当前lane
    std::vector<int> x(ipm_para.height+2);
    std::vector<int> y(ipm_para.height+2);         
    double y_max_t, x_t;
    int u, v;

    if(R == 0)
    {
        y_max_t = ipm_para.y_limits[1];  // IPM 横向是x
    }
    else
    {
         y_max_t = min(ipm_para.y_limits[1], fabs(R));
    }
    
    for (double y_t = ipm_para.y_limits[0]; y_t < y_max_t; y_t+=ipm_para.y_scale) // x
    {
        if(R > 0)
        {
            x_t = R - sqrt(R*R - y_t*y_t);
        }
        else if(R < 0)
        {
            x_t = R + sqrt(R*R - y_t*y_t);
        }
        else
        {
            x_t = 0;
        }

        u = (-y_t + ipm_para.y_limits[1])/ipm_para.y_scale;
        v = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;       
        

        if (v < 0 || v > ipm_para.width )
        {
           continue;
        }
        else
        {
           ipm_image.at<float>(u, v) = val;
        }            
    }
}


// 获取转弯半径
void do_get_turn_radius()
{
    int64 t_1, t_2;
    int64 image_timestamp_cur_int = (int64)(image_timestamp*1000);

    int r_1 = -1;
    int main_sleep_counter = 0; //  一次外部调用，main sleep的次数
    while(r_1<0)
    {
        // 测试运行时间
        t_1 = f_time_counter.Microseconds();
        r_1 = data_fusion.GetTurnRadius( image_timestamp_cur_int, &g_R_cur);        
        t_2 = f_time_counter.Microseconds();

        int64 predict_cal_dt = (t_2 - t_1) ;
//        VLOG(VLOG_INFO)<<"DF:main- "<<"predict_cal_dt= "<<predict_cal_dt<<endl; 
        
        if(main_sleep_counter > 0)
        {            
            printf("main timestamp diamatch conunter:%d, match state= %d, so sleep\n", main_sleep_counter, r_1);
            sleep(1);
        }
        main_sleep_counter++;       
    }       
    printf("R = %f\n", g_R_cur);   
       
}

