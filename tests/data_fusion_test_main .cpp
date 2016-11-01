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
#include "common/base/stdint.h"
#include "common/math/polyfit.h"
#include "common/base/log_level.h"
#include "common/relative_locate/relative_locate.h"
#include "common/relative_locate/bird_perspective_mapping.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"

using namespace std;

DEFINE_string(image_name, "./1.jpg", "image_name");
DEFINE_double(fu, 1482.0, "fu");
DEFINE_double(fv, 1475.874, "fv");
DEFINE_double(cu, 685.044, "cu");
DEFINE_double(cv, 360.02380, "cv");
DEFINE_double(camera_height, 1.25, "camera height mm");    // ??? mm
DEFINE_double(pitch, -0.3, "pitch angle (degree)"); // -1.8
DEFINE_double(yaw, 7.3, "yaw angle (degree)");
DEFINE_int32(image_width, 1280, "image width");
DEFINE_int32(image_height, 720, "image height");
DEFINE_double(x_start_offset, -7.0, "x start offset");
DEFINE_double(x_end_offset, 7.0, "x start offset");
DEFINE_double(y_start_offset, 1.0, "y start offset");
DEFINE_double(y_end_offset, 70.0, "y end offset");
DEFINE_double(x_res, 0.04, "x resolution");
DEFINE_double(y_res, 0.1, "y resolution");


// 读入log和图片路径
string str_image_frame_add = "data/radius/frame/";

ifstream infile_log("data/radius/log.txt");       // 指定log的路径
string buffer_log;
string data_flag;    
stringstream ss_log;
stringstream ss_tmp;


// 进行数据融合的类
DataFusion data_fusion;


int main(int argc, char *argv[])
{   

    #if defined(USE_GLOG)
        #if (!defined(ANDROID))
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
            google::InitGoogleLogging(argv[0]);
        #endif
    #endif  
    

// 初始化融合函数
    data_fusion.StartDataFusionTask();    
    while(1)
    {
      sleep(1);  
    }

  
    return 0;
}


