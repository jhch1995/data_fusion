#include <iostream>
#include <math.h>
#include <dirent.h>
#include <time.h>   

#include "opencv2/opencv.hpp"
#include "common/base/stdint.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"

using namespace common;

// 进行数据融合的类
DataFusion data_fusion;

int main(int argc, char *argv[])
{      
        

// 初始化融合函数
    data_fusion.StartDataFusionTask();    
    
    return 0;
}


