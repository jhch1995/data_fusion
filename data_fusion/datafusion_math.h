#ifndef DATA_FUSION_MATH_H
#define DATA_FUSION_MATH_H

#include "gflags/gflags.h"

#define R2D 180.0f/M_PI 
#define D2R M_PI/180.0f
#define ONE_G 9.80665f;

#define GET_ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))


// 设置关于VLOG的打印等级
#define VLOG_FATAL 0
#define VLOG_ERROR 1
#define VLOG_WARNING 2
#define VLOG_INFO 3
#define VLOG_DEBUG 4

#if defined(LOG_STD_COUT)
#include <iostream>
//#define LOG(serverity) std::cout
#define VLOG(serverity) std::cout
//#define LOG_IF(serverity, is_true) if(is_true) std::cout
#endif

// gflog
DECLARE_double(gyro_bias_x);
DECLARE_double(gyro_bias_y);
DECLARE_double(gyro_bias_z);

#endif  // DATA_FUSION_MATH_H
