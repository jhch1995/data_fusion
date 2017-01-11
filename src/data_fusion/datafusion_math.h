#ifndef DATA_FUSION_MATH_H
#define DATA_FUSION_MATH_H

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
    #define VLOG(serverity) std::cout
#endif

// gflog
#include "gflags/gflags.h"
DECLARE_double(gyro_bias_x);
DECLARE_double(gyro_bias_y);
DECLARE_double(gyro_bias_z);

DECLARE_string(imu_init_addr); // imu校正参数文件存储地址
DECLARE_string(imu_parameter_log_addr); // imu每次上电后校准的结果，仅用于分析
DECLARE_string(log_data_addr); // 读取log.txt数据地址
DECLARE_string(jpg_data_addr); // 读取jpg数据文件夹


#endif  // DATA_FUSION_MATH_H
