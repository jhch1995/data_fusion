#ifndef TURNLAMPDETECTOR_H
#define TURNLAMPDETECTOR_H

#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <sys/time.h>


#include "common/concurrency/work_thread.h"
#include "common/concurrency/rwlock.h"
#include "common/hal/halio.h"

#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"
#include "datafusion_math.h"

namespace imu {
class TurnlampDetector
{
    public:
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

        TurnlampDetector();
        virtual ~TurnlampDetector();

        void Init( );

        int ReadFmuDataFromLog( );

        // 两个IMU相互之间坐标转换矩阵
        void CalculateRotationFmu2Camera(const double acc_fmu[3], const double acc_camera[3]);

        // array_new = R*array_origin;
        void Array3Rotation(const double R[3][3], const double array_origin[3], double array_new[3]);

        // 仅利用拨杆上ＩＭＵ检测拨杆是否可能被拨动
        // 1: 可能被拨动
        // 0: 没拨动
        int DetectRodShift();

        void StartTurnlampDetectTaskOnline();

        // 自身检测拨杆是否可能是被拨动了
        void DetectSelfRodShiftOnline();

        // 读取log日志，进行检测
        void RunDetectTurnlampOffline();

        int ReadRodDataOnline( );

        int m_rod_shift_state; // 拨杆可能被拨动的标志位

    public:
        // 线程
        WorkThread m_turnlamp_detect_thread;
        bool m_is_turnlamp_detect_running;

        // 读入log
        string buffer_log;
        stringstream ss_log;
        stringstream ss_tmp;
        ifstream infile_log;

        ImuAttitudeEstimate m_fmu_attitude_estimate;
        bool m_is_first_read_fmu;
        double m_fmu_sample_hz;  // fmu采样频率
        double m_R_fmu2camera[3][3];
        StructImuData m_fmu_imu_data;
        StructImuData m_fmu_imu_data_pre;
        int m_turnlamp_state; //  转向灯信号 -1: right 1: left
        int m_turnlamp_state_pre; // 上一时刻的数据 转向灯信号 -1: right 1: left
        bool m_turnlamp_state_change_CAN; // CAN 数据上显示turnlamp状态变化


        bool m_is_first_detect_rod_shift; // 第一次进入拨杆拨动判断
        double m_d_acc_threshold[3]; // 检测拨杆拨动的acc前后变化阈值
        double m_d_gyro_threshold[3]; // 检测拨杆拨动的gyro前后变化阈值
};


}

#endif // TURNLAMPDETECTOR_H

