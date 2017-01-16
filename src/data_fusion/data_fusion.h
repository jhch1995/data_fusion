#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <sys/time.h>

#include "gflags/gflags.h"
#include "opencv2/opencv.hpp"
#include "common/relative_locate/linear_r3.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/concurrency/work_thread.h"
#include "common/concurrency/rwlock.h"
#include "common/hal/android_gsensor.h"
#include "common/hal/halio.h"
#include "common/time/time_utils.h"

#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"
#include "datafusion_math.h"

using namespace std;
namespace imu {

class DataFusion : public SingletonBase<DataFusion>
{
public:
    friend class SingletonBase<DataFusion>;
    
//#pragma pack(1)
//    struct StructAtt
//    {
//        double timestamp;
//        double att[3];
//        double angle_z;
//        double att_gyro[3];
//        double acc[3];
//        double gyro[3];
//    };

//    struct StructVehicleState
//    {
//        double timestamp;
//        double pos[2];
//        double vel[2];
//        double yaw;
//    };

//    struct StructTurnRadius
//    {
//        double timestamp;
//        double R;
//        bool is_imu_value_ok;
//    };

//    struct StructImageFrameInfo
//    {
//        double timestamp;
//        double index;
//    };

//    struct StructImuData
//    {
//        double timestamp;
//        double acc_raw[3];
//        double gyro_raw[3];
//        double acc[3];
//        double gyro[3];
//        double temp;
//    };

//    struct StructCanSpeedData
//    {
//        double timestamp;
//        double speed;
//    };

//    struct StructImuParameter
//    {
//       double gyro_bias[3];
//    //   double acc_A0[3];
//    //   double acc_A0[3][3];
//    };
//#pragma pack()


public:
    DataFusion();

    ~DataFusion();

    void Init( );

    // 线程循环函数
    void StartDataFusionTask();

    // 读取数据
    int ReadData();

    // 从log中读取数据
    int ReadDataFromLog( );

    // 在线读取imu数据
    int ReadImuOnline( );

    // 在线读取speed数据
    int ReadSpeedOnline( );

    // 配置是否打印IMU数据
    void PrintImuData(const bool is_print_imu);

     // 配置是否打印speed数据
    void PrintSpeedData(const bool is_print_speed);

    // 更新当前读到数据的时间戳
    void UpdateCurrentDataTimestamp( double data_timestample);

    // 判断是否还要继续读取数据
    bool UpdateRreadDataState( );

    // 只保留设定时间长度的数据
    void DeleteOldData( );

    void DeleteOldRadiusData( );

    // 根据时间戳查找对应的数据
    int GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z, double att_gyro[3], double acc[3], double gyro[3] );

    // 估计汽车的运动状态数据
    void EstimateVehicelState();

    // 估计姿态数据
    void EstimateAtt();

    // 进行数据融合
    void RunFusion( );

    // 线程循环周期控制
    void FusionScheduler(const timeval time_counter_pre, const int64_t period_us);

    int Polyfit(const cv::Mat& xy_feature, int order , std::vector<float>* lane_coeffs);

    double Raw2Degree(short raw);

    // 校准陀螺仪零偏
    int CalibrateGyroBias( double new_gyro_bias[3] );

    int DoCalibrateGyroBiasOnline( );

    int CalibrateGyroBiasOnline(double gyro_bias[3]);

    // 读取imu参数
    int read_imu_calibration_parameter( StructImuParameter *imu_parameter);

    // 写入imu校准结果
    int write_imu_calibration_parameter(const StructImuParameter &imu_parameter);

    // 外部调用接口: 预测特征点的坐标
    int GetPredictFeature( const std::vector<cv::Point2f>& vector_feature_pre ,
            int64_t image_timestamp_pre,
            int64_t image_timestamp_cur,
            std::vector<cv::Point2f>* vector_feature_predict);

    // 计算特征点预测坐标
    int FeaturePredict(const std::vector<cv::Point2f>& vector_feature_pre ,
            double vehicle_pos_pre[2], double att_pre[3], double angle_z_pre,
            double vehicle_pos_cur[2], double att_cur[3], double angle_z_cur,
            std::vector<cv::Point2f>* vector_feature_predict);

    // 外部调用接口:获取转弯半径
    int GetTurnRadius( const int64 &timestamp_search, double *R);

    // 计算测量的转弯半径
    void CalculateVehicleTurnRadius();

private:
    // 线程
    WorkThread m_fusion_thread;
    bool m_is_running;

    // 读入log
    string buffer_log;
    stringstream ss_log;
    stringstream ss_tmp;
    ifstream infile_log;

    // read imu data
    bool m_init_state; // 初始化是否正常
    bool m_is_first_read_gsensor;
    bool m_data_gsensor_update; // 分别对应的数据是否已经更新
    bool m_data_speed_update;
    bool m_data_image_update;
    bool m_is_first_run_read_data;
    StructImageFrameInfo m_image_frame_info;
    StructImuData m_imu_data;
    std::vector<StructImuData> m_vector_imu_data;
    StructCanSpeedData m_can_speed_data;
    std::vector<StructCanSpeedData> m_vector_can_speed_data;

    //murata
    string m_imu_log_flag; // 读取哪个imu的flag

    bool m_is_print_imu_data; // 是否打印IMU数据
    int m_is_print_speed_data;// 是否打印speed数据

    // imu在线校准
    double m_zero_speed_time_counter; // 持续速度为0时间计数
    double m_zero_speed_time_pre; // 上一个speed为0时刻
    bool m_is_first_zero_speed; // 是否是最近时段第一次速度为0
    bool m_is_gyro_online_calibrate_ok; // 在线校准gyro是否OK
    bool m_is_need_reset_calibrate; // 是否需要重置校准
    bool m_is_first_calibrate;
    double m_gyro_sum[3], m_gyro_avg[3], m_gyro_diff[3], m_accel_diff[3], m_last_average[3], m_best_avg[3], m_accel_start[3];
    double m_gyro_diff_norm, m_acc_diff_norm;
    double m_new_gyro_offset[3], m_best_gyro_diff;
    int m_num_converged;
    int m_num_converged_set; // 设定收敛多少次认为稳定
    bool m_converged;
    int m_gyro_sample_num; // the gyro sample numbers each cycle
    int m_sample_counter; // 采样计数

    string m_imu_parameter_addr; // imu参数存放地址
    string m_imu_parameter_log_addr; // 记录每次上电后校准的imu参数结果

    // 数据读写锁
    RWLock radius_rw_lock;
    RWLock feature_rw_lock;
    RWLock get_data_rw_lock; // get_timestamp_data

    // 读取数据控制
    double m_cur_fusion_timestamp; // 当前在进行计算的时间点，
    double m_cur_data_timestamp; // 当前数据的时间戳, 跟外部调用预测的时间戳进行比对
    bool m_is_first_fusion_timestamp; // 第一次fusion更新
    bool m_is_first_data_timestamp; // 第一次read data更新
    double m_data_save_length; // 保存历史数据的长度(时间为单位: s)
    bool m_is_continue_read_data; // 1:继续读取数据  0:暂停读取数据 由data_timestamp控制
    double m_cur_run_timestamp; // 当前系统的运行时间

    // 外部调用的时间
    double m_call_predict_timestamp; // 当前外部图像处理模块处理的图片生成的时间戳
    double m_call_radius_timestamp; // 当前外部调用转弯半径计算的时间戳
    struct timeval time_run_fusion_pre; // for test: 计算run fusion的周期

    /// vehicle state   imu+speed运动信息解算
    CAN_VehicleEstimate m_can_vehicle_estimate;
    double m_pre_vehicle_timestamp ;
    StructVehicleState m_struct_vehicle_state;
    std::vector<StructVehicleState> m_vector_vehicle_state;
//    folly::ProducerConsumerQueue<StructVehicleState> m_queue_vehicle_state(500);
    char m_is_first_speed_data; //  1: 第一次获取到speed数据 0:不是第一次

    // att
    double m_imu_sample_hz; // 配置的IMU数据采样频率
    double m_imu_dt_set; // 设定的IMU更新dt
    ImuAttitudeEstimate m_imu_attitude_estimate;
    double m_acc_filt_hz; // 加速度计的低通截止频率
    double m_gyro_filt_hz; //陀螺仪的低通截止频率
    bool m_isFirstTime_att; // 是否是第一次进入
    double m_pre_imu_timestamp; // IMU数据上次得到的时刻
    double m_pre_att_timestamp; // att上次得到的时刻
    StructAtt m_struct_att;
    std::vector<StructAtt> m_vector_att;
//    folly::ProducerConsumerQueue<StructAtt> m_queue_att(500);
    double m_angle_z_cur, m_angle_z_pre;

    // 转弯半径 R
    bool m_is_first_R_filter;
    double m_gyro_R_filt_hz; //用于转弯半径计算的陀螺仪的低通截止频率
    double m_can_speed_R_filt_hz; //车速的低通
    StructTurnRadius m_struct_turn_radius;
    std::vector<StructTurnRadius> m_vector_turn_radius;
//    folly::ProducerConsumerQueue<StructTurnRadius> m_queue_turn_radius(500);
    bool m_is_R_ok;
};

}

#endif  // DATA_FUSION_H
