#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <sys/time.h>

#include "opencv2/opencv.hpp"
#include "common/relative_locate/linear_r3.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/concurrency/work_thread.h"
#include "common/concurrency/rwlock.h"
#include "common/hal/android_gsensor.h"
#include "common/hal/halio.h"

#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"
#include "datafusion_math.h"

using namespace std;

class DataFusion
{
private:
#pragma pack(1)    
    struct StructAtt
    {
        double timestamp;
        double att[3];
        double angle_z;
    };

    struct StructVehicleState
    {
        double timestamp;
        double pos[2];
        double vel[2];
        double yaw;
    };

    struct StructTurnRadius
    {
        double timestamp;
        double R;
    };

    struct StructImageFrameInfo
    {
        double timestamp;
        double index;
    };

    struct StructImuData
    {
        double timestamp;
        double acc_raw[3];
        double gyro_raw[3];
        double acc[3];
        double gyro[3];
        double temp;
    };

    struct StructCanSpeedData
    {
        double timestamp;
        double speed;
    };

#pragma pack()

    
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
    void PrintImuData(const int is_print_imu);

     // 配置是否打印speed数据
    void PrintSpeedData(const int is_print_speed);

    // 更新当前读到数据的时间戳
    void UpdateCurrentDataTimestamp( double data_timestample);

    // 判断是否还要继续读取数据
    bool UpdateRreadDataState( );

    // 只保留设定时间长度的数据
    void DeleteOldData( );

    void DeleteOldRadiusData( );

    // 根据时间戳查找对应的数据
    int GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z );

    // 估计汽车的运动状态数据
    void EstimateVehicelState();

    // 估计姿态数据
    void EstimateAtt();

    // 进行数据融合
    void RunFusion( );
    
    int Polyfit(const cv::Mat& xy_feature, int order , std::vector<float>* lane_coeffs);

    float Raw2Degree(short raw);

    // 外部调用接口: 预测特征点的坐标
    int GetPredictFeature( const std::vector<cv::Point2f>& vector_feature_pre ,int64 image_timestamp_pre, int64 image_timestamp_cur, 
                                      std::vector<cv::Point2f>* vector_feature_predict);
    // 计算特征点预测坐标
    int FeaturePredict( const std::vector<cv::Point2f>& vector_feature_pre , double vehicle_pos_pre[2], double att_pre[3], double angle_z_pre, 
                                 double vehicle_pos_cur[2], double att_cur[3], double angle_z_cur, std::vector<cv::Point2f>* vector_feature_predict);

    // 外部调用接口:获取转弯半径
    int GetTurnRadius( const int64 &timestamp_search, double *R);
    
    // 计算测量的转弯半径
    void CalculateVehicleTurnRadius();   
        
    // 数据融合的线程
    static void *ThreadRunFusion(void *p)//线程执行函数
    {
        DataFusion *ptr = (DataFusion *)p;        
        ptr->RunFusion(); //通过p指针间接访问类的非静态成员;

    }
    pthread_t data_fusion_id;
    int ExecTaskRunFusion()
    {        
        int ERR = pthread_create(&data_fusion_id, NULL, ThreadRunFusion, (void *)this); //启动线程执行例程
        return ERR;
    }


private:  
    // 线程
    WorkThread m_fusion_thread;
    bool m_is_running;        

    // 读入log
    string buffer_log;
    stringstream ss_log;
    stringstream ss_tmp;
    ifstream infile_log;       // ofstream

    // read data
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

    int m_is_print_imu_data; // 是否打印IMU数据
    int m_is_print_speed_data;// 是否打印speed数据
    
    // 数据读写锁
    RWLock radius_rw_lock;
    RWLock feature_rw_lock;

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
    
    /// vehicle state   imu+speed运动信息解算
    CAN_VehicleEstimate m_can_vehicle_estimate;
    double m_pre_vehicle_timestamp ;
    StructVehicleState m_struct_vehicle_state;
    std::vector<StructVehicleState> m_vector_vehicle_state;
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
    double m_angle_z_cur, m_angle_z_pre;    

    // 转弯半径 R
    double m_gyro_R_filt_hz; //用于转弯半径计算的陀螺仪的低通截止频率
    double m_can_speed_R_filt_hz; //车速的低通
    StructTurnRadius m_struct_turn_radius;
    std::vector<StructTurnRadius> m_vector_turn_radius;
    

};


#endif  // DATA_FUSION_H
