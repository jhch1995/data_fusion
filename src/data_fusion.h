#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#if defined(_MSC_VER) && (_MSC_VER >= 1600)
#pragma execution_character_set("utf-8")
#endif

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"

#include "relative_locate.h"
#include "bird_perspective_mapping.h"
#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"
#include "datafusion_math.h"

#include "common/base/stdint.h"
#include "common/math/polyfit.h"
#include "common/base/log_level.h"

#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue> 

using namespace std;

class DataFusion
{
private:
    struct StructAtt
    {
        double timestamp;
        double att[3];
    };

    struct StructVehicleState
    {
        double timestamp;
        double pos[2];
        double vel[2];
        double yaw;
    };
    
public:

    /// @brief construct  obj
    DataFusion();
    
    ~DataFusion() {}

    void Initialize( );

    // TODO:
    int red_data();
    
    int data_fusion_main();
    
    int data_fusion_main_new();

    void update_current_fusion_timestamp( double data_timestample);

    void update_current_data_timestamp( double data_timestample);

    // 判断是否还要继续读取数据
    bool update_read_data_state( );

    // 只保留设定时间长度的数据
    void  delete_history_save_data( );

    // 根据时间戳查找对应的数据
    int get_timestamp_data(double (&vehicle_pos)[2], double (&att)[3], double timestamp_search);
        
    int run_fusion( string buffer_log,     string data_flag);

    int run_fusion_new( );
    
    int polyfit(std::vector<float>* lane_coeffs, const cv::Mat& xy_feature, int order );

    int GetLanePredictParameter(cv::Mat& lane_coeffs_predict, double image_timestamp, cv::Mat lane_coeffs_pre, double lane_num, double m_order );

    int GetLanePredictParameter_new(cv::Mat& lane_coeffs_predict, double image_timestamp_cur, double image_timestamp_pre, 
                                                     cv::Mat lane_coeffs_pre, double lane_num, double m_order );
    
    int LanePredict(cv::Mat& lane_coeffs_predict, cv::Mat lane_coeffs_pre, double lane_num, double m_order);

    int LanePredict_new(cv::Mat& lane_coeffs_predict, cv::Mat lane_coeffs_pre, double lane_num, double m_order, 
                                  StructVehicleState pre_vehicle_state, StructVehicleState cur_vehicle_state, 
                                  StructAtt pre_att_xy, StructAtt cur_att_xy);

    static void *thread_run_fusion(void *tmp)//线程执行函数
    {
        DataFusion *p = (DataFusion *)tmp;
        //通过p指针间接访问类的非静态成员
        p->run_fusion_new();
    }
    pthread_t data_fusion_id;
    int exec_task_run_fusion()
    {        
        int ERR = pthread_create(&data_fusion_id, NULL, thread_run_fusion, (void *)this); //启动线程执行例程
        return ERR;
    }

    // read data
    static void *thread_read_data(void *tmp)//线程执行函数
    {
        DataFusion *p = (DataFusion *)tmp;
        //通过p指针间接访问类的非静态成员
        p->red_data();
    }
    pthread_t read_data_id;
    int exec_task_read_data()
    {        
        int ERR = pthread_create(&read_data_id, NULL, thread_read_data, (void *)this); //启动线程执行例程
        return ERR;
    }



private:

    bool isFirsttimeGetParameter; // 是否是外部第一次调用参数预测
    double m_call_pre_predict_timestamp; // 上一帧
    double m_call_predict_timestamp; // 当前外部图像处理模块处理的图片生成的时间戳
    bool m_new_lane_parameter_get; // 是否有新的外部调用
    char m_camera_match_state; // 1: 正常匹配 -1:本地camera时间戳大于外部调用的get时间戳
    
    // 读入log
    string buffer_log;
    stringstream ss_log;
    stringstream ss_tmp;
    ifstream infile_log;       // ofstream
    
    /// CAN
    CAN_VehicleEstimate can_vehicle_estimate;    
    bool is_steer_angle_OK; // 当前是否steer数据已经有了
    double steer_angle_deg;
    double steer_timestamp; 
    double speed_can;
    double speed_timestamp; 
    double can_timestamp;
    double pre_can_timestamp ;
    bool isFirstTime_can;
    StructVehicleState struct_vehicle_state;
    std::queue<StructVehicleState> queue_vehicle_state;
    std::vector<StructVehicleState> vector_vehicle_state;

    // IMU
    ImuAttitudeEstimate imu_attitude_estimate;
    double acc_filt_hz; // 加速度计的低通截止频率
    bool isFirstTime_att; // 是否是第一次进入
    double imu_timestamp;
    double pre_imu_timestamp; // IMU数据上次得到的时刻 
    double pre_att_timestamp; // att上次得到的时刻 
    StructAtt struct_att;    
    std::queue<StructAtt> queue_att;// 定义队列;
    std::vector<StructAtt> vector_att;

    // lane
    double att_cur[3] ;        
    double vehicle_vel[2];
    double vehicle_pos[2];
    double vehicle_fai;
    double att_pre[3] ;    
    double vehicle_vel_pre[2];
    double vehicle_pos_pre[2];
    double vehicle_fai_pre ;
    
    StructAtt predict_struct_att_pre; 
    StructAtt predict_struct_att_cur; 
    StructVehicleState predict_struct_vehicle_pos_pre;
    StructVehicleState predict_struct_vehicle_pos_cur; 


    // imu+speed运动信息解算
    char is_first_speed_data = 1; //  1: 第一次获取到speed数据 0:不是第一次
    double att_xy_pre[3]; // 上一stamp的角度
    double att_xy_cur[3]; // 当前stamp的角度
    double vehicle_yaw;
    double vehicle_yaw_pre;

    // read data
    bool is_first_read_data = 1; // 是否是第一次进入读取函数，初始化cur_timestamp
    bool is_first_read_gsensor = 1; 
    double cur_read_data_timestamp; // 当前参考的读取数据时间(跟外部调用时传递进来的当前帧时间)
   
    bool data_gsensor_update = 0;
    bool data_speed_update = 0;
    bool data_image_update = 0;

    struct StructImageFrameInfo
    {
        double timestamp;
        double index;
    };
    StructImageFrameInfo image_frame_info;

    struct StructImuData
    {
        double timestamp;
        double acc_raw[3];
        double gyro_raw[3];
        double acc[3];
        double gyro[3];
    };
    StructImuData imu_data;

    struct StructCanSpeedData
    {
        double timestamp;
        double speed;
    };
    StructCanSpeedData can_speed_data;

    // 读取数据控制
    double cur_fusion_timestamp; // 当前在进行计算的时间点，跟外部调用预测的时间戳进行比对
    double cur_data_timestamp; // 当前数据的时间戳
    bool is_first_fusion_timestamp = 1; // 第一次更新
    bool is_first_data_timestamp = 1; // 第一次更新
    double data_save_length = 2; // 保存历史数据的长度(时间为单位: s)
    bool is_continue_read_data = 1; // 1:继续读取数据  2:暂停读取数据 由fusion控制
    bool extern_call_timestamp_update = 0; // 外部调用预测，更新了时间戳


};


#endif  // DATA_FUSION_H
