#ifndef IMU_ATTITUDE_ESTIMATE_H
#define IMU_ATTITUDE_ESTIMATE_H

#include "common/relative_locate/linear_r3.h"
#include "common/base/log_level.h"

#include "datafusion_math.h"


class ImuAttitudeEstimate
{
public:

    /// @brief construct  obj
    ImuAttitudeEstimate();
    
    ~ImuAttitudeEstimate() {}

    void Initialize( );

    /// att_new:新的姿态
    /// acc_data: 加速度数据
    /// gyro_data: 陀螺仪数据
    /// dt: 前后两次量测数据更新的时间差
    void UpdataAttitude( const double acc_data[3], const double gyro_data[3], double dt);

    void GetAttitude(double att[3]);

    void GetAttitudeAngleZ(double att[3], double *angle_z);

    /// 一阶低通函数
    int LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] );

    int AccDataCalibation(const double acc_data_raw[3], double acc_data_ned[3] );

    int SetAccCalibationParam(double A0[3], double A1[3][3]);

    int GyrocDataCalibation(const double gyro_data_raw[3], double gyro_data_new[3] );

    void ResetState();    

    void GetGyroBias( double gyro_bias[3] );
    
    void SetGyroBias( const double gyro_bias[3] );

    void ClearGyroBias();


private:
    enum
    {
      X_AXIS = 0,
      Y_AXIS,
      Z_AXIS
    };
    
    double m_factor_acc_gyro[3]; // 加速度计修正的姿态的系数
    double m_att[3];
    double m_gyro_angle[3];
    double m_angle_z;
    int m_att_init_counter;// = 20;
    
    // Y1模组的加速度计校正参数
    double m_accel_range_scale;
    double m_A0[3];// = {0.0628f, 0.0079f, -0.0003f};
    double m_A1[3][3]; // = {0.9986f, -0.0027f, 0.0139f, 0.0164f, 0.9993f, -0.0176f, -0.0159f, 0.0064f, 0.9859f };
    double m_gyro_range_scale;
    double m_gyro_drift[3]; // = {0.0155f, -0.0421f, -0.0217f};  // 陀螺仪零偏，在线估计    
    double m_gyro_offset[3]; // gyro calibation;
    

};


#endif  // IMU_ATTITUDE_ESTIMATE_H
