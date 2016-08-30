
#ifndef IMU_ATTITUDE_ESTIMATE_H
#define IMU_ATTITUDE_ESTIMATE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1600)
#pragma execution_character_set("utf-8")
#endif

#include <vector>
#include "linear_r3.h"

//struct def_attitude_T
//{
//	double mAccAVSFactor[3]; // 加速度计平滑的参数
//	double mFactorAccGyro[3]; // 加速度计修正的姿态的系数
//	double	mAtt[3];
//	double	SmoothAcc[2];
//	double	SmoothAccAngle[3]; // 平滑之后的加速度计角度
//	double	RawAccAngle[3]; // 原始的加速度计角度
//	double  mGyroAngle[3];
//	
//};

class ImuAttitudeEstimate
{
public:

    /// @brief construct  obj
    ImuAttitudeEstimate();
	
    ~ImuAttitudeEstimate() {}

    void Initialize( );

	
	/// @brief 获取摄像头姿态
	///
	/// @param axis
	/// att_new:新的姿态
	/// AccData: 加速度数据
	/// GyroData: 陀螺仪数据
	/// dt: 前后两次量测数据更新的时间差
	void UpdataAttitude( double AccData[3], double GyroData[3], double dt);

	void GetAttitude(double (&att)[3]);

	/// 一阶低通函数
	int LowpassFilter3f(double (&y_new)[3], double y_pre[3], double x_new[3], double dt, double filt_hz);

	int AccDataCalibation(double (&AccData_new)[3], double AccData_raw[3]);

	int SetAccCalibationParam(double A0[3], double A1[3][3]);

	int GyrocDataCalibation(double (&GyroData_new)[3], double GyroData_raw[3]);

	void ResetState();



	enum
	{
	  X_AXIS = 0,
	  Y_AXIS,
	  Z_AXIS
	};

//	def_attitude_T gAttVar;


private:
	bool m_isFirstTimeUpdate; // 判断是否是第一次更新姿态

	double  m_AccAVSFactor[3]; // 加速度计平滑的参数
	double  m_FactorAccGyro[3]; // 加速度计修正的姿态的系数
	double	m_Att[3];
	double	m_SmoothAcc[2];
	double	m_SmoothAccAngle[3]; // 平滑之后的加速度计角度
	double	m_RawAccAngle[3]; // 原始的加速度计角度
	double  m_GyroAngle[3];
	int m_AttInitCounter;// = 20;

	
	// Y1模组的加速度计校正参数
	double	m_accel_range_scale;
	double m_A0[3];// = {0.0628f, 0.0079f, -0.0003f};
	double m_A1[3][3]; // = {0.9986f, -0.0027f, 0.0139f, 0.0164f, 0.9993f, -0.0176f, -0.0159f, 0.0064f, 0.9859f };

	double m_gyro_range_scale;
	double m_GyroDrift[3]; // = {0.0155f, -0.0421f, -0.0217f};  // 陀螺仪零偏，在线估计
	

};


#endif  // IMU_ATTITUDE_ESTIMATE_H
