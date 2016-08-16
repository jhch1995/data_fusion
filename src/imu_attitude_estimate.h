
#ifndef IMU_ATTITUDE_ESTIMATE_H
#define IMU_ATTITUDE_ESTIMATE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1600)
#pragma execution_character_set("utf-8")
#endif

#include <vector>
#include "linear_r3.h"

struct def_attitude_T
{
	double mAccAVSFactor[3]; // 加速度计平滑的参数
	double mFactorAccGyro[3]; // 加速度计修正的姿态的系数
	double	mAtt[3];
	double	SmoothAcc[2];
	double	SmoothAccAngle[3]; // 平滑之后的加速度计角度
	double	RawAccAngle[3]; // 原始的加速度计角度
	double  mGyroAngle[3];
	
};

class ImuAttitudeEstimate
{
public:

    /// @brief construct  obj
    ImuAttitudeEstimate() {}
	
    ~ImuAttitudeEstimate() {}

    void Initialize( );

	
	/// @brief 获取摄像头姿态
	///
	/// @param axis
	/// att_new:新的姿态
	/// AccData: 加速度数据
	/// GyroData: 陀螺仪数据
	/// dt: 前后两次量测数据更新的时间差
	void Get_Attitude(double (&att_new)[3], double AccData[3], double GyroData[3], double dt);

	/// 一阶低通函数
	int LowpassFilter3f(double (&y_new)[3], double y_pre[3], double x_new[3], double dt, double filt_hz);



	enum
	{
	  X_AXIS = 0,
	  Y_AXIS,
	  Z_AXIS
	};

	def_attitude_T gAttVar;


private:
	int m_index_counter = 0;
	 



};


#endif  // IMU_ATTITUDE_ESTIMATE_H
