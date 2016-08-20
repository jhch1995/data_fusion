#include "imu_attitude_estimate.h"
#include "datafusion_math.h"

#include <stdio.h>

using namespace common;


ImuAttitudeEstimate::ImuAttitudeEstimate()
{
	m_AccAVSFactor[X_AXIS] = 20.0f;
	m_AccAVSFactor[Y_AXIS] = 20.0f;
	m_AccAVSFactor[Z_AXIS] = 20.0f;

	m_FactorAccGyro[X_AXIS] = 0.05f;
	m_FactorAccGyro[Y_AXIS] = 0.05f;
	m_FactorAccGyro[Z_AXIS] = 0.05f;  

	m_accel_range_scale = 8.0f/32768;
	m_gyro_range_scale = 2000.0f/180*M_PI/32768;


	m_A0[0] = 0.0628;
	m_A0[1] = 0.0079;
	m_A0[2] = -0.0003;

	m_A1[0][0] = 0.9986;
	m_A1[0][1] = -0.0027;
	m_A1[0][2] = 0.0139;
	m_A1[1][0] = 0.0164;
	m_A1[1][1] = 0.9993;
	m_A1[1][2] = -0.0176;
	m_A1[2][0] = -0.0159;
	m_A1[2][1] = 0.0064;
	m_A1[2][2] = 0.9859;

	m_GyroDrift[0] = 0.0217;
	m_GyroDrift[1] = 0.0421;
	m_GyroDrift[2] = -0.0155;
	
	
}


void ImuAttitudeEstimate::Initialize( )
{
	
}

void ImuAttitudeEstimate::UpdataAttitude( double AccData[3], double GyroData[3], double dt)
{
	double AccAngle[3];
	double GyroRate[3];
	static unsigned char StartFlag = 0; // first time run
	// ACC to angle
	AccAngle[X_AXIS] = (atan2f(-AccData[Y_AXIS], -AccData[Z_AXIS]));  	 // Calculating pitch ACC angle
	AccAngle[Y_AXIS] = (atan2f(AccData[X_AXIS], sqrtf(AccData[Z_AXIS]*AccData[Z_AXIS] + AccData[Y_AXIS]*AccData[Y_AXIS])));   //Calculating roll ACC angle

	std::cout<<"AccAngle "<<AccAngle[0]*R2D<<"  "<<AccAngle[1]*R2D<<std::endl;
	if( StartFlag == 0 )
	{
		m_AttInitCounter--;
		if(m_AttInitCounter<1)
		{
			StartFlag = 1;
			m_Att[X_AXIS] = AccAngle[X_AXIS];
			m_Att[Y_AXIS] = AccAngle[Y_AXIS];
			m_Att[Z_AXIS] = 0.0;	
		}else{
			m_Att[X_AXIS] = 0.0;
			m_Att[Y_AXIS] = 0.0;
			m_Att[Z_AXIS] = 0.0;	
		}
	}else{ 
		// X
		GyroRate[X_AXIS] =  GyroData[X_AXIS] + sinf(m_Att[X_AXIS])*tanf(m_Att[Y_AXIS])*GyroData[Y_AXIS] + cosf(m_Att[X_AXIS])*tanf(m_Att[Y_AXIS])*GyroData[Z_AXIS];
		m_GyroAngle[X_AXIS] = m_GyroAngle[X_AXIS]+ GyroRate[X_AXIS]* dt;
		m_Att[X_AXIS] = (m_Att[X_AXIS] + GyroRate[X_AXIS]* dt)+ m_FactorAccGyro[X_AXIS]*(AccAngle[X_AXIS] - m_Att[X_AXIS]);
		// Y
		GyroRate[Y_AXIS] = cosf(m_Att[X_AXIS])*GyroData[Y_AXIS] - sinf(m_Att[X_AXIS])*GyroData[Z_AXIS] ;
		m_GyroAngle[Y_AXIS] = m_GyroAngle[Y_AXIS]+ GyroRate[Y_AXIS] * dt;
		m_Att[Y_AXIS] = (m_Att[Y_AXIS] + GyroRate[Y_AXIS]*dt)+ m_FactorAccGyro[Y_AXIS]*(AccAngle[Y_AXIS] - m_Att[Y_AXIS]);	
		// Z
		GyroRate[Z_AXIS] = sinf(m_Att[X_AXIS])/cosf(m_Att[Y_AXIS])*GyroData[Y_AXIS] + cosf(m_Att[X_AXIS])/cosf(m_Att[Y_AXIS])*GyroData[Z_AXIS]; 
		m_GyroAngle[Z_AXIS]  = m_GyroAngle[Z_AXIS]+ GyroRate[Z_AXIS] * dt; 
		m_Att[Z_AXIS] = (m_Att[Z_AXIS] + GyroRate[Z_AXIS]*dt);

	}	

//	att_new[X_AXIS]	= m_Att[X_AXIS];
//	att_new[Y_AXIS] = m_Att[Y_AXIS];
//	att_new[Z_AXIS] = m_Att[Z_AXIS];

//	printf("att index=%d, %6.2f, %6.2f, %6.2f\n", ++m_index_counter, att_new[0]*R2D, att_new[1]*R2D, att_new[2]*R2D);
  
}

void ImuAttitudeEstimate::GetAttitude(double (&att)[3])
{
	att[X_AXIS]	= m_Att[X_AXIS];
	att[Y_AXIS] = m_Att[Y_AXIS];
	att[Z_AXIS] = m_Att[Z_AXIS];
}


int ImuAttitudeEstimate::LowpassFilter3f(double (&y_new)[3], double y_pre[3], double x_new[3], double dt, double filt_hz)
{
	double alpha = 0.0f, rc = 0.0f;

	if(filt_hz <= 0.0f || dt < 0.0f)
	{
		y_new[0] = x_new[0];
		y_new[1] = x_new[1];
		y_new[2] = x_new[2];
		return 0;
	}else{
		rc = 1.0f/(2.0f*M_PI*filt_hz);
        alpha = dt/(dt + rc);
	}

	y_new[0] = y_pre[0] + alpha*(x_new[0] - y_pre[0]);
	y_new[1] = y_pre[1] + alpha*(x_new[1] - y_pre[1]);
	y_new[2] = y_pre[2] + alpha*(x_new[2] - y_pre[2]);

	return 1;	
	
}

int ImuAttitudeEstimate::SetAccCalibationParam(double A0[3], double A1[3][3])
{
	m_A0[0]  = A0[0];
	m_A0[1]  = A0[1];
	m_A0[2]  = A0[2];

	m_A1[0][0] = A1[0][0];
	m_A1[0][1] = A1[0][1];
	m_A1[0][2] = A1[0][2];
	m_A1[1][0] = A1[1][0];
	m_A1[1][1] = A1[1][1];
	m_A1[1][2] = A1[1][2];
	m_A1[2][0] = A1[2][0];
	m_A1[2][1] = A1[2][1];
	m_A1[2][2] = A1[2][2];

	return 1;
	
}


int ImuAttitudeEstimate::AccDataCalibation(double (&AccData_NED)[3], double AccData_raw[3])
{
	double AccData_t[3];
	double AccData_IMU[3];
	AccData_t[0] = AccData_raw[0]*m_accel_range_scale - m_A0[0];
	AccData_t[1] = AccData_raw[1]*m_accel_range_scale - m_A0[1];
	AccData_t[2] = AccData_raw[2]*m_accel_range_scale - m_A0[2];

	AccData_IMU[0]= (m_A1[0][0]*AccData_t[0] + m_A1[0][1]*AccData_t[1] + m_A1[0][2]*AccData_t[2])*ONE_G; // 地理坐标系Z
	AccData_IMU[1]= (m_A1[1][0]*AccData_t[0] + m_A1[1][1]*AccData_t[1] + m_A1[1][2]*AccData_t[2])*ONE_G; // 地理坐标系Y
	AccData_IMU[2]= (m_A1[2][0]*AccData_t[0] + m_A1[2][1]*AccData_t[1] + m_A1[2][2]*AccData_t[2])*ONE_G;  // 地理坐标系X

	// IMU原始坐标系-->大地坐标系(NED)
	AccData_NED[0] = -AccData_IMU[2];
	AccData_NED[1] = -AccData_IMU[1];
	AccData_NED[2] = -AccData_IMU[0];
	
	printf("AccData_raw: %f %f %f\n", AccData_raw[0], AccData_raw[1], AccData_raw[2]);
	printf("AccData_t: %f %f %f\n", AccData_t[0], AccData_t[1], AccData_t[2]);
	printf("AccData_IMU: %f %f %f\n", AccData_IMU[0], AccData_IMU[1], AccData_IMU[2]);

	return 1;

}


int ImuAttitudeEstimate::GyrocDataCalibation(double (&GyroData_NED)[3], double GyroData_raw[3])
{
	double GyroData_IMU[3];
	GyroData_IMU[0] = GyroData_raw[0]*m_gyro_range_scale - m_GyroDrift[0]; // 地理坐标系Z
	GyroData_IMU[1] = GyroData_raw[1]*m_gyro_range_scale - m_GyroDrift[1]; // 地理坐标系Y
	GyroData_IMU[2] = GyroData_raw[2]*m_gyro_range_scale - m_GyroDrift[2]; // 地理坐标系X

	// IMU原始坐标系-->大地坐标系(NED)
	GyroData_NED[0] = -GyroData_IMU[2];
	GyroData_NED[1] = -GyroData_IMU[1];
	GyroData_NED[2] = -GyroData_IMU[0];
	
	return 1;


}



