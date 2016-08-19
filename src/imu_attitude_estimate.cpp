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
  	
	if( StartFlag == 0 )
	{
		StartFlag = 1;
		m_Att[X_AXIS] = AccAngle[X_AXIS];
		m_Att[Y_AXIS] = AccAngle[Y_AXIS];
		m_Att[Z_AXIS] = 0.0f;	  
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



