#include "imu_attitude_estimate.h"
#include <stdio.h>

#define R2D 180.0f/M_PI 
using namespace common;

void ImuAttitudeEstimate::Initialize( )
{
	gAttVar.mAccAVSFactor[X_AXIS] = 20.0f;
	gAttVar.mAccAVSFactor[Y_AXIS] = 20.0f;
	gAttVar.mAccAVSFactor[Z_AXIS] = 20.0f;

	gAttVar.mFactorAccGyro[X_AXIS] = 0.05f;
	gAttVar.mFactorAccGyro[Y_AXIS] = 0.05f;
	gAttVar.mFactorAccGyro[Z_AXIS] = 0.05f;  
}

void ImuAttitudeEstimate::Get_Attitude(double (&att_new)[3], double AccData[3], double GyroData[3], double dt)
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
		gAttVar.mAtt[X_AXIS] = AccAngle[X_AXIS];
		gAttVar.mAtt[Y_AXIS] = AccAngle[Y_AXIS];
		gAttVar.mAtt[Z_AXIS] = 0.0f;	  
	}else{ 
		// X
		GyroRate[X_AXIS] =  GyroData[X_AXIS] + sinf(gAttVar.mAtt[X_AXIS])*tanf(gAttVar.mAtt[Y_AXIS])*GyroData[Y_AXIS] + cosf(gAttVar.mAtt[X_AXIS])*tanf(gAttVar.mAtt[Y_AXIS])*GyroData[Z_AXIS];
		gAttVar.mGyroAngle[X_AXIS] = gAttVar.mGyroAngle[X_AXIS]+ GyroRate[X_AXIS]* dt;
		gAttVar.mAtt[X_AXIS] = (gAttVar.mAtt[X_AXIS] + GyroRate[X_AXIS]* dt)+ gAttVar.mFactorAccGyro[X_AXIS]*(AccAngle[X_AXIS] - gAttVar.mAtt[X_AXIS]);
		// Y
		GyroRate[Y_AXIS] = cosf(gAttVar.mAtt[X_AXIS])*GyroData[Y_AXIS] - sinf(gAttVar.mAtt[X_AXIS])*GyroData[Z_AXIS] ;
		gAttVar.mGyroAngle[Y_AXIS] = gAttVar.mGyroAngle[Y_AXIS]+ GyroRate[Y_AXIS] * dt;
		gAttVar.mAtt[Y_AXIS] = (gAttVar.mAtt[Y_AXIS] + GyroRate[Y_AXIS]*dt)+ gAttVar.mFactorAccGyro[Y_AXIS]*(AccAngle[Y_AXIS] - gAttVar.mAtt[Y_AXIS]);	
		// Z
		GyroRate[Z_AXIS] = sinf(gAttVar.mAtt[X_AXIS])/cosf(gAttVar.mAtt[Y_AXIS])*GyroData[Y_AXIS] + cosf(gAttVar.mAtt[X_AXIS])/cosf(gAttVar.mAtt[Y_AXIS])*GyroData[Z_AXIS]; 
		gAttVar.mGyroAngle[Z_AXIS]  = gAttVar.mGyroAngle[Z_AXIS]+ GyroRate[Z_AXIS] * dt; 
		gAttVar.mAtt[Z_AXIS] = (gAttVar.mAtt[Z_AXIS] + GyroRate[Z_AXIS]*dt);

	}	

	att_new[X_AXIS]	= gAttVar.mAtt[X_AXIS];
	att_new[Y_AXIS] = gAttVar.mAtt[Y_AXIS];
	att_new[Z_AXIS] = gAttVar.mAtt[Z_AXIS];

//	printf("att index=%d, %6.2f, %6.2f, %6.2f\n", ++m_index_counter, att_new[0]*R2D, att_new[1]*R2D, att_new[2]*R2D);
  
  
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



