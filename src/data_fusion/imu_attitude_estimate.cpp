#include "imu_attitude_estimate.h"
#include "datafusion_math.h"
#include <stdio.h>

namespace imu {

ImuAttitudeEstimate::ImuAttitudeEstimate()
{
    Initialize( );    
}


void ImuAttitudeEstimate::Initialize( )
{
    m_accel_range_scale = 8.0f/32768;
    m_gyro_range_scale = 2000.0/180*3.141593/32768;

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

    // 
//    m_gyro_drift[0] = 0;
//    m_gyro_drift[1] = 0;
//    m_gyro_drift[2] = 0;

    // nj 
    m_gyro_drift[0] = 0.00897;
    m_gyro_drift[1] = -0.0322;
    m_gyro_drift[2] = -0.0214;

    // Y-1
//    m_gyro_drift[0] = 0.0155;
//    m_gyro_drift[1] = -0.0421;
//    m_gyro_drift[2] = -0.0217;  

    m_angle_z = 0.0;
    m_att_init_counter = 30;
}

void ImuAttitudeEstimate::UpdataAttitude( const double acc_data[3], const double gyro_data[3], double dt)
{
    double acc_angle[3];
    double gyro_rate[3];
    static unsigned char start_flag = 0; // first time run
    // ACC to angle
    acc_angle[X_AXIS] = (atan2f(-acc_data[Y_AXIS], -acc_data[Z_AXIS]));       // Calculating pitch ACC angle
    acc_angle[Y_AXIS] = (atan2f(acc_data[X_AXIS], sqrtf(acc_data[Z_AXIS]*acc_data[Z_AXIS] + acc_data[Y_AXIS]*acc_data[Y_AXIS])));   //Calculating roll ACC angle

    if( start_flag == 0 ){
        m_att_init_counter--;
        if(m_att_init_counter<1){
            start_flag = 1;
            m_att[X_AXIS] = acc_angle[X_AXIS];
            m_att[Y_AXIS] = acc_angle[Y_AXIS];
            m_att[Z_AXIS] = 0.0;    
        }else{
            m_att[X_AXIS] = 0.0;
            m_att[Y_AXIS] = 0.0;
            m_att[Z_AXIS] = 0.0;    
        }
    }else{ 
        // X
        gyro_rate[X_AXIS] =  gyro_data[X_AXIS] + sinf(m_att[X_AXIS])*tanf(m_att[Y_AXIS])*gyro_data[Y_AXIS] + cosf(m_att[X_AXIS])*tanf(m_att[Y_AXIS])*gyro_data[Z_AXIS];
        m_gyro_angle[X_AXIS] = m_gyro_angle[X_AXIS]+ gyro_rate[X_AXIS]* dt;
        m_att[X_AXIS] = (m_att[X_AXIS] + gyro_rate[X_AXIS]* dt)+ m_factor_acc_gyro[X_AXIS]*(acc_angle[X_AXIS] - m_att[X_AXIS]);
        // Y
        gyro_rate[Y_AXIS] = cosf(m_att[X_AXIS])*gyro_data[Y_AXIS] - sinf(m_att[X_AXIS])*gyro_data[Z_AXIS] ;
        m_gyro_angle[Y_AXIS] = m_gyro_angle[Y_AXIS]+ gyro_rate[Y_AXIS] * dt;
        m_att[Y_AXIS] = (m_att[Y_AXIS] + gyro_rate[Y_AXIS]*dt)+ m_factor_acc_gyro[Y_AXIS]*(acc_angle[Y_AXIS] - m_att[Y_AXIS]);    
        // Z
        gyro_rate[Z_AXIS] = sinf(m_att[X_AXIS])/cosf(m_att[Y_AXIS])*gyro_data[Y_AXIS] + cosf(m_att[X_AXIS])/cosf(m_att[Y_AXIS])*gyro_data[Z_AXIS]; 
        m_gyro_angle[Z_AXIS]  = m_gyro_angle[Z_AXIS]+ gyro_rate[Z_AXIS] * dt; 
        m_att[Z_AXIS] = (m_att[Z_AXIS] + gyro_rate[Z_AXIS]*dt);

        m_angle_z += gyro_data[Z_AXIS] * dt;

//        VLOG(VLOG_DEBUG)<<"IAE:UpdataAttitude--"<<"att[3]: "<<m_att[Z_AXIS]*180/M_PI; 
//        VLOG(VLOG_DEBUG)<<"IAE:UpdataAttitude--"<<"m_angle_z: "<<m_angle_z*180/M_PI; 
//        VLOG(VLOG_DEBUG)<<"IAE:UpdataAttitude--"<<"gyro_z_new: "<<gyro_rate[Z_AXIS]*180/M_PI; 
//        VLOG(VLOG_DEBUG)<<"IAE:UpdataAttitude--"<<"gyro_z_raw: "<<gyro_data[Z_AXIS]*180/M_PI; 
    }    

}

void ImuAttitudeEstimate::GetAttitude(double att[3])
{
    att[X_AXIS] = m_att[X_AXIS];
    att[Y_AXIS] = m_att[Y_AXIS];
    att[Z_AXIS] = m_att[Z_AXIS];
}

void ImuAttitudeEstimate::GetAttitudeAngleZ(double att[3], double *angle_z)
{
    att[X_AXIS] = m_att[X_AXIS];
    att[Y_AXIS] = m_att[Y_AXIS];
    att[Z_AXIS] = m_att[Z_AXIS];

    *angle_z = m_angle_z;
}


void ImuAttitudeEstimate::ResetState()
{
    m_att[Z_AXIS] = 0.0;
}


int ImuAttitudeEstimate::LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] )
{
    double alpha = 0.0f, rc = 0.0f;
    double y_filter[3];

    if(filt_hz <= 0.0f || dt < 0.0f){
        y_new[0] = x_new[0];
        y_new[1] = x_new[1];
        y_new[2] = x_new[2];
        return 0;
    }else{
        rc = 1.0f/(2.0*3.1415*filt_hz);
        alpha = dt/(dt + rc);
    }

    y_filter[0] = y_pre[0] + alpha*(x_new[0] - y_pre[0]);
    y_filter[1] = y_pre[1] + alpha*(x_new[1] - y_pre[1]);
    y_filter[2] = y_pre[2] + alpha*(x_new[2] - y_pre[2]);

    memcpy(y_new, y_filter, sizeof(double)*3);
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


int ImuAttitudeEstimate::AccDataCalibation(const double acc_data_raw[3], double acc_data_ned[3] )
{
    double acc_data_t[3];
    double acc_data_imu[3];
    acc_data_t[0] = acc_data_raw[0]*m_accel_range_scale - m_A0[0];
    acc_data_t[1] = acc_data_raw[1]*m_accel_range_scale - m_A0[1];
    acc_data_t[2] = acc_data_raw[2]*m_accel_range_scale - m_A0[2];

    acc_data_imu[0]= (m_A1[0][0]*acc_data_t[0] + m_A1[0][1]*acc_data_t[1] + m_A1[0][2]*acc_data_t[2])*ONE_G; // 地理坐标系Z
    acc_data_imu[1]= (m_A1[1][0]*acc_data_t[0] + m_A1[1][1]*acc_data_t[1] + m_A1[1][2]*acc_data_t[2])*ONE_G; // 地理坐标系Y
    acc_data_imu[2]= (m_A1[2][0]*acc_data_t[0] + m_A1[2][1]*acc_data_t[1] + m_A1[2][2]*acc_data_t[2])*ONE_G;  // 地理坐标系X

    // IMU原始坐标系-->大地坐标系(NED)
    acc_data_ned[0] = -acc_data_imu[2];
    acc_data_ned[1] = -acc_data_imu[1];
    acc_data_ned[2] = -acc_data_imu[0];    

    return 1;
}

int ImuAttitudeEstimate::GyrocDataCalibation(const double gyro_data_raw[3], double gyro_data_new[3] )
{
    double gyro_data_imu[3];
    gyro_data_imu[0] = gyro_data_raw[0]*m_gyro_range_scale;// - m_gyro_drift[0]; // 地理坐标系Z
    gyro_data_imu[1] = gyro_data_raw[1]*m_gyro_range_scale;// - m_gyro_drift[1]; // 地理坐标系Y
    gyro_data_imu[2] = gyro_data_raw[2]*m_gyro_range_scale;// - m_gyro_drift[2]; // 地理坐标系X

    // IMU原始坐标系-->大地坐标系(NED)
    gyro_data_new[0] = -gyro_data_imu[2] - m_gyro_drift[0];
    gyro_data_new[1] = -gyro_data_imu[1] - m_gyro_drift[1];
    gyro_data_new[2] = -gyro_data_imu[0] - m_gyro_drift[2];    
    
    return 1;
}

}
