#include "can_vehicle_estimate.h"

using namespace common;

CAN_VehicleEstimate::CAN_VehicleEstimate()
{
    m_vehicle_L = 2.637f;
        
    m_min_steer_angle = 5.0f; //最小的有效方向盘转角，小于这个角度不计算
    m_k_steer2wheel_angle = 0.07f;
    m_virtual_front_angle = 0.0f;
    m_fai = 0.0f;
    
    m_vehicle_vel[0] = 0.0f;
    m_vehicle_vel[1] = 0.0f;

    m_vehicle_pos[0] = 0.0f;
    m_vehicle_pos[1] = 0.0f;
    
}

void CAN_VehicleEstimate::Initialize( )
{

}


void CAN_VehicleEstimate::UpdateVehicleState(double steer_angle, double vehicle_speed           , double dt )
{
    double d_fai = 0.0f;
    if(fabs(steer_angle) > m_min_steer_angle*D2R){
        m_virtual_front_angle = m_k_steer2wheel_angle*steer_angle;
        m_beta = atanf(m_vehicle_L/2.0f*tanf(m_virtual_front_angle)/m_vehicle_L);
        d_fai = vehicle_speed/m_vehicle_L*tanf(m_virtual_front_angle)*cosf(m_beta)*dt;
    }else{
        m_virtual_front_angle = 0.0f;
        d_fai = 0.0f;
        
    }
    m_fai = m_fai + d_fai;
    m_vehicle_vel[0] = vehicle_speed*cosf(m_fai + m_beta);  // X
    m_vehicle_vel[1] = vehicle_speed*sinf(m_fai + m_beta);  // Y

    if(dt < 0.05)
    {
        m_vehicle_pos[0] = m_vehicle_pos[0] + m_vehicle_vel[0]*dt;
        m_vehicle_pos[1] = m_vehicle_pos[1] + m_vehicle_vel[1]*dt;
    }

}


void CAN_VehicleEstimate::UpdateVehicleStateImu(double yaw, double vehicle_speed           , double dt )
{
    m_yaw = yaw;
    m_vehicle_vel[0] = vehicle_speed*cosf(yaw);  // X
    m_vehicle_vel[1] = vehicle_speed*sinf(yaw);  // Y

    if(dt < 1){
        m_vehicle_pos[0] = m_vehicle_pos[0] + m_vehicle_vel[0]*dt;
        m_vehicle_pos[1] = m_vehicle_pos[1] + m_vehicle_vel[1]*dt;
    }
}

void CAN_VehicleEstimate::GetVehicleState(double vel[2], double pos[2], double *yaw)
{
    vel[0] = m_vehicle_vel[0];
    vel[1] = m_vehicle_vel[1];

    pos[0] = m_vehicle_pos[0];
    pos[1] = m_vehicle_pos[1];

    *yaw = m_yaw;

}

void CAN_VehicleEstimate::ResetState()
{
    m_fai = 0.0f;
    m_vehicle_vel[0] = 0.0f;  // X
    m_vehicle_vel[1] = 0.0f;   // Y

    m_vehicle_pos[0] = 0.0f; 
    m_vehicle_pos[1] = 0.0f; 
}



