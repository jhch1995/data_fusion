
#ifndef CAN_VEHICLE_ESTIMATE_H  
#define CAN_VEHICLE_ESTIMATE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1600)
#pragma execution_character_set("utf-8")
#endif

#include <vector>
#include "linear_r3.h"

#include <math.h>

class CAN_VehicleEstimate
{
public:

    /// @brief construct  obj
    CAN_VehicleEstimate();
	
    ~CAN_VehicleEstimate() {}

    void Initialize( );


	void UpdateVehicleState(double steer_angle, double vehicle_speed           , double dt );
		
	// ��ȡ��ǰ������λ���ٶ�
	void GetVelPos(double (&vel)[2], double (&pos)[2]);

	// ����������״̬����
	// ����: �ٶȡ�λ�á������
	void ResetState();


private:
	double m_vehicle_L; // ���������
	
	double m_min_steer_angle; //��С����Ч������ת�ǣ�С������ǶȲ�����
	
	double m_k_steer2wheel_angle; // ������ת�ǵ�����ǰ��ת�ǵ�ϵ��
	
	double m_virtual_front_angle; // ����ǰ�ֵĽǶ�
	
	double m_beta; // �����໬��
	
	double m_fai; // ���������(��Գ����ߣ���ʱ������)
	
	double m_vehicle_vel[2];
	double m_vehicle_pos[2];

};


#endif  // CAN_VEHICLE_ESTIMATE_H