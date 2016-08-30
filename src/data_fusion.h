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
public:

    /// @brief construct  obj
    DataFusion();
	
    ~DataFusion() {}

    void Initialize(CameraPara camera_para_t, IPMPara ipm_para_t );
	int data_fusion_main();
		
	int run_fusion( string buffer_log, 	string data_flag);
	int polyfit(std::vector<float>* lane_coeffs, const cv::Mat& xy_feature, int order );

	int GetLanePredictParameter(cv::Mat& lane_coeffs_predict, double image_timestamp, cv::Mat lane_coeffs_pre, double lane_num, double m_order );
	int LanePredict(cv::Mat& lane_coeffs_predict, cv::Mat lane_coeffs_pre, double lane_num, double m_order);

	static void *thread_rounter(void *tmp)//线程执行函数
	{
		DataFusion *p=(DataFusion *)tmp;
		//通过p指针间接访问类的非静态成员
		p->data_fusion_main();
	}

	pthread_t tid;
	int exec_task()
	{
		
		int ERR = pthread_create(&tid,NULL,thread_rounter,(void *)this);//启动线程执行例程
//		return ERR;
	}



private:

	bool isFirsttimeGetParameter; // 是否是外部第一次调用参数预测
	double m_pre_image_stamptime; // 上一帧
	double m_cur_image_stamptime; // 当前外部图像处理模块处理的图片生成的时间戳
	bool m_new_lane_parameter_get; // 是否有新的外部调用
	char m_camera_match_state; // 1: 正常匹配 -1:本地camera时间戳大于外部调用的get时间戳
	
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
		double fai;
	};

	// 读入log
	string buffer_log;
	stringstream ss_log;
	stringstream ss_tmp;
	ifstream infile_log;	   // ofstream
	double log_data[2];

	/// CAN
	CAN_VehicleEstimate can_vehicle_estimate;	
	bool is_steer_angle_OK; // 当前是否steer数据已经有了
	double steer_angle_deg;
	double steer_timestamp; 
	double speed_can;
	double speed_timestamp; 
	double can_timestamp;
	double can_timestamp_pre ;
	bool isFirstTime_can;
	StructVehicleState struct_vehicle_state;
	std::queue<StructVehicleState> queue_vehicle_state;

	// IMU
	ImuAttitudeEstimate imu_attitude_estimate;
	double acc_filt_hz; // 加速度计的低通截止频率
	bool isFirstTime_att; // 是否是第一次进入
	double imu_timestamp;
	double pre_imu_timestamp; // IMU数据上次得到的时刻 
	StructAtt struct_att;	
	std::queue<StructAtt> queue_att;// 定义队列;

	// lane
	CameraPara camera_para;// parameter set up	
	IPMPara ipm_para;// ipm para
	bool isFirstTime_Lane;

	double att_cur[3] ;		
	double vehicle_vel[2];
	double vehicle_pos[2];
	double	vehicle_fai;

	double att_pre[3] ;	
	double vehicle_vel_pre[2];
	double vehicle_pos_pre[2];
	double vehicle_fai_pre ;


};


#endif  // DATA_FUSION_H
