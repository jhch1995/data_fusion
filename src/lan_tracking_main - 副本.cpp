#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"

#include "relative_locate.h"
#include "bird_perspective_mapping.h"
#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"

// ployfit
#include "common/base/stdint.h"
#include "common/math/polyfit.h"
//#include "gtest/gtest.h"

#include "common/base/log_level.h"

#include "datafusion_math.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue> 

using namespace std;

DEFINE_string(image_name, "./1.jpg", "image_name");
DEFINE_double(fu, 1506.64297, "fu");
DEFINE_double(fv, 1504.18761, "fv");
DEFINE_double(cu, 664.30351, "cu");
DEFINE_double(cv, 340.94998, "cv");
DEFINE_double(camera_height, 1.3, "camera height mm");  // ??? mm
DEFINE_double(pitch, -1.8, "pitch angle (degree)");
DEFINE_double(yaw, 0.0, "yaw angle (degree)");
DEFINE_int32(image_width, 1280, "image width");
DEFINE_int32(image_height, 720, "image height");
DEFINE_double(x_start_offset, -7.0, "x start offset");
DEFINE_double(x_end_offset, 7.0, "x start offset");
DEFINE_double(y_start_offset, 1.0, "y start offset");
DEFINE_double(y_end_offset, 50.0, "y end offset");
DEFINE_double(x_res, 0.05, "x resolution");
DEFINE_double(y_res, 0.1, "y resolution");




void LoadImage(cv::Mat* image, string image_name)
{
    cv::Mat org_image = cv::imread(image_name, 1);
    cv::Mat channels[3];
    cv::split(org_image, &channels[0]);
    image->create(org_image.rows, org_image.cols, CV_32FC1);
    channels[0].convertTo(*image, CV_32FC1);
    *image = *image * (1.0 / 255);
}


int polyfit1(std::vector<float>* lane_coeffs, const cv::Mat& xy_feature, int order )
{  
	int feature_points_num = xy_feature.cols;
	std::vector<float> x(feature_points_num);
	std::vector<float> y(feature_points_num);
	cv::Mat A = cv::Mat(feature_points_num, order + 1, CV_32FC1);
	cv::Mat b = cv::Mat(feature_points_num+1, 1, CV_32FC1);

		 for (int i = 0; i < feature_points_num; i++) 
		{
			x[i] = xy_feature.at<float>(0, i);
			y[i] = xy_feature.at<float>(1, i); 
	
			for (int j = 0; j <= order; j++) 
			{
				A.at<float>(i, j) = pow(y[i], j);
			}
			b.at<float>(i) = x[i];
		}
		
		cv::Mat coeffs;
		int ret = cv::solve(A, b, coeffs, CV_SVD);
		if(ret<=0)
		{	
			printf("cv:solve error!!!\n");
			return -1;
		}
		
		for(int i=0; i<order+1; i++)
		{
			lane_coeffs->push_back(coeffs.at<float>(i,0));
		}	
		return 1;
}

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

/// 读数据
// 读入log
string buffer_log;
string data_flag;	
stringstream ss_log;
stringstream ss_tmp;
ifstream infile_log("data/log.txt");	   // ofstream
double log_data[2];


/// CAN
CAN_VehicleEstimate can_vehicle_estimate;	
bool is_steer_angle_OK = 0; // 当前是否steer数据已经有了
double steer_angle_deg;
double steer_timestamp; 
double speed_can;
double speed_timestamp; 
double can_timestamp;
double can_timestamp_pre = 0.0f;
bool isFirstTime_can = 1;
StructVehicleState struct_vehicle_state;
std::queue<StructVehicleState> queue_vehicle_state;

// IMU
ImuAttitudeEstimate imu_attitude_estimate;
double acc_filt_hz = 5.0f; // 加速度计的低通截止频率
bool isFirstTime_att = 1; // 是否是第一次进入
double imu_timestamp = 0.0f;
double pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻 

StructAtt struct_att;	
std::queue<StructAtt> queue_att;// 定义队列;

int run_fusion( string buffer_log)
{
	ss_tmp.clear();
	ss_tmp.str(buffer_log);
	ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
	ss_log.clear();
	ss_log.str(buffer_log);

	if(data_flag == "Gsensor")
	{	
//		std::cout<<"Gsensor"<<endl;
		
		double AccData_raw[3]; // acc原始坐标系下的
		double AccData_NED[3]; // 大地坐标系
		double AccDataFilter[3]; // 一阶低通之后的数据
		double GyroData_raw[3];
		double GyroData_NED[3];			
		double imu_time_raw[2];
		double imu_temperature;	
		string imu_flag;

        ss_log>>imu_time_raw[0]>>imu_time_raw[1]>>imu_flag>>AccData_raw[0]>>AccData_raw[1]>>AccData_raw[2]
			  >>GyroData_raw[0]>>GyroData_raw[1]>>GyroData_raw[2]>>imu_temperature;
		imu_timestamp = imu_time_raw[0] + imu_time_raw[1]*1e-6;			
		imu_attitude_estimate.AccDataCalibation(AccData_NED, AccData_raw);// 原始数据校正
		imu_attitude_estimate.GyrocDataCalibation(GyroData_NED, GyroData_raw);
		if(isFirstTime_att)
		{
			isFirstTime_att = 0;
			pre_imu_timestamp = imu_timestamp;
			AccDataFilter[0] = AccData_NED[0];
			AccDataFilter[1] = AccData_NED[1];
			AccDataFilter[2] = AccData_NED[2];					
		}else{
			double dt_imu = imu_timestamp - pre_imu_timestamp;					
			imu_attitude_estimate.LowpassFilter3f(AccDataFilter, AccDataFilter, AccData_NED, dt_imu, acc_filt_hz);						
			imu_attitude_estimate.UpdataAttitude(AccDataFilter, GyroData_NED, dt_imu);
			pre_imu_timestamp = imu_timestamp;	

			// save att
			imu_attitude_estimate.GetAttitude(struct_att.att);

//			std::cout<<"att: "<<struct_att.att[2]*R2D<<endl;
			struct_att.timestamp = imu_timestamp;
			queue_att.push(struct_att);
		}
		
	}else if(data_flag == "StDir")
	{
//		std::cout<<"StDir"<<endl;
		int steer_direction;	
		double steer_angle_t;
		double steer_raw_timestamp[2];
		string steer_str[2];			
		ss_log>>steer_raw_timestamp[0]>>steer_raw_timestamp[1]>>steer_str[0]>>steer_direction>>steer_str[0]>>steer_angle_t;				
		steer_timestamp = steer_raw_timestamp[0] + steer_raw_timestamp[1]*1e-6;
		if(steer_direction == 1)
		{
			steer_angle_deg = steer_angle_t; // 1:right +
		}else{
			steer_angle_deg = -steer_angle_t;
		}
		is_steer_angle_OK = 1;		
	}else if(data_flag == "speed")
	{
//		std::cout<<"speed"<<endl;
		
		if( is_steer_angle_OK == 1)
		{
			is_steer_angle_OK = 0;
			double speed_raw_timestamp[2];	
			string speed_str;
			ss_log>>speed_raw_timestamp[0]>>speed_raw_timestamp[1]>>speed_str>>speed_can;	
			speed_can = speed_can/3.6;// km/h-->m/s
			speed_timestamp = speed_raw_timestamp[0] + speed_raw_timestamp[1]*1e-6;
			can_timestamp = speed_timestamp;

			if(isFirstTime_can)
			{
				can_timestamp_pre = speed_timestamp;					
				isFirstTime_can = 0;
			}else{
				double dt_can = can_timestamp - can_timestamp_pre;
				can_vehicle_estimate.UpdateVehicleState(steer_angle_deg*D2R, speed_can, dt_can );
				can_timestamp_pre = can_timestamp;
				
				// save queue
				struct_vehicle_state.timestamp = can_timestamp;
				can_vehicle_estimate.GetVelPosFai(struct_vehicle_state.vel, struct_vehicle_state.pos, struct_vehicle_state.fai);

//				printf("steer: %f speed: %f  dt: %f\n", steer_angle_deg, speed_can, dt_can);
//				printf("vel: %f, %f pos: %f, %f\n", struct_vehicle_state.vel[0], struct_vehicle_state.vel[1], struct_vehicle_state.pos[0], struct_vehicle_state.pos[1]);
			}
		}			
	}
	return 1;
}

int main(int argc, char *argv[])
{
	// 初始化
	google::ParseCommandLineFlags(&argc, &argv, true);
	
	google::InitGoogleLogging(argv[0]);
	FLAGS_log_dir = "./log/";

	
	// parameter set up
	CameraPara camera_para;
	camera_para.fu = FLAGS_fu;
	camera_para.fv = FLAGS_fv;
	camera_para.cu = FLAGS_cu;
	camera_para.cv = FLAGS_cv;
	camera_para.height = FLAGS_camera_height; // m
	camera_para.pitch = FLAGS_pitch * CV_PI / 180;
	camera_para.yaw = FLAGS_yaw * CV_PI / 180;
	camera_para.image_width = FLAGS_image_width;
	camera_para.image_height = FLAGS_image_height;
	BirdPerspectiveMapping bp_mapping(camera_para);

	// ipm para
	IPMPara ipm_para;
	ipm_para.x_limits[0] = FLAGS_x_start_offset;
	ipm_para.x_limits[1] = FLAGS_x_end_offset;
	ipm_para.y_limits[0] = FLAGS_y_start_offset;
	ipm_para.y_limits[1] = FLAGS_y_end_offset;
	ipm_para.x_scale = FLAGS_x_res;
	ipm_para.y_scale = FLAGS_y_res;
	bp_mapping.GetUVLimitsFromXY(&ipm_para);

	/// lane
	int m_order = 1;
	int lane_num = 3;
	int pts_num = 4;	
	//int frame_index = 0; // 图像帧序号
	int lane_index = 0; // 车道线的序号
	double lane_timestamp;
	int uv_feature[2][12];
	string buffer_lane;
    stringstream ss_lane;
    ifstream infile_lane("data/lane_data.ini");       // ofstream
	cv::Mat uv_feature_pts;
	cv::Mat xy_feature; 
	cv::Mat xy_feature_pre = cv::Mat::zeros(2, pts_num, CV_32FC1);; // 上一帧的中车道线的特征点
	cv::Mat xy_feature_predict; 
	cv::Mat lane_coeffs = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);
	cv::Mat lane_coeffs_pre = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);

	// 外部lane
	bool isFirstTime_Lane = 1;
	double att_cur[3] = {0.0f, 0.0f, 0.0f };	
	double att_pre[3] = {0.0f, 0.0f, 0.0f };
	//double vehicle_vel[2];
	double vehicle_pos[2];
	double vehicle_pos_pre[2] = {0.0, 0.0};
	double vehicle_fai_pre = 0.0;

// 外部lane循环控制
	//int start_image_index = 30; // 从哪一帧开始
	int image_cal_step = 10;// 每隔多少帧计算一次车道线预测
	int image_cal_counter = 0;  //计数
	bool is_lane_match_image = 0;
	
    while(getline(infile_log, buffer_log))				
	{	
		// IMU CAN信息融合
		run_fusion( buffer_log );

		ss_tmp.clear();
		ss_tmp.str(buffer_log);
		ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
		ss_log.clear();
		ss_log.str(buffer_log);

		if(data_flag == "cam_frame")
		{
			/// IPM
			double camera_raw_timestamp[2];
			string camera_flag, camera_add, image_index_str;
			string image_name;
			int image_index;
			ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_add>>camera_flag>>image_index;
			image_index = image_index + 1; // image是从0开始计数的

//			std::cout<<"image_index: "<<image_index<<endl;

			if(++image_cal_counter >= image_cal_step)
			{
				printf("image_index: %d\n", image_index);				
				image_cal_counter = 0; // 重置

				// 查找匹配的车道线标注数据
				is_lane_match_image = 0; // 进入查找匹配的lane
				while(!is_lane_match_image)
				{
					getline(infile_lane, buffer_lane);
			        ss_lane.clear();
			        ss_lane.str(buffer_lane);
			        ss_lane>>lane_index>>lane_timestamp
						>>uv_feature[0][0]>>uv_feature[1][0]>>uv_feature[0][1]>>uv_feature[1][1]>>uv_feature[0][2]>>uv_feature[1][2]>>uv_feature[0][3]>>uv_feature[1][3]
						>>uv_feature[0][4]>>uv_feature[1][4]>>uv_feature[0][5]>>uv_feature[1][5]>>uv_feature[0][6]>>uv_feature[1][6]>>uv_feature[0][7]>>uv_feature[1][7]
						>>uv_feature[0][8]>>uv_feature[1][8]>>uv_feature[0][9]>>uv_feature[1][9]>>uv_feature[0][10]>>uv_feature[1][10]>>uv_feature[0][11]>>uv_feature[1][11];
					if(lane_index == image_index)
					{
						is_lane_match_image = 1;
					}else if(lane_index < image_index)
					{
						continue;
					}else{
						printf("error: lane index is bigger than image index!!!\n");
						break;
					}
				}

				stringstream ss;
				ss << image_index;
				ss >> image_index_str;
				image_name = "data/1/" + image_index_str + ".jpg";

				cv::Mat org_image;
				LoadImage(&org_image, image_name);
				cv::Mat ipm_image = cv::Mat::zeros(ipm_para.height+1, ipm_para.width+1, CV_32FC1);
				for (int i = 0; i < ipm_para.height; ++i) 
				{
					int base = i * ipm_para.width;
					for (int j = 0; j < ipm_para.width; ++j) 
					{
						int offset = base + j;
						float ui = ipm_para.uv_grid.at<float>(0, offset);
						float vi = ipm_para.uv_grid.at<float>(1, offset);
						if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] || vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
							continue;
						int x1 = int(ui), x2 = int(ui + 1);
						int y1 = int(vi), y2 = int(vi + 1);
						float x = ui - x1, y = vi - y1;
						float val = org_image.at<float>(y1, x1) * (1 - x) * (1-y) +	org_image.at<float>(y1, x2) * x * (1-y) +
									org_image.at<float>(y2, x1) * (1-x) * y + org_image.at<float>(y2, x2) * x * y;
						ipm_image.at<float>(i, j) = static_cast<float>(val);
					}
				}	

				// 预测车道线
				att_cur[0] = struct_att.att[0];
				att_cur[1] = struct_att.att[1];
				att_cur[2] = struct_att.att[2];
				vehicle_pos[0] = struct_vehicle_state.pos[0];
				vehicle_pos[1] = struct_vehicle_state.pos[1];
				
				if (isFirstTime_Lane)
				{
					isFirstTime_Lane = 0;
					att_pre[0] = att_cur[0];
					att_pre[1] = att_cur[1];
					att_pre[2] = att_cur[2];
					vehicle_pos_pre[0] = vehicle_pos[0];
					vehicle_pos_pre[1] = vehicle_pos[1];
				}
				// 对pos进行坐标系转换，转到以pre时刻为初始坐标
				double d_pos[2]; // 前后两帧在初始坐标系下的汽车运动
				double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
				d_pos[0] = vehicle_pos[0] - vehicle_pos_pre[0];
				d_pos[1] = vehicle_pos[1] - vehicle_pos_pre[1];			
				d_pos_new_c[0] = cosf(vehicle_fai_pre)*d_pos[0] + sinf(vehicle_fai_pre)*d_pos[1];
				d_pos_new_c[1] = -sinf(vehicle_fai_pre)*d_pos[0] + cos(vehicle_fai_pre)*d_pos[1];
				
				double dyaw = att_cur[2] - att_pre[2]; 
				double Rn2c_kT[2][2];
				Rn2c_kT[0][0] = cosf(dyaw);
				Rn2c_kT[0][1] = sinf(dyaw);
				Rn2c_kT[1][0] = -Rn2c_kT[0][1];
				Rn2c_kT[1][1] = Rn2c_kT[0][0];

				LOG(INFO) << "dyaw: "<<dyaw[0]*R2D; 
				LOG(INFO) << "vehicle_pos_pre: "<<vehicle_pos_pre[0]<<vehicle_pos_pre[1];  
				LOG(INFO) << "vehicle_pos: "<<vehicle_pos[0]<<vehicle_pos[1]<<"fai: "<<struct_vehicle_state.fai;  
				LOG(INFO) << "d_pos_new_c: "<<d_pos_new_c[0]<< d_pos_new_c[1];

				// 更新pre的值
				att_pre[0] = att_cur[0];
				att_pre[1] = att_cur[1];
				att_pre[2] = att_cur[2];
				vehicle_pos_pre[0] = vehicle_pos[0];
				vehicle_pos_pre[1] = vehicle_pos[1];
				vehicle_fai_pre = struct_vehicle_state.fai;

				cv::Mat lane_coeffs_predict = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);
				xy_feature_predict = cv::Mat::zeros(m_order+1, pts_num, CV_32FC1);
				//for(int k=0; k<lane_num; k++)
				//{
					// 预测
					for(int i1 = 0; i1<pts_num; i1++)
					{
						// NED坐标系下的
						double dx = xy_feature_pre.at<float>(1, i1) - d_pos_new_c[0];
						double dy = xy_feature_pre.at<float>(0, i1) - d_pos_new_c[1];
						xy_feature_predict.at<float>(1, i1) = Rn2c_kT[0][0]*dx + Rn2c_kT[0][1]*dy;
						xy_feature_predict.at<float>(0, i1) = Rn2c_kT[1][0]*dx + Rn2c_kT[1][1]*dy;
					}				
					// 车道线拟合	Y = AX(X是纵轴)
					std::vector<float> lane_coeffs_t1;
					polyfit1(&lane_coeffs_t1, xy_feature_predict, m_order );
					std::cout<<"pre coeffs: "<<lane_coeffs_t1[0]<<" "<<lane_coeffs_t1[1]<<endl;					

					lane_coeffs_predict.at<float>(0, 0) = lane_coeffs_t1[0];
					lane_coeffs_predict.at<float>(1, 0) = lane_coeffs_t1[1];
				//}

				/// lane
				xy_feature = cv::Mat::zeros(2, pts_num, CV_32FC1);			
				uv_feature_pts = cv::Mat::zeros(2, 4, CV_32FC1);
				lane_coeffs.copyTo(lane_coeffs_pre);
				for(int k=0; k<lane_num; k++)
				{
					for(int i1 = 0; i1<pts_num; i1++)
					{
						uv_feature_pts.at<float>(0, i1) = uv_feature[0][k*pts_num + i1];
						uv_feature_pts.at<float>(1, i1) = uv_feature[1][k*pts_num + i1];
					}		
					// get these points on the ground plane
					bp_mapping.TransformImage2Ground(uv_feature_pts, &xy_feature);	
					// 车道线拟合	Y = AX(X是纵轴)
					std::vector<float> lane_coeffs_t;
					polyfit1(&lane_coeffs_t, xy_feature, m_order );
					
					lane_coeffs.at<float>(0, k) = lane_coeffs_t[0];
					lane_coeffs.at<float>(1, k) = lane_coeffs_t[1];
					// 保存特征点 目前只保存中间车道线
					if( k == 1)
					{
						xy_feature.copyTo(xy_feature_pre);					
						std::cout<<"cur coeffs: "<<lane_coeffs_t[0]<<" "<<lane_coeffs_t[1]<<endl;
					}
					
				}

				/// 在IPM中标注当前lane
				std::vector<int> x(ipm_para.height+2);
			    std::vector<int> y(ipm_para.height+2);
				int i_index = -1;			
				for (float i = ipm_para.y_limits[0]; i < ipm_para.y_limits[1]; i+=ipm_para.y_scale) // x
				{
					i_index += 1;
					y[i_index] = (-i + ipm_para.y_limits[1])/ipm_para.y_scale;
					float x_t = lane_coeffs.at<float>(0, 1) + lane_coeffs.at<float>(1, 1)*i;
					x[i_index] = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;

					if (x[i_index] < 0 || x[i_index] > ipm_para.width )
					{
						continue;
					}else{
						ipm_image.at<float>(y[i_index], x[i_index]) = 0.9;
					}			
				}	

				/// 在IPM中标注上一帧lane
				i_index = -1;			
				for (float i = ipm_para.y_limits[0]; i < ipm_para.y_limits[1]; i+=ipm_para.y_scale) // x
				{
					i_index += 1;
					y[i_index] = (-i + ipm_para.y_limits[1])/ipm_para.y_scale;
					float x_t = lane_coeffs_pre.at<float>(0, 1) + lane_coeffs_pre.at<float>(1, 1)*i;
					x[i_index] = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;

					if (x[i_index] < 0 || x[i_index] > ipm_para.width )
					{
						continue;
					}else{
						ipm_image.at<float>(y[i_index], x[i_index]) = 0.75;
					}			
				}

				/// 在IPM中标注预测lane
				std::vector<int> x_predict(ipm_para.height+2);
			    std::vector<int> y_predict(ipm_para.height+2);
				i_index = -1;			
				for (float i = ipm_para.y_limits[0]; i < ipm_para.y_limits[1]; i+=ipm_para.y_scale) // x
				{
					i_index += 1;
					y_predict[i_index] = (-i + ipm_para.y_limits[1])/ipm_para.y_scale;
					float x_t = lane_coeffs_predict.at<float>(0, 0) + lane_coeffs_predict.at<float>(1, 0)*i;
					x_predict[i_index] = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;

					if (x_predict[i_index] < 0 || x_predict[i_index] > ipm_para.width )
					{
						continue;
					}else{
						ipm_image.at<float>(y_predict[i_index], x_predict[i_index]) = 0.1;
					}			
				}	
				
				cv::imshow("ipm", ipm_image);
				if(cv::waitKey(50))
				{}
				
			}	
		}

    }
	
    return 0;
}



