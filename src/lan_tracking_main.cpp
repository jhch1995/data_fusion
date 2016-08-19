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


#include "datafusion_math.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>


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


int main(int argc, char *argv[])
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	// parameter set up
	CameraPara camera_para;
	camera_para.fu = FLAGS_fu;
	camera_para.fv = FLAGS_fv;

	camera_para.cu = FLAGS_cu;
	camera_para.cv = FLAGS_cv;
	camera_para.height = FLAGS_camera_height; // mm

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


	// IPM transform
//	string image_name;

//	for(int frame_index = 1; frame_index < 170; frame_index++)
//	{	
//		string str_1;
//		stringstream ss;
//		ss << frame_index;
//		ss >> str_1;
//		image_name = "data/1/" + str_1 + ".jpg";

//		cv::Mat org_image;
//		LoadImage(&org_image, image_name);
//		cv::Mat ipm_image = cv::Mat::zeros(ipm_para.height, ipm_para.width, CV_32FC1);
//		for (int i = 0; i < ipm_para.height; ++i) 
//		{
//			int base = i * ipm_para.width;
//			for (int j = 0; j < ipm_para.width; ++j) 
//			{
//				int offset = base + j;
//				float ui = ipm_para.uv_grid.at<float>(0, offset);
//				float vi = ipm_para.uv_grid.at<float>(1, offset);
//				if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] || vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
//					continue;
//				int x1 = int(ui), x2 = int(ui + 1);
//				int y1 = int(vi), y2 = int(vi + 1);
//				float x = ui - x1, y = vi - y1;
//				float val = org_image.at<float>(y1, x1) * (1 - x) * (1-y) +	org_image.at<float>(y1, x2) * x * (1-y) +
//							org_image.at<float>(y2, x1) * (1-x) * y + org_image.at<float>(y2, x2) * x * y;
//				ipm_image.at<float>(i, j) = static_cast<float>(val);

//			}
//		}

///// lane
	int frame_index = 0;
	double lane_time_data[2];
	double lane_timestamp;
	int uv_feature[2][12];
	// 读取lane 标注的结果
	string buffer_lane;
    stringstream ss_lane;
    ifstream infile_lane("data/lane_data.ini");       // ofstream

	cv::Mat uv_feature_pts;
	cv::Mat xy_feature;  


	
    while(getline(infile_lane, buffer_lane))
    {

	/// IPM
		string image_name;
		frame_index++;
		string str_1;
		stringstream ss;
		ss << frame_index;
		ss >> str_1;
		image_name = "data/1/" + str_1 + ".jpg";

		cv::Mat org_image;
		LoadImage(&org_image, image_name);
		cv::Mat ipm_image = cv::Mat::zeros(ipm_para.height, ipm_para.width, CV_32FC1);
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
	
    	
	/// lane
        ss_lane.clear();
        ss_lane.str(buffer_lane);
        ss_lane>>lane_time_data[0]>>lane_time_data[1]
			>>uv_feature[0][0]>>uv_feature[1][0]>>uv_feature[0][1]>>uv_feature[1][1]>>uv_feature[0][2]>>uv_feature[1][2]>>uv_feature[0][3]>>uv_feature[1][3]
			>>uv_feature[0][4]>>uv_feature[1][4]>>uv_feature[0][5]>>uv_feature[1][5]>>uv_feature[0][6]>>uv_feature[1][6]>>uv_feature[0][7]>>uv_feature[1][7]
			>>uv_feature[0][8]>>uv_feature[1][8]>>uv_feature[0][9]>>uv_feature[1][9]>>uv_feature[0][10]>>uv_feature[1][10]>>uv_feature[0][11]>>uv_feature[1][11];

		lane_timestamp = lane_time_data[0] + lane_time_data[1]*1e-6;

		int m_order = 1;
		int lane_num = 3;
		int pts_num = 4;		

		xy_feature = cv::Mat::zeros(m_order+1, pts_num, CV_32FC1);
		uv_feature_pts = cv::Mat::zeros(2, 4, CV_32FC1);
		for(int k=0; k<lane_num; k++)
		{
			for(int i1 = 0; i1<pts_num; i1++)
			{
				uv_feature_pts.at<float>(0, i1) = uv_feature[0][k*pts_num + i1];
				uv_feature_pts.at<float>(1, i1) = uv_feature[1][k*pts_num + i1];
			}		
			// get these points on the ground plane
			bp_mapping.TransformImage2Ground(uv_feature_pts, &xy_feature);		

			// 车道线拟合	
			std::vector<float> lane_coeffs;
			polyfit1(&lane_coeffs, xy_feature, m_order );
			std::cout<<lane_coeffs[0]<<" "<<lane_coeffs[1]<<endl;
		}
	  	
		cv::imshow("ipm", ipm_image);
		if(cv::waitKey(50))
			continue;
	}


	/// IMU
	ImuAttitudeEstimate imu_attitude_estimate;
	imu_attitude_estimate.Initialize();

	bool isFirstTime_att = 1; // 是否是第一次进入
	double imu_timestamp = 0.0f;
	double pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻 183
	double att_new[3] = {0.0f, 0.0f, 0.0f };
	double AccData[3] = {1.0f, 0.0f, 8.8f };
	double AccDataFilter[3] = {1.0f, 0.0f, 8.8f}; // 一阶低通之后的数据
	double acc_filt_hz = 5.0f; // 加速度计的低通截止频率
	
	double GyroData[3] = {0.0f, 0.0f, 0.0f};
	double GyroDrift[3] = {0.0155f, -0.0421f, -0.0217f};  // 陀螺仪零偏，在线估计
	double dt = 0.0f;

	double imu_data[9];
	double accel_range_scale = 8.0f/32768;
	double gyro_range_scale = 2000.0f/180*M_PI/32768;

	// Y1模组的加速度计校正参数
	VectorR3 A0;
	A0.Set(0.0628f, 0.0079f, -0.0003f);
	VectorR3 A1_inv_0 ; // inv(A)的第一行
	A1_inv_0.Set(0.9986f, -0.0027f, 0.0139f); 	
	VectorR3 A1_inv_1;
	A1_inv_1.Set(0.0164f, 0.9993f, -0.0176f);
	VectorR3 A1_inv_2;
	A1_inv_2.Set(-0.0159f, 0.0064f, 0.9859f );

	VectorR3 AccRawDataR3;
	VectorR3 GyroRawDataR3;
	VectorR3 AccRawDataR3_tmp;

	// 读取IMU txt数据
	string buffer_imu;
    stringstream ss_imu;
    ifstream infile_imu("data/log-gsensor.ini");       // ofstream
    while(getline(infile_imu, buffer_imu))
    {
        ss_imu.clear();
        ss_imu.str(buffer_imu);
        ss_imu>>imu_data[0]>>imu_data[1]>>imu_data[2]>>imu_data[3]>>imu_data[4]>>imu_data[5]>>imu_data[6]>>imu_data[7]>>imu_data[8];

		imu_timestamp = imu_data[0] + imu_data[1]*1e-6;

		AccData[0] = imu_data[2]*accel_range_scale;
		AccData[1] = imu_data[3]*accel_range_scale;
		AccData[2] = imu_data[4]*accel_range_scale;
		AccRawDataR3.Set(AccData[0], AccData[1], AccData[2]);
		
		AccRawDataR3_tmp = AccRawDataR3 - A0;
		AccData[2]= -(A1_inv_0.x*AccRawDataR3_tmp.x + A1_inv_0.y*AccRawDataR3_tmp.y + A1_inv_0.z*AccRawDataR3_tmp.z)*ONE_G; // 地理坐标系Z
		AccData[1]= -(A1_inv_1.x*AccRawDataR3_tmp.x + A1_inv_1.y*AccRawDataR3_tmp.y + A1_inv_1.z*AccRawDataR3_tmp.z)*ONE_G; // 地理坐标系Y
		AccData[0]= -(A1_inv_2.x*AccRawDataR3_tmp.x + A1_inv_2.y*AccRawDataR3_tmp.y + A1_inv_2.z*AccRawDataR3_tmp.z)*ONE_G;  // 地理坐标系X
		
		GyroData[2] = -imu_data[5]*gyro_range_scale - GyroDrift[2]; // 地理坐标系Z
		GyroData[1] = -imu_data[6]*gyro_range_scale - GyroDrift[1]; // 地理坐标系Y
		GyroData[0] = -imu_data[7]*gyro_range_scale - GyroDrift[0]; // 地理坐标系X

		if(isFirstTime_att)
		{
			pre_imu_timestamp = imu_timestamp;
			AccDataFilter[0] = AccData[0];
			AccDataFilter[1] = AccData[1];
			AccDataFilter[2] = AccData[2];
			
			isFirstTime_att = 0;
		}else{
			dt = imu_timestamp - pre_imu_timestamp;
			pre_imu_timestamp = imu_timestamp;
			imu_attitude_estimate.LowpassFilter3f(AccDataFilter, AccDataFilter, AccData, dt, acc_filt_hz);
			imu_attitude_estimate.UpdataAttitude(AccDataFilter, GyroData, dt);
			imu_attitude_estimate.GetAttitude(att_new);
		}
    }

 /// CAN计算车信息
	double can_data[3];
	double can_timestamp;
	double pre_can_timestamp = 0.0f;
	double dt_can = 0.0f;
	double vehicle_speed;
	double steer_angle_deg;

	bool isFirstTime_can = 1;
	double vehicle_vel[2];
	double vehicle_pos[2];
	CAN_VehicleEstimate can_vehicle_estimate;
	// 读取CAN txt数据
	string buffer_can;
    stringstream ss_can;
	ifstream infile_can("data/can_data.ini");       // ofstream
    while(getline(infile_can, buffer_can))
    {
		ss_can.clear();
		ss_can.str(buffer_can);
		ss_can>>can_data[0]>>can_data[1]>>can_data[2];

		can_timestamp = can_data[0];
		vehicle_speed = can_data[1];
		steer_angle_deg = can_data[2];

		if(isFirstTime_can)
		{
			pre_can_timestamp = can_timestamp;
			
			isFirstTime_can = 0;
		}else{
			dt_can = can_timestamp - pre_can_timestamp;
			pre_can_timestamp = can_timestamp;
			can_vehicle_estimate.UpdateVehicleState(steer_angle_deg*R2D, vehicle_speed, dt_can );
			can_vehicle_estimate.GetVelPos(vehicle_vel, vehicle_pos);
		}
    }
   

    return 0;
}



