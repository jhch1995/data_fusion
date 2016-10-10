#include "data_fusion.h"

//# define LOG(INFO) std::cout


using namespace common;

DataFusion::DataFusion()
{

}


void DataFusion::Initialize(CameraPara camera_para_t, IPMPara ipm_para_t )
{
    infile_log.open("data/log.txt");       // ofstream
    
    /// CAN
    is_steer_angle_OK = 0; // 当前是否steer数据已经有了
    can_timestamp_pre = 0.0f;
    isFirstTime_can = 1;

    // IMU
    acc_filt_hz = 5.0f; // 加速度计的低通截止频率
    isFirstTime_att = 1; // 是否是第一次进入
    imu_timestamp = 0.0f;
    pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻 

    // 外部lane
    att_cur[0] = 0.0;
    att_cur[1] = 0.0;
    att_cur[2] = 0.0;
    vehicle_vel[0] = 0.0;
    vehicle_vel[1] = 0.0;
    vehicle_pos[0] = 0.0;
    vehicle_pos[1] = 0.0;
    vehicle_fai = 0.0;
    vehicle_yaw = 0.0;
    
    att_pre[0] = 0.0;
    att_pre[1] = 0.0;
    att_pre[2] = 0.0;
    vehicle_vel_pre[0] = 0.0;
    vehicle_vel_pre[1] = 0.0;
    vehicle_pos_pre[0] = 0.0;
    vehicle_pos_pre[1] = 0.0;
    vehicle_fai_pre = 0.0;

    isFirsttimeGetParameter = 1;
    m_new_lane_parameter_get = 0;
}


int DataFusion::data_fusion_main()
{
//    while(1)                
//    {    
        while(m_new_lane_parameter_get && m_camera_match_state != -1)
        {
            if(getline(infile_log, buffer_log))
            {
                string data_flag;
                double camera_timestamp_raw[2];
                ss_tmp.clear();
                ss_tmp.str(buffer_log);
                ss_tmp>>camera_timestamp_raw[0]>>camera_timestamp_raw[1]>>data_flag;
                double camera_timestamp;
                // 读取log，度过读到camera数据
                if(data_flag == "cam_frame")
                {
                    camera_timestamp = camera_timestamp_raw[0] + camera_timestamp_raw[1]*1e-6;
                    if( fabs(camera_timestamp - m_extern_call_cur_image_stamptime) < 0.001)
                    {
                        m_camera_match_state = 1; // 时间戳已经匹配
                        m_new_lane_parameter_get = 0;
                        // 更新当前车状态数据
                        imu_attitude_estimate.GetAttitude(att_cur);
                        can_vehicle_estimate.GetVelPosXY(vehicle_vel, vehicle_pos);                        
                        
                    }else if( camera_timestamp - m_extern_call_cur_image_stamptime > 0.1) // 时间戳匹配错误 防止死循环
                    {
                        m_camera_match_state = -1; // 本地时间戳已经超时
                    }
                }else
                {
                    // IMU CAN信息融合
                    run_fusion( buffer_log, data_flag);
                }
            }            
        }
//     }

    return 1;
}


int DataFusion::run_fusion( string buffer_log, string data_flag)
{
    ss_log.clear();
    ss_log.str(buffer_log);

    if(data_flag == "Gsensor")
    {            
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
            struct_att.timestamp = imu_timestamp;
            queue_att.push(struct_att);
        }
        
    }else if(data_flag == "StDir")
    {
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
//        if( is_steer_angle_OK == 1)
//        {
//            is_steer_angle_OK = 0;
//            double speed_raw_timestamp[2];    
//            string speed_str;
//            ss_log>>speed_raw_timestamp[0]>>speed_raw_timestamp[1]>>speed_str>>speed_can;    
//            speed_can = speed_can/3.6;// km/h-->m/s
//            speed_timestamp = speed_raw_timestamp[0] + speed_raw_timestamp[1]*1e-6;
//            can_timestamp = speed_timestamp;

//            if(isFirstTime_can)
//            {
//                can_timestamp_pre = speed_timestamp;                    
//                isFirstTime_can = 0;
//            }else
//            {
//                double dt_can = can_timestamp - can_timestamp_pre;
//                can_vehicle_estimate.UpdateVehicleState(steer_angle_deg*D2R, speed_can, dt_can );
//                can_timestamp_pre = can_timestamp;
//                
//                // save queue
//                struct_vehicle_state.timestamp = can_timestamp;
//                can_vehicle_estimate.GetVelPosFai(struct_vehicle_state.vel, struct_vehicle_state.pos, struct_vehicle_state.fai);

//                //printf("steer: %f speed: %f  dt: %f\n", steer_angle_deg, speed_can, dt_can);
//                //printf("vel: %f, %f pos: %f, %f\n", struct_vehicle_state.vel[0], struct_vehicle_state.vel[1], struct_vehicle_state.pos[0], struct_vehicle_state.pos[1]);
//            }
//        }

        // 利用imu+speed计算汽车运动
        double speed_raw_timestamp[2];    
        string speed_str;
        ss_log>>speed_raw_timestamp[0]>>speed_raw_timestamp[1]>>speed_str>>speed_can;    
        speed_can = speed_can/3.6;// km/h-->m/s
        speed_timestamp = speed_raw_timestamp[0] + speed_raw_timestamp[1]*1e-6;
        can_timestamp = speed_timestamp;
        
        if(is_first_speed_data)
        {
            is_first_speed_data = 0;
            imu_attitude_estimate.GetAttitude(att_xy_pre);
            can_timestamp_pre = speed_timestamp;  
            
        }else
        {
            double dt_can = can_timestamp - can_timestamp_pre;
            imu_attitude_estimate.GetAttitude(att_xy_cur);
            can_vehicle_estimate.UpdateVehicleState_imu(att_xy_cur[2], speed_can, dt_can );
            can_timestamp_pre = can_timestamp;
            
        }
        
    }
    return 1;
}


int DataFusion::polyfit(std::vector<float>* lane_coeffs, const cv::Mat& xy_feature, int order )
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


int DataFusion::GetLanePredictParameter(cv::Mat& lane_coeffs_predict, double image_timestamp, cv::Mat lane_coeffs_pre, double lane_num, double m_order )
{
    // 更新时间戳
    m_extern_call_cur_image_stamptime = image_timestamp;    
    m_new_lane_parameter_get = 1;
    m_camera_match_state = 0; // 重置为初始状态    

    // 根据时间戳，进行查找和计算，直到本地camera时间戳与get的时间戳匹配
    data_fusion_main();
    
    if(isFirsttimeGetParameter)
    {
        std::cout<<"isFirsttimeGetParameter=1"<<endl;
        isFirsttimeGetParameter = 0;
        m_extern_call_pre_image_stamptime = image_timestamp;

        att_pre[0] = att_cur[0];
        att_pre[1] = att_cur[1];
        att_pre[2] = att_cur[2];
        vehicle_pos_pre[0] = vehicle_pos[0];
        vehicle_pos_pre[1] = vehicle_pos[1];
        
        lane_coeffs_pre.copyTo(lane_coeffs_predict);
    }else
    {
        if(m_camera_match_state == 1)
        {            
            LanePredict(lane_coeffs_predict, lane_coeffs_pre, lane_num, m_order);
        }else // if (m_camera_match_state == -1)
        {
            std::cout<<"!!!error camera timestamp dismatch"<<endl;
            return -1;
        }
        
    }
    return 1;    
    
}

// lane_coeffs_pre: 每一列代表一个样本
int DataFusion::LanePredict(cv::Mat& lane_coeffs_predict, cv::Mat lane_coeffs_pre, double lane_num, double m_order)
{
/// init:(待定)
    int lane_points_nums = 5; // 每一条车道线取的样本点数量
    double X[5] = {2.0, 5.0, 10.0, 20.0, 35.0};
    LOG(INFO)<<"lane_coeffs_pre: "<<lane_coeffs_pre<<endl<<endl;


// 计算汽车在两帧之间的状态变化    
    // 对pos进行坐标系转换，转到以pre时刻为初始坐标
    double d_pos_tmp[2]; // 前后两帧在初始坐标系下的汽车运动
    double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
    d_pos_tmp[0] = vehicle_pos[0] - vehicle_pos_pre[0];
    d_pos_tmp[1] = vehicle_pos[1] - vehicle_pos_pre[1];         
    d_pos_new_c[0] = cosf(att_pre[2])*d_pos_tmp[0] + sinf(att_pre[2])*d_pos_tmp[1];
    d_pos_new_c[1] = -sinf(att_pre[2])*d_pos_tmp[0] + cos(att_pre[2])*d_pos_tmp[1];
    
    double dyaw = att_cur[2] - att_pre[2]; 
    double Rn2c_kT[2][2];
    Rn2c_kT[0][0] = cosf(dyaw);
    Rn2c_kT[0][1] = sinf(dyaw);
    Rn2c_kT[1][0] = -Rn2c_kT[0][1];
    Rn2c_kT[1][1] = Rn2c_kT[0][0];
    
    LOG(INFO)<<"dyaw: "<<dyaw<<endl; 
    LOG(INFO)<<"vehicle_pos: "<<vehicle_pos[0]<<" "<<vehicle_pos[1]<<endl;
    LOG(INFO)<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<" "<<vehicle_pos_pre[1]<<endl; 
    LOG(INFO)<<"d_pos_new_c: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1]<<endl; 
    LOG(INFO)<<"vehicle_fai_pre: "<<vehicle_fai_pre;     
    
// 从车道线参数中获取特征点,并预测特征点
    cv::Mat xy_feature_pre = cv::Mat::zeros(2, lane_points_nums, CV_32FC1);; // 上一帧的中车道线的特征点
    cv::Mat xy_feature_predict = cv::Mat::zeros(2, lane_points_nums, CV_32FC1); //  预测的当前帧的中车道线的特征点
    for(int lane_index=0; lane_index<lane_num; lane_index++)
    {        
        for(int points_index = 0; points_index<lane_points_nums; points_index++)
        {
            // Y = aX + b, X = {5, 10, 20, 30, 50}        
            xy_feature_pre.at<float>(0, points_index) = X[points_index];
            xy_feature_pre.at<float>(1, points_index) = lane_coeffs_pre.at<float>(0, lane_index) + lane_coeffs_pre.at<float>(1, lane_index)*X[points_index];

            // NED坐标系下的
            double dx = xy_feature_pre.at<float>(0, points_index) - d_pos_new_c[0];
            double dy = xy_feature_pre.at<float>(1, points_index) - d_pos_new_c[1];
            xy_feature_predict.at<float>(1, points_index) = Rn2c_kT[0][0]*dx + Rn2c_kT[0][1]*dy;
            xy_feature_predict.at<float>(0, points_index) = Rn2c_kT[1][0]*dx + Rn2c_kT[1][1]*dy;
        }

        LOG(INFO)<<"xy_feature_predict: "<<xy_feature_predict<< endl << endl; 
        
        // 车道线拟合    Y = AX(X是纵轴)
        std::vector<float> lane_coeffs_t;
        polyfit(&lane_coeffs_t, xy_feature_predict, m_order );
        
        lane_coeffs_predict.at<float>(0, lane_index) = lane_coeffs_t[0];
        lane_coeffs_predict.at<float>(1, lane_index) = lane_coeffs_t[1];

        LOG(INFO)<<"predict_lane_coeffs: "<<lane_coeffs_t[0]<<" "<<lane_coeffs_t[1]<<endl;     
        
    }

    // 更新pre的值
    att_pre[0] = att_cur[0];
    att_pre[1] = att_cur[1];
    att_pre[2] = att_cur[2];
    vehicle_pos_pre[0] = vehicle_pos[0];
    vehicle_pos_pre[1] = vehicle_pos[1];
    vehicle_fai_pre = vehicle_fai; //struct_vehicle_state.fai;
    vehicle_yaw_pre = vehicle_yaw;

    return 1;
}
