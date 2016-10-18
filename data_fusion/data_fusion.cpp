#include "data_fusion.h"
#include <unistd.h>

# define DATA_FROM_LOG 1  // 从log中读取数据
# define DATA_FROM_ONLINE 0 //在线读取数据

using namespace common;
DataFusion::DataFusion()
{
}

void DataFusion::Initialize( )
{
    infile_log.open("data/doing/log.txt");       // ofstream    
    m_pre_can_timestamp = 0.0f; /// CAN;

    // IMU
    m_acc_filt_hz = 5.0f; // 加速度计的低通截止频率
    m_gyro_filt_hz = 20.0;
    m_isFirstTime_att = 1; // 是否是第一次进入
    m_pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻     
    m_is_first_speed_data = 1; //  1: 第一次获取到speed数据 0:不是第一次
    
    // read data
    m_is_first_read_gsensor = 1; 
    m_data_gsensor_update = 0;
    m_data_speed_update = 0;
    m_data_image_update = 0;
    
    // 读取数据控制
    m_is_first_fusion_timestamp = 1; // 第一次更新
    m_is_first_data_timestamp = 1; // 第一次更新
    m_data_save_length = 2; // 保存历史数据的长度(时间为单位: s)
    m_is_continue_read_data = 1; // 1:继续读取数据  2:暂停读取数据 由fusion控制   

}

// 读取数据,两种方式:
//      1.离线从log 
// TODO: 2.在线
int DataFusion::ReadData( )
{    
    double log_data[2], timestamp_raw[2];
    string data_flag;
    while(1)
    {  
        //更新is_continue_read_data
        UpdateRreadDataState();
        if(DATA_FROM_LOG && m_is_continue_read_data)
        {
            getline(infile_log, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
            ss_log.clear();
            ss_log.str(buffer_log);

            if(data_flag == "cam_frame")
            {
                string camera_flag, camera_add, image_index_str;
                string image_name;
                int log_image_index;
                ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>camera_flag>>camera_add>>log_image_index;
                
                m_image_frame_info.timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
                m_image_frame_info.index = log_image_index;
                m_data_image_update = 1;
                
            }else if(data_flag == "Gsensor")
            {            
                double AccData_raw[3]; // acc原始坐标系下的
                double AccData_NED[3]; // 大地坐标系
                static double AccDataFilter[3]; // 一阶低通之后的数据
                double GyroData_raw[3];
                double GyroData_NED[3];  
                static double GyroDataFilter[3]; // 一阶低通之后的数据
                double imu_temperature, imu_timestamp;    
                string imu_flag;

                ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>imu_flag>>AccData_raw[0]>>AccData_raw[1]>>AccData_raw[2]
                        >>GyroData_raw[0]>>GyroData_raw[1]>>GyroData_raw[2]>>imu_temperature;
                imu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;                 
                m_imu_attitude_estimate.AccDataCalibation(AccData_raw, AccData_NED);// 原始数据校正
                m_imu_attitude_estimate.GyrocDataCalibation(GyroData_raw, GyroData_NED);

                if(m_is_first_read_gsensor)
                {
                    m_is_first_read_gsensor = 0;
                    m_pre_imu_timestamp = imu_timestamp;
                    AccDataFilter[0] = AccData_NED[0];
                    AccDataFilter[1] = AccData_NED[1];
                    AccDataFilter[2] = AccData_NED[2];
                    GyroDataFilter[0] = GyroData_NED[0];
                    GyroDataFilter[1] = GyroData_NED[1];
                    GyroDataFilter[2] = GyroData_NED[2]; 
                }else
                {
                    double dt_imu = imu_timestamp - m_pre_imu_timestamp;                    
                    m_imu_attitude_estimate.LowpassFilter3f(AccDataFilter, AccData_NED, dt_imu, m_acc_filt_hz, AccDataFilter);    
                    m_imu_attitude_estimate.LowpassFilter3f(GyroDataFilter, GyroData_NED, dt_imu, m_gyro_filt_hz, GyroDataFilter);       
                    m_pre_imu_timestamp = imu_timestamp;    
                }                    
                m_imu_data.timestamp = imu_timestamp;
                UpdateCurrentDataTimestamp(imu_timestamp);
                m_imu_data.acc[0] = AccDataFilter[0];
                m_imu_data.acc[1] = AccDataFilter[1];
                m_imu_data.acc[2] = AccDataFilter[2];
                m_imu_data.gyro[0] = GyroDataFilter[0];
                m_imu_data.gyro[1] = GyroDataFilter[1];
                m_imu_data.gyro[2] = GyroDataFilter[2];
                m_data_gsensor_update = 1; 
            }else if(data_flag == "brake_signal")
            {
                string str_t[10],str_speed;
                int data_t[10];
                double raw_timestamp[2];  
                double speed_can;
                ss_log>>raw_timestamp[0]>>raw_timestamp[1]
                    >>str_t[0]>>data_t[0]>>str_t[1]>>data_t[1]>>str_t[2]>>data_t[2]>>str_t[3]>>data_t[3]>>str_t[4]>>data_t[4]
                    >>str_t[5]>>data_t[5]>>str_t[6]>>data_t[6]>>str_t[7]>>data_t[7]>>str_t[8]>>data_t[8]>>str_t[9]>>data_t[9]
                    >>str_speed>>speed_can;  
                m_can_speed_data.timestamp = raw_timestamp[0] + raw_timestamp[1]*1e-6;;
                UpdateCurrentDataTimestamp(m_can_speed_data.timestamp);
                m_can_speed_data.speed= speed_can/3.6;// km/h-->m/s   
                m_data_speed_update = 1;  
            }             
        }
        usleep(1);
    }
}


// 更新当前fusion的时间戳，用于控制读取数据的长度
void DataFusion::UpdateCurrentFusionTimestamp( double data_timestample)
{
    if(m_is_first_fusion_timestamp)
    {
        m_is_first_fusion_timestamp = 0;
        m_cur_fusion_timestamp = data_timestample;
    }else
    {
        if(m_cur_fusion_timestamp < data_timestample)
        {
            m_cur_fusion_timestamp = data_timestample;
        }
    }
}

// 更新当前data的时间戳，用于控制读取数据的长度
void DataFusion::UpdateCurrentDataTimestamp( double data_timestample)
{
    if(m_is_first_data_timestamp)
    {
        m_cur_data_timestamp = data_timestample;
        m_is_first_data_timestamp = 0;
    }else
    {
        if(m_cur_data_timestamp < data_timestample)
        {
            m_cur_data_timestamp = data_timestample;
        }
    }
}


// 判断是否还要继续读取数据
bool  DataFusion::UpdateRreadDataState( )
{
    double dt  = m_cur_data_timestamp - m_call_predict_timestamp;
    // 提前读取data_save_length长度的数据
    if(dt > 0 && dt >= m_data_save_length)  // 时间超过了
    {
        m_is_continue_read_data = 0; //  暂停读取数据
    }else
    {
        m_is_continue_read_data = 1;
    }

    return m_is_continue_read_data;
}

// 根据设定最长记忆时间的历史数据，删除多余数据
void DataFusion::DeleteHistoryData( )
{
    double dt;
    int att_data_length = m_vector_att.size();    
    int delete_conter = 0;
    for(int i=0; i<att_data_length; i++)
    {
        dt = (m_vector_att.begin()+i)->timestamp - m_call_predict_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }
    
    if(delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_att.erase(m_vector_att.begin(), m_vector_att.begin()+delete_conter);
    }    

    // 汽车运动数据
    int vehicle_data_length = m_vector_vehicle_state.size(); 
    delete_conter = 0;
    for(int i=0; i<vehicle_data_length; i++)
    {
        dt = (m_vector_vehicle_state.begin()+i)->timestamp - m_call_predict_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }
    
    if(delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_vehicle_state.erase(m_vector_vehicle_state.begin(), m_vector_vehicle_state.begin()+delete_conter);  
    }
    
}

// 进行汽车运动信息解算和航向角变化解算
int DataFusion::RunFusion( )
{
    double att_xy_cur[3]; // 当前stamp的角度
    while(1)
    {
        // m_is_continue_read_data
        if(m_data_gsensor_update)
        {
            double cur_att_timestamp = m_imu_data.timestamp;
            if(m_isFirstTime_att)
            {
                m_isFirstTime_att = 0;
                m_pre_att_timestamp = cur_att_timestamp; 
            }else
            {            
                double dt_att = cur_att_timestamp - m_pre_att_timestamp;                
                m_imu_attitude_estimate.UpdataAttitude(m_imu_data.acc, m_imu_data.gyro, dt_att);
                m_pre_att_timestamp = cur_att_timestamp;

                // save att
                m_imu_attitude_estimate.GetAttitude(m_struct_att.att);
                m_struct_att.timestamp = cur_att_timestamp;
                m_vector_att.push_back(m_struct_att);
            } 
            UpdateCurrentFusionTimestamp( cur_att_timestamp );
            m_data_gsensor_update = 0;          
        }

        if(m_data_speed_update)
        {    
            // 利用imu+speed计算汽车运动
            double cur_can_timestamp = m_can_speed_data.timestamp; 
            if(m_is_first_speed_data)
            {
                m_is_first_speed_data = 0;
                m_pre_can_timestamp = cur_can_timestamp;  
                
            }else
            {
                double dt_can = cur_can_timestamp - m_pre_can_timestamp;                
                m_imu_attitude_estimate.GetAttitude(att_xy_cur);
                m_can_vehicle_estimate.UpdateVehicleStateImu(att_xy_cur[2], m_can_speed_data.speed, dt_can );
                m_pre_can_timestamp = cur_can_timestamp;

                // save vehicle state            
                m_can_vehicle_estimate.GetVehicleState(m_struct_vehicle_state.vel, m_struct_vehicle_state.pos, &(m_struct_vehicle_state.yaw));
                m_struct_vehicle_state.timestamp = cur_can_timestamp;
                m_vector_vehicle_state.push_back(m_struct_vehicle_state);
            }
            UpdateCurrentFusionTimestamp( cur_can_timestamp );            
            m_data_speed_update = 0;        
        }        
        DeleteHistoryData();
        usleep(1); // 1us
    }
        
    return 1;
}

// 拟合曲线
int DataFusion::Polyfit(const cv::Mat& xy_feature, int order, std::vector<float>* lane_coeffs )
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
            LOG(INFO)<<"cv:solve error!!!"<<endl;
            return -1;
        }
        
        for(int i=0; i<order+1; i++)
        {
            lane_coeffs->push_back(coeffs.at<float>(i,0));
        }    
        return 1;
}


// 根据时间戳查找对应的数据
int DataFusion::GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3] )
{
    // m_vector_att
    bool att_data_search_ok = 0;
    bool vehicle_data_search_ok = 0;
    int att_data_length = m_vector_att.size();
    double timestamp_t, timestamp_t_1, dt_t, dt_t_pre;
    for(int i = 1; i<att_data_length; i++)
    {
        timestamp_t = (m_vector_att.end()-i)->timestamp;
        timestamp_t_1 = (m_vector_att.end()-i-1)->timestamp;
        
        dt_t = timestamp_t - timestamp_search;
        dt_t_pre = timestamp_t_1 - timestamp_search;
        // 选取离timestamp_search最近的时刻，2种条件满足一种即可
        if(dt_t_pre<0 && dt_t>0)
        {
            att[0] = (m_vector_att.end()-i)->att[0];
            att[1] = (m_vector_att.end()-i)->att[1];
            att[2] = (m_vector_att.end()-i)->att[2];
            att_data_search_ok = 1;
            break;
        }
    }

    // m_vector_vehicle_state
    int vehicle_data_length = m_vector_vehicle_state.size();
    for(int i = 1; i<vehicle_data_length; i++)
    {
        timestamp_t = (m_vector_vehicle_state.end()-i)->timestamp;
        timestamp_t_1 = (m_vector_vehicle_state.end()-i-1)->timestamp;
        
        dt_t = timestamp_t - timestamp_search;
        dt_t_pre = timestamp_t_1 - timestamp_search;
        // 选取离timestamp_search最近的时刻，2种条件满足一种即可
        if(dt_t_pre<0 && dt_t>0)
        {
            vehicle_pos[0] = (m_vector_vehicle_state.end()-i)->pos[0];
            vehicle_pos[1] = (m_vector_vehicle_state.end()-i)->pos[1];
            vehicle_data_search_ok = 1;
            break;
        }
    }

    if(att_data_search_ok && vehicle_data_search_ok)
        return 1;
    else
        return 0;    
}

// 获取预测的车道线参数(主要用来自己的本地测试)
int DataFusion::GetLanePredictParameter(double image_timestamp_cur, double image_timestamp_pre, const cv::Mat &lane_coeffs_pre, 
                                                    double lane_num, double m_order, cv::Mat* lane_coeffs_predict )
{
    bool data_search_cur = 0; // 搜索指定时间戳的数据
    bool data_search_pre = 0;    
    double att_cur[3], att_pre[3], vehicle_pos_cur[2], vehicle_pos_pre[2];
    // 更新时间戳
    m_call_predict_timestamp = image_timestamp_cur; 
   
    // 寻找跟需求的 timestamp对应的att,vehicle数据
    data_search_cur = GetTimestampData( image_timestamp_cur, vehicle_pos_cur, att_cur);
    data_search_pre = GetTimestampData( image_timestamp_pre, vehicle_pos_pre, att_pre);

    if(data_search_cur && data_search_pre)
    {
        LanePredict( lane_coeffs_pre, lane_num, m_order, vehicle_pos_pre, att_pre, vehicle_pos_cur, att_cur, lane_coeffs_predict);
        
        LOG(INFO)<<"LanePredict_new-vehicle_pos_pre "<<vehicle_pos_pre[0]<<" "<<vehicle_pos_pre[1]<<endl;   
        LOG(INFO)<<"LanePredict_new-vehicle_pos_cur "<<vehicle_pos_cur[0]<<" "<<vehicle_pos_cur[1]<<endl; 
        LOG(INFO)<<"LanePredict_new-att_cur "<<att_cur[0]<<" "<<att_cur[1]<<" "<<att_cur[2]<<" "<<endl; 
        LOG(INFO)<<"LanePredict_new-att_pre "<<att_pre[0]<<" "<<att_pre[1]<<" "<<att_pre[2]<<" "<<endl; 
    }
    
    if( !data_search_cur )
    {
        LOG(INFO)<<"!!!error cur--camera timestamp dismatch"<<endl;
    }

    if(!data_search_pre)
    {
        LOG(INFO)<<"!!!error pre--camera timestamp dismatch"<<endl;
    }
    return 1;    
    
}


// 车道线预测
// lane_coeffs_pre: 每一列代表一个样本
int DataFusion::LanePredict( const cv::Mat& lane_coeffs_pre, double lane_num, double m_order, 
                                     const double vehicle_pos_pre[2], const double att_pre[3],
                                     const double vehicle_pos_cur[2],  const double att_cur[3], cv::Mat* lane_coeffs_predict)
{
    /// init:(待定)
    int lane_points_nums = 5; // 每一条车道线取的样本点数量
    double X[5] = {2.0, 5.0, 10.0, 20.0, 35.0};
    LOG(INFO)<<"lane_coeffs_pre: "<<lane_coeffs_pre<<endl<<endl;

    // 计算汽车在两帧之间的状态变化    
    // 对pos进行坐标系转换，转到以pre时刻为初始坐标
    double d_pos_tmp[2]; // 前后两帧在初始坐标系下的汽车运动
    double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
    d_pos_tmp[0] = vehicle_pos_cur[0] - vehicle_pos_pre[0];
    d_pos_tmp[1] = vehicle_pos_cur[1] - vehicle_pos_pre[1];         
    d_pos_new_c[0] = cosf(att_pre[2])*d_pos_tmp[0] + sinf(att_pre[2])*d_pos_tmp[1];
    d_pos_new_c[1] = -sinf(att_pre[2])*d_pos_tmp[0] + cos(att_pre[2])*d_pos_tmp[1];
    
    double dyaw = att_cur[2] - att_pre[2]; 
    double Rn2c_kT[2][2];
    Rn2c_kT[0][0] = cosf(dyaw);
    Rn2c_kT[0][1] = sinf(dyaw);
    Rn2c_kT[1][0] = -Rn2c_kT[0][1];
    Rn2c_kT[1][1] = Rn2c_kT[0][0];
    
    LOG(INFO)<<"dyaw: "<<dyaw<<endl; 
    LOG(INFO)<<"vehicle_pos: "<<vehicle_pos_cur[0]<<" "<<vehicle_pos_cur[1]<<endl;
    LOG(INFO)<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<" "<<vehicle_pos_pre[1]<<endl; 
    LOG(INFO)<<"d_pos_new_c: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1]<<endl; 
    LOG(INFO)<<"vehicle_yaw_pre: "<<att_pre[2]*180/3.14<<endl;     
    
    // 从车道线参数中获取特征点,并预测特征点
    cv::Mat xy_feature_pre = cv::Mat::zeros(2, lane_points_nums, CV_32FC1);; // 上一帧的中车道线的特征点
    cv::Mat xy_feature_predict = cv::Mat::zeros(2, lane_points_nums, CV_32FC1); //  预测的当前帧的中车道线的特征点
    for(int lane_index=0; lane_index<lane_num; lane_index++)
    {        
        for(int points_index = 0; points_index<lane_points_nums; points_index++)
        {
            // Y = aX + b, X = {5, 10, 20, 30, 50}        
            xy_feature_pre.at<float>(0, points_index) = X[points_index];
            xy_feature_pre.at<float>(1, points_index) = lane_coeffs_pre.at<float>(0, lane_index) + lane_coeffs_pre.at<float>(1, lane_index)*X[points_index] + lane_coeffs_pre.at<float>(2, lane_index)*X[points_index]*X[points_index];

            // NED坐标系下的
            double dx = xy_feature_pre.at<float>(0, points_index) - d_pos_new_c[0];
            double dy = xy_feature_pre.at<float>(1, points_index) - d_pos_new_c[1];
            xy_feature_predict.at<float>(1, points_index) = Rn2c_kT[0][0]*dx + Rn2c_kT[0][1]*dy;
            xy_feature_predict.at<float>(0, points_index) = Rn2c_kT[1][0]*dx + Rn2c_kT[1][1]*dy;
        }

        LOG(INFO)<<"xy_feature_predict: "<<xy_feature_predict<< endl << endl; 
        
        // 车道线拟合    Y = AX(X是纵轴)
        std::vector<float> lane_coeffs_t;
        Polyfit( xy_feature_predict, m_order, &lane_coeffs_t );

        for(int i = 0; i<m_order+1; i++)
        {
            lane_coeffs_predict->at<float>(i, lane_index) = lane_coeffs_t[i];
        }

        LOG(INFO)<<"predict_lane_coeffs: "<<lane_coeffs_t[0]<<" "<<lane_coeffs_t[1]<<" "<<lane_coeffs_t[2]<<endl;     
        
    }

    return 1;
}

// 给外部调用的接口:特征点预测
int DataFusion::GetPredictFeature( const std::vector<cv::Point2f>& vector_feature_pre ,int64 image_timestamp_pre, int64 image_timestamp_cur, 
                                               std::vector<cv::Point2f>* vector_feature_predict)
{
    bool is_data_search_cur_ok; // 搜索指定时间戳的数据是否成功
    bool is_data_search_pre_ok;    
    double att_cur[3], att_pre[3], vehicle_pos_cur[2], vehicle_pos_pre[2];
    double image_timestamp_cur_t = image_timestamp_cur/1000.0;
    double image_timestamp_pre_t = image_timestamp_pre/1000.0;    
    
    m_call_predict_timestamp = image_timestamp_cur/1000.0; // 更新时间戳
   
    // 寻找跟需求的 timestamp对应的att,vehicle数据
    is_data_search_cur_ok = GetTimestampData( image_timestamp_cur_t, vehicle_pos_cur, att_cur);
    is_data_search_pre_ok = GetTimestampData( image_timestamp_pre_t, vehicle_pos_pre, att_pre);

    if(is_data_search_cur_ok && is_data_search_pre_ok)
    {
        FeaturePredict( vector_feature_pre , vehicle_pos_pre, att_pre, vehicle_pos_cur, att_cur, vector_feature_predict);
        
        LOG(INFO)<<"LanePredict_new-vehicle_pos_pre "<<vehicle_pos_pre[0]<<" "<<vehicle_pos_pre[1]<<endl;   
        LOG(INFO)<<"LanePredict_new-vehicle_pos_cur "<<vehicle_pos_cur[0]<<" "<<vehicle_pos_cur[1]<<endl; 
        LOG(INFO)<<"LanePredict_new-att_cur "<<att_cur[0]<<" "<<att_cur[1]<<" "<<att_cur[2]<<" "<<endl; 
        LOG(INFO)<<"LanePredict_new-att_pre "<<att_pre[0]<<" "<<att_pre[1]<<" "<<att_pre[2]<<" "<<endl; 

        return 1; 
    }
    
    if( !is_data_search_cur_ok )
    {
        LOG(INFO)<<"!!!error cur-camera timestamp dismatch"<<endl;
    }

    if(!is_data_search_pre_ok)
    {
        LOG(INFO)<<"!!!error pre-camera timestamp dismatch"<<endl;
    }
    return 0;    
    
}


// 特征点预测
int DataFusion::FeaturePredict( const std::vector<cv::Point2f>& vector_feature_pre , double vehicle_pos_pre[2], double att_pre[3], 
                                         double vehicle_pos_cur[2], double att_cur[3], std::vector<cv::Point2f>* vector_feature_predict)
{
    // 计算汽车在两帧之间的状态变化    
    // 对pos进行坐标系转换，转到以pre时刻为初始坐标
    double d_pos_tmp[2]; // 前后两帧在初始坐标系下的汽车运动
    double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
    d_pos_tmp[0] = vehicle_pos_cur[0] - vehicle_pos_pre[0];
    d_pos_tmp[1] = vehicle_pos_cur[1] - vehicle_pos_pre[1];         
    d_pos_new_c[0] = cosf(att_pre[2])*d_pos_tmp[0] + sinf(att_pre[2])*d_pos_tmp[1];
    d_pos_new_c[1] = -sinf(att_pre[2])*d_pos_tmp[0] + cos(att_pre[2])*d_pos_tmp[1];
    
    double dyaw = att_cur[2] - att_pre[2]; 
    double Rn2c_kT[2][2];
    Rn2c_kT[0][0] = cosf(dyaw);
    Rn2c_kT[0][1] = sinf(dyaw);
    Rn2c_kT[1][0] = -Rn2c_kT[0][1];
    Rn2c_kT[1][1] = Rn2c_kT[0][0];  
    
    // 预测特征点
    double feature_points_nums = vector_feature_pre.size();  
    cv::Point2f XY_pre, XY_predict;
    vector_feature_predict->clear(); // 清空数据
    for(int points_index = 0; points_index<feature_points_nums; points_index++)
    {        
        XY_pre.x = (vector_feature_pre.begin()+points_index)->x;
        XY_pre.y = (vector_feature_pre.begin()+points_index)->y;
        
        double dx = XY_pre.x - d_pos_new_c[0];// NED坐标系下的
        double dy = XY_pre.y - d_pos_new_c[1];
        XY_predict.x = Rn2c_kT[0][0]*dx + Rn2c_kT[0][1]*dy;
        XY_predict.y = Rn2c_kT[1][0]*dx + Rn2c_kT[1][1]*dy;

        vector_feature_predict->push_back(XY_predict);
    }   

    LOG(INFO)<<"lane_coeffs_pre: "<<vector_feature_pre<<endl;
    LOG(INFO)<<"vehicle_pos: "<<vehicle_pos_cur[0]<<" "<<vehicle_pos_cur[1]<<endl;
    LOG(INFO)<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<" "<<vehicle_pos_pre[1]<<endl; 
    LOG(INFO)<<"d_pos_new_c: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1]<<endl; 
    LOG(INFO)<<"dyaw: "<<dyaw<<endl; 
    LOG(INFO)<<"vehicle_yaw_pre: "<<att_pre[2]*180/3.14<<endl; 
    LOG(INFO)<<"xy_feature_predict: "<<vector_feature_predict << endl; 

    return 1;
}



