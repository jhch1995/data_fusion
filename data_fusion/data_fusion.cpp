#include "data_fusion.h"
#include <unistd.h>

# define DATA_FROM_LOG 1  // 从log中读取数据
# define DATA_FROM_ONLINE 0 //在线读取数据

using namespace common;
DataFusion::DataFusion()
{
    Initialize();    
}

DataFusion::~DataFusion()
{
    m_is_running = false;
    m_fusion_thread.StopAndWaitForExit();
    
}


void DataFusion::Initialize( )
{
    if(USE_RADIUS)
    {
        infile_log.open("data/radius/log.txt");       // ofstream  
    }
    else
    {
        infile_log.open("data/doing/log.txt");       // ofstream   
    }
            
    m_pre_vehicle_timestamp = 0.0f; /// CAN;

    // IMU
    m_imu_sample_hz = 100.0;
    m_acc_filt_hz = 5.0; // 加速度计的低通截止频率
    m_gyro_filt_hz = 20.0;
    m_isFirstTime_att = 1; // 是否是第一次进入
    m_pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻     
    m_is_first_speed_data = 1; //  1: 第一次获取到speed数据 0:不是第一次

    // 转弯半径R
    m_gyro_R_filt_hz = 2.0;
    m_can_speed_R_filt_hz = 2.0;
    m_call_radius_timestamp = 0;
    
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
    m_cur_data_timestamp = 0;
    m_call_predict_timestamp = 0;
    m_is_first_run_read_data = 1; // 第一次运行读取数据

    // 数据锁
    is_imu_data_allow_write = 1;
    is_can_speed_allow_write = 1;

}

// 开始线程
void DataFusion::StartDataFusionTask()
{
    m_fusion_thread.Start();
    m_is_running= 1;

    Closure<void>* cls = NewClosure(this, &DataFusion::RunFusion);
    m_fusion_thread.AddTask(cls);
}

// 线程循环执行的函数
void DataFusion::RunFusion( )
{
    while(1)
    {
        ReadData();
        if(m_data_gsensor_update)
        {
            EstimateAtt();
            EstimateVehicelState(); 
            CalculateVehicleTurnRadius();

            m_data_gsensor_update = 0;  
        }
            
        DeleteOldData();   
        usleep(1); // 1us       
    }
}


// 读取数据,两种方式:
//      1.离线从log   
// TODO:利用FIFO信息进行IMU时间戳逆推
// TODO: 2.在线
void DataFusion::ReadData( )
{    
    double log_data[2], timestamp_raw[2];
    string data_flag;
    //更新is_continue_read_data
    UpdateRreadDataState();
    if(DATA_FROM_LOG)
    {
        // 第一次运行程序，初始化m_cur_data_timestamp
        if(m_is_first_run_read_data)
        {
            getline(infile_log, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>log_data[0]>>log_data[1]>>data_flag; 
            m_cur_data_timestamp = log_data[0] + log_data[1]*1e-6;
            m_is_first_run_read_data = 0;
        }
        
        if(m_is_continue_read_data )
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
            }
            else if(data_flag == "Gsensor")
            {            
                double acc_data_raw[3]; // acc原始坐标系下的
                double acc_data_ned[3]; // 大地坐标系
                static double acc_data_filter[3]; // 一阶低通之后的数据
                double gyro_data_raw[3];
                double gyro_data_ned[3];  
                static double gyro_data_filter[3]; // 一阶低通之后的数据
                double imu_temperature, imu_timestamp;    
                string imu_flag;

                ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>imu_flag>>acc_data_raw[0]>>acc_data_raw[1]>>acc_data_raw[2]
                        >>gyro_data_raw[0]>>gyro_data_raw[1]>>gyro_data_raw[2]>>imu_temperature;
                imu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;                 
                m_imu_attitude_estimate.AccDataCalibation(acc_data_raw, acc_data_ned);// 原始数据校正
                m_imu_attitude_estimate.GyrocDataCalibation(gyro_data_raw, gyro_data_ned);

                if(m_is_first_read_gsensor)
                {
                    m_is_first_read_gsensor = 0;
                    m_pre_imu_timestamp = imu_timestamp;
                    acc_data_filter[0] = acc_data_ned[0];
                    acc_data_filter[1] = acc_data_ned[1];
                    acc_data_filter[2] = acc_data_ned[2];
                    gyro_data_filter[0] = gyro_data_ned[0];
                    gyro_data_filter[1] = gyro_data_ned[1];
                    gyro_data_filter[2] = gyro_data_ned[2]; 
                }
                else
                {
                    double dt_imu = imu_timestamp - m_pre_imu_timestamp; 
                    dt_imu = 1/m_imu_sample_hz; // 100hz
                    m_imu_attitude_estimate.LowpassFilter3f(acc_data_filter, acc_data_ned, dt_imu, m_acc_filt_hz, acc_data_filter);    
                    m_imu_attitude_estimate.LowpassFilter3f(gyro_data_filter, gyro_data_ned, dt_imu, m_gyro_filt_hz, gyro_data_filter);       
                    m_pre_imu_timestamp = imu_timestamp;    
                }

                // 更新数据
                m_imu_data.timestamp = imu_timestamp;
                for(int i = 0; i<3; i++)
                {
                    m_imu_data.acc[i] = acc_data_filter[i];
                    m_imu_data.gyro[i] = gyro_data_filter[i];
                    
                    m_imu_data.acc_raw[i] = acc_data_ned[i];
                    m_imu_data.gyro_raw[i] = gyro_data_ned[i];                    
                }
                    
//                memcpy( &(m_imu_data.acc_raw[0]), acc_data_ned, 3);
//                memcpy( &(m_imu_data.gyro_raw[0]), gyro_data_ned, 3);
//                memcpy( &(m_imu_data.acc[0]), acc_data_filter, 3);
//                memcpy( &(m_imu_data.gyro[0]), gyro_data_filter, 3);

                UpdateCurrentDataTimestamp(imu_timestamp);
                m_data_gsensor_update = 1;    
               
            }else if(data_flag == "brake_signal")
            {
                string str_t[10],str_speed;
                int data_t[10];
                double raw_timestamp[2];  
                double speed_can, speed_timestamp;
                ss_log>>raw_timestamp[0]>>raw_timestamp[1]
                    >>str_t[0]>>data_t[0]>>str_t[1]>>data_t[1]>>str_t[2]>>data_t[2]>>str_t[3]>>data_t[3]>>str_t[4]>>data_t[4]
                    >>str_t[5]>>data_t[5]>>str_t[6]>>data_t[6]>>str_t[7]>>data_t[7]>>str_t[8]>>data_t[8]>>str_t[9]>>data_t[9]
                    >>str_speed>>speed_can;  
                
                speed_timestamp = raw_timestamp[0] + raw_timestamp[1]*1e-6;   
                speed_can = speed_can/3.6;// km/h-->m/s 

                // 更新数据
                m_can_speed_data.timestamp = speed_timestamp;
                m_can_speed_data.speed = speed_can; 
                UpdateCurrentDataTimestamp(speed_timestamp);
                m_data_speed_update = 1;  

            }

        }
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


// 判断是否还要继续读取数据，如果run_fusion在进行操作的时候，不读数据
bool  DataFusion::UpdateRreadDataState( )
{
    double dt  = m_cur_data_timestamp - m_call_predict_timestamp;
    // 提前读取data_save_length长度的数据
    if(dt >= m_data_save_length)  // 时间超过了
    {
        m_is_continue_read_data = 0; //  暂停读取数据
    }else
    {
        m_is_continue_read_data = 1;
    }

    return m_is_continue_read_data;
}

// 根据设定最长记忆时间的历史数据，删除多余数据
void DataFusion::DeleteOldData( )
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


// 根据设定最长记忆时间的历史数据，删除多余转弯半径的数据
void DataFusion::DeleteOldRadiusData( )
{
    double dt;
    int R_data_length = m_vector_turn_radius.size();    
    int delete_conter = 0;
    for(int i=0; i<R_data_length; i++)
    {
        dt = (m_vector_turn_radius.begin()+i)->time - m_call_radius_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }
    
    if(delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_turn_radius.erase(m_vector_turn_radius.begin(), m_vector_turn_radius.begin()+delete_conter);
    }    
}


void DataFusion::EstimateAtt()
{
    StructImuData imu_data;
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));     
    
    double cur_att_timestamp = imu_data.timestamp;
    if(m_isFirstTime_att)
    {
        m_isFirstTime_att = 0;
        m_pre_att_timestamp = cur_att_timestamp; 
    }else
    {  
        VLOG(VLOG_DEBUG)<<"run fusion imu" <<endl;
        
        double dt_att = cur_att_timestamp - m_pre_att_timestamp;
        dt_att = 1/m_imu_sample_hz;
        m_imu_attitude_estimate.UpdataAttitude(imu_data.acc, imu_data.gyro, dt_att);
        m_pre_att_timestamp = cur_att_timestamp;

        // save att
        m_imu_attitude_estimate.GetAttitudeAngleZ(m_struct_att.att, &(m_struct_att.angle_z));
        m_struct_att.timestamp = cur_att_timestamp;
        m_vector_att.push_back(m_struct_att);
    } 
    UpdateCurrentFusionTimestamp( cur_att_timestamp );        
    
}


void DataFusion::EstimateVehicelState()
{  
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));

    // 利用imu+speed计算汽车运动
    double dt;
    double cur_vehicle_timestamp = m_struct_att.timestamp; 
    if(m_is_first_speed_data)
    {
        m_is_first_speed_data = 0;
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;  
        
    }else
    {
        //dt = cur_vehicle_timestamp - m_pre_vehicle_timestamp; // 暂时没用
        dt = 1/m_imu_sample_hz; // 每次IMU更新数据便计算一次
        m_can_vehicle_estimate.UpdateVehicleStateImu(m_struct_att.angle_z, can_speed_data.speed, dt );
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;

        // save vehicle state            
        m_can_vehicle_estimate.GetVehicleState(m_struct_vehicle_state.vel, m_struct_vehicle_state.pos, &(m_struct_vehicle_state.yaw));
        m_struct_vehicle_state.timestamp = cur_vehicle_timestamp;
        m_vector_vehicle_state.push_back(m_struct_vehicle_state);
    }

}


// 根据时间戳查找对应的数据
int DataFusion::GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z )
{
    // m_vector_att
    bool att_data_search_ok = 0;
    bool vehicle_data_search_ok = 0;
    int att_data_length = m_vector_att.size();
    double timestamp_cur, timestamp_pre, dt_t_cur, dt_t_pre, dt_t;
    for(int i = 1; i<att_data_length; i++)
    {
        timestamp_cur = (m_vector_att.end()-i)->timestamp;
        timestamp_pre = (m_vector_att.end()-i-1)->timestamp;
        dt_t = timestamp_cur - timestamp_pre;
        dt_t_cur = timestamp_cur - timestamp_search;
        dt_t_pre = timestamp_pre - timestamp_search;
 
        if(dt_t_pre<0 && dt_t_cur>0 && dt_t>=0)
        {
            // 方法: 线性差插值
            double att_pre[3], att_cur[3], d_att[3];            
            for(int k = 0; k<3; k++)
            {
                att_pre[k] = (m_vector_att.end()-i-1)->att[k];
                att_cur[k] = (m_vector_att.end()-i)->att[k];
                d_att[k] = att_cur[k] - att_pre[k];                
                att[k] = att_pre[k] + (fabs(dt_t_pre)/fabs(dt_t))*d_att[k];// 线性插值                
            }

            double angle_z_pre, angle_z_cur, d_angle_z;
            angle_z_pre = (m_vector_att.end()-i-1)->angle_z;
            angle_z_cur = (m_vector_att.end()-i)->angle_z;
            d_angle_z = angle_z_cur - angle_z_pre;                
            *angle_z = angle_z_pre + (fabs(dt_t_pre)/fabs(dt_t))*d_angle_z;// 线性插值

            att_data_search_ok = 1;
            break;
        }
    }
    VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"att:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 

    // m_vector_vehicle_state
    int vehicle_data_length = m_vector_vehicle_state.size();
    for(int i = 1; i<vehicle_data_length; i++)
    {        
        timestamp_cur = (m_vector_vehicle_state.end()-i)->timestamp;
        timestamp_pre = (m_vector_vehicle_state.end()-i-1)->timestamp;
        dt_t = timestamp_cur - timestamp_pre;
        dt_t_cur = timestamp_cur - timestamp_search;
        dt_t_pre = timestamp_pre - timestamp_search;
        if(dt_t_pre<0 && dt_t_cur>0)
        {
             // 方法: 线性差插值
            double pos_pre[2], pos_cur[2], d_pos[2] ;
            for(int k=0; k<2; k++)
            {
                pos_pre[k] = (m_vector_vehicle_state.end()-i-1)->pos[k];
                pos_cur[k] = (m_vector_vehicle_state.end()-i)->pos[k];
                d_pos[k] = pos_cur[k] - pos_pre[k];               
                vehicle_pos[k] = pos_pre[k] + (fabs(dt_t_pre)/fabs(dt_t))*d_pos[k];     // 线性插值            
            }            
            vehicle_data_search_ok = 1;
            break;
        }
    }
    VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"pos:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 

    if(att_data_search_ok && vehicle_data_search_ok)
        return 1;
    else
        return 0;    
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
    is_data_search_pre_ok = GetTimestampData( image_timestamp_pre_t, vehicle_pos_pre, att_pre, &m_angle_z_pre);
    is_data_search_cur_ok = GetTimestampData( image_timestamp_cur_t, vehicle_pos_cur, att_cur, &m_angle_z_cur);
    
    VLOG(VLOG_DEBUG)<<"DF:GetPredictFeature--"<<"call: dt(ms) = "<<image_timestamp_cur - image_timestamp_pre<<endl; 


    if(is_data_search_cur_ok && is_data_search_pre_ok)
    {
        FeaturePredict( vector_feature_pre , vehicle_pos_pre, att_pre, m_angle_z_pre, vehicle_pos_cur, att_cur, m_angle_z_pre, vector_feature_predict);
        return 1; 
    }
    else if(!is_data_search_pre_ok && !is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning cur & pre-camera both timestamp dismatch"<<endl;
    }
    else if(!is_data_search_pre_ok && is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning pre-camera timestamp dismatch"<<endl;
    }
    else if( is_data_search_pre_ok && !is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning cur-camera timestamp dismatch"<<endl;
    }
    return 0;    
    
}


// 特征点预测
int DataFusion::FeaturePredict( const std::vector<cv::Point2f>& vector_feature_pre , double vehicle_pos_pre[2], double att_pre[3], double angle_z_pre, 
                                         double vehicle_pos_cur[2], double att_cur[3], double angle_z_cur, std::vector<cv::Point2f>* vector_feature_predict)
{
    // 计算汽车在两帧之间的状态变化    
    // 对pos进行坐标系转换，转到以pre时刻为初始坐标
    double d_pos_tmp[2]; // 在初始时刻的汽车坐标系下，前后两帧的运动位置变化
    double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
    d_pos_tmp[0] = vehicle_pos_cur[0] - vehicle_pos_pre[0];
    d_pos_tmp[1] = vehicle_pos_cur[1] - vehicle_pos_pre[1];         
//    d_pos_new_c[0] = cosf(att_pre[2])*d_pos_tmp[0] + sinf(att_pre[2])*d_pos_tmp[1];
//    d_pos_new_c[1] = -sinf(att_pre[2])*d_pos_tmp[0] + cos(att_pre[2])*d_pos_tmp[1];
    d_pos_new_c[0] = cosf(angle_z_pre)*d_pos_tmp[0] + sinf(angle_z_pre)*d_pos_tmp[1];
    d_pos_new_c[1] = -sinf(angle_z_pre)*d_pos_tmp[0] + cos(angle_z_pre)*d_pos_tmp[1];


    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<", "<<vehicle_pos_pre[1]; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_cur: "<<vehicle_pos_cur[0]<<", "<<vehicle_pos_cur[1]; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"d_pos: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1];
   
    double dyaw = att_cur[2] - att_pre[2]; 
    // test
    double dangle_z = m_angle_z_cur - m_angle_z_pre; 

    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_pre: "<<att_pre[0]*180/M_PI<<", "<<att_pre[1]*180/M_PI<<", "<<att_pre[2]*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_cur: "<<att_cur[0]*180/M_PI<<", "<<att_cur[1]*180/M_PI<<", "<<att_cur[2]*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dyaw: "<<dyaw*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dangle_z: "<<dangle_z*180/M_PI; 

    dyaw = dangle_z;
    
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
//        VLOG(VLOG_DEBUG)<<"DF:FeaturePredict- "<<"xy_feature_pre: "<<"x="<<XY_pre.x<<", y="<< XY_pre.y<< endl; 
//        VLOG(VLOG_DEBUG)<<"DF:FeaturePredict- "<<"xy_feature_predict: "<<"x="<<XY_predict.x<<", y="<< XY_predict.y<< endl; 
    } 

    return 1;
}



// 计算汽车的转弯半径
void DataFusion::CalculateVehicleTurnRadius()
{
    double R;
    // IMU
    StructImuData imu_data;
    static double gyro_filter_R[3]; // 一阶低通之后的数据 
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));    
    double dt_imu= 1/m_imu_sample_hz;
    m_imu_attitude_estimate.LowpassFilter3f(gyro_filter_R, imu_data.gyro_raw, dt_imu, m_gyro_R_filt_hz, &gyro_filter_R[0]);    

    // CAN speed
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));
   
    if(fabs(gyro_filter_R[2])>0.01 && fabs(can_speed_data.speed)>4) 
    {
        R = can_speed_data.speed/gyro_filter_R[2];
    }
    else // 太小的速度和角速度
    {        
        R = 0;
    }
    
    // save R
    m_struct_turn_radius.time = imu_data.timestamp;
    m_struct_turn_radius.R = R;
    m_vector_turn_radius.push_back(m_struct_turn_radius);
    
}


// 给外部调用的接口:特征点预测
int DataFusion::GetTurnRadius( const int64 &int_timestamp_search, double *R)
{
    bool R_search_ok = 0;
    int R_data_length = m_vector_turn_radius.size();
    double time_cur, time_pre, dt_t_cur, dt_t_pre, dt_t;
    double R_t;
    double timestamp_search = int_timestamp_search/1000.0;
    
    m_call_predict_timestamp = timestamp_search;// 更新时间戳
    for(int i = 1; i<R_data_length; i++)
    {
       time_cur = (m_vector_turn_radius.end()-i)->time;
       time_pre = (m_vector_turn_radius.end()-i-1)->time;
       dt_t = time_cur - time_pre;
       dt_t_cur = time_cur - timestamp_search;
       dt_t_pre = time_pre - timestamp_search;

       if(dt_t_pre<0 && dt_t_cur>0 && dt_t>=0)
       {
           // 方法: 线性差插值
           double R_pre, R_cur, d_R;            
           R_pre = (m_vector_turn_radius.end()-i-1)->R;
           R_cur = (m_vector_turn_radius.end()-i)->R;
           d_R = R_cur - R_pre;                
           R_t = R_pre + (fabs(dt_t_pre)/fabs(dt_t))*d_R;// 线性插值                

           R_search_ok = 1;
           break;
       }
    }
    
    VLOG(VLOG_INFO)<<"DF:GetTurnRadius--"<<"R:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 
    if(R_search_ok)
    {
        *R = R_t;
        return 1;
    }
    else
    {
        *R = 0;
        return 0;
    }
    

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

