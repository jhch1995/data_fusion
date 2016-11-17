#include "data_fusion.h"

using namespace common;

namespace imu {

DataFusion::DataFusion()
{
    Init();
}

DataFusion::~DataFusion()
{
#if defined(DATA_FROM_LOG)
    infile_log.close();
#endif

    m_is_running = false;
    m_fusion_thread.StopAndWaitForExit();
}


void DataFusion::Init( )
{   
    m_init_state = 0; // imu默认是初始化成功的(为了)

    // IMU
    m_imu_sample_hz = 100.0;
    m_imu_dt_set = 1/m_imu_sample_hz;
    m_acc_filt_hz = 5.0; // 加速度计的低通截止频率
    m_gyro_filt_hz = 20.0;
    m_isFirstTime_att = 1; // 是否是第一次进入
    m_pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻
    m_is_first_speed_data = 1; //  1: 第一次获取到speed数据 0:不是第一次
    m_is_print_imu_data = 0; // 是否打印IMU数据
    m_is_print_speed_data = 0;

    // 转弯半径R
    m_is_first_R_filter = 1;
    m_gyro_R_filt_hz = 0.2;
    m_can_speed_R_filt_hz = 1.0;
    m_call_radius_timestamp = 0;
    m_is_R_ok = false;

    // read data
    m_is_first_read_gsensor = 1;
    m_data_gsensor_update = 0;
    m_data_speed_update = 0;
    m_data_image_update = 0;
    m_pre_vehicle_timestamp = 0.0f; /// CAN;

    m_can_speed_data.speed = 0;
    m_can_speed_data.timestamp = 0;
    memset(&m_imu_data, 0, sizeof(StructImuData));
    memset(&m_image_frame_info, 0, sizeof(StructImageFrameInfo));
    memset(&m_struct_vehicle_state, 0, sizeof(StructVehicleState));
    memset(&m_struct_att, 0, sizeof(StructAtt));
    memset(&m_struct_turn_radius, 0, sizeof(StructTurnRadius));

    // 读取数据控制
    m_is_first_fusion_timestamp = 1; // 第一次更新
    m_is_first_data_timestamp = 1; // 第一次更新
    m_data_save_length = 2; // 保存历史数据的长度(时间为单位: s)
    m_is_continue_read_data = 1; // 1:继续读取数据  2:暂停读取数据 由fusion控制
    m_cur_data_timestamp = 0;
    m_call_predict_timestamp = 0;
    m_is_first_run_read_data = 1; // 第一次运行读取数据

    #if !defined(DATA_FROM_LOG)
    {
        int stste = init_gsensor();
        if(stste >= 0){
            m_init_state = true;
            printf("init_gsensor OK!\n");
            LOG(ERROR) << "DataFusion::Init--init_gsensor OK!";
        }else{
            m_init_state = false;
            printf("init_gsensor ERROR!!!\n");
            LOG(ERROR) << "DataFusion::Init--init_gsensor ERROR!!!";
        }
    }
    #else
    {
        m_init_state = true; // 读取log的时候认为是imu初始化是OK的
        // read data from log
        infile_log.open("data/doing/log.txt");       // ofstream
        LOG(ERROR) << "try open \"data/doing/log.txt\"\n";
        if(!infile_log){
            LOG(ERROR) << "open \"data/doing/log.txt\" ERROR!!\n";
            printf("open \"data/doing/log.txt\" ERROR!!\n");
        }else{
            VLOG(VLOG_WARNING) << "open \"data/doing/log.txt\" OK!\n";
            printf("open \"data/doing/log.txt\" OK!\n");
        }
    }
    #endif

}

// 开始线程
void DataFusion::StartDataFusionTask()
{
    m_fusion_thread.Start();
    m_is_running = true;

    Closure<void>* cls = NewClosure(this, &DataFusion::RunFusion);
    m_fusion_thread.AddTask(cls);
}

// 线程循环执行的函数
void DataFusion::RunFusion( )
{
    struct timeval time_counter_pre, time_counter_cur;
    int64_t dt_counter;
    int64_t run_fusion_period_us = 5000;// 数据生成的频率是100hz， 读取判断最好是2倍，保证数据的实时性

    if(m_init_state){
        while (m_is_running) {
            gettimeofday(&time_counter_pre, NULL); // 用于控制频率
            ReadData(); // 数据保存在  m_vector_imu_data
            int vector_imu_length = m_vector_imu_data.size();
            for (int i = 0; i < vector_imu_length; i++){
                // 只有当IMU有有效数据的时候，才进行fusion
                m_imu_data = *(m_vector_imu_data.begin() + i);
                EstimateAtt();
                EstimateVehicelState();
                CalculateVehicleTurnRadius();

                // 更新数据，清除历史数据
                DeleteOldData();
                DeleteOldRadiusData();
                m_vector_imu_data.clear(); // 清空缓存的buffer
            }

        // 因为在PC端，日志可能会很大, 可以考虑不进行刷新频率控制；
        // 但是在板子上不存在这个问题，所以进行刷新频率控制，减少无意义资源占用
        #if !defined(DATA_FROM_LOG)
        {
            gettimeofday(&time_counter_cur, NULL);
            dt_counter = (time_counter_cur.tv_sec - time_counter_pre.tv_sec)*1000000 + (time_counter_cur.tv_usec - time_counter_pre.tv_usec);
            if(dt_counter < run_fusion_period_us){   // 10ms
                int64_t sleep_us = run_fusion_period_us - dt_counter;
                //printf("run fusion use time = %d\n", dt_counter);
                usleep(sleep_us);
            }
        }
        #endif
        }
    }else{
        LOG(ERROR)<<"DF:init imu failed!!!"<<endl;
        printf("DF:imu init failed!!!!\n");        
    }
}


// 读取数据,两种方式:
//  1.离线从log
//  2.在线
int DataFusion::ReadData( )
{
    int rtn_state;
    #if defined(DATA_FROM_LOG)
    {
        //从log中离线读取数据，需要通过时间戳来确定是否要继续读取数据，更新is_continue_read_data
        UpdateRreadDataState();
        rtn_state = ReadDataFromLog();
        return rtn_state;
    }
    #else
    {
        int imu_state = ReadImuOnline();
        int speed_state = ReadSpeedOnline();
        //TODO: rtn_state
        return imu_state;
    }
    #endif
}

int DataFusion::ReadDataFromLog( )
{
    double log_data[2], timestamp_raw[2];
    string data_flag;
    struct StructImuData imu_data;

    if(m_is_continue_read_data){
        getline(infile_log, buffer_log);
        ss_tmp.clear();
        ss_tmp.str(buffer_log);
        ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
        ss_log.clear();
        ss_log.str(buffer_log);

        if(data_flag == "cam_frame"){
            string camera_flag, camera_add, image_index_str;
            string image_name;
            int log_image_index;
            ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>camera_flag>>camera_add>>log_image_index;

            m_image_frame_info.timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
            m_image_frame_info.index = log_image_index;
            m_data_image_update = 1;
        }else if(data_flag == "Gsensor"){
            double acc_data_raw[3]; // acc原始坐标系下的
            double acc_data_ned[3]; // 大地坐标系
            static double acc_data_filter_pre[3]; // 保存fliter变量
            double acc_data_filter[3];
            double gyro_data_raw[3];
            double gyro_data_ned[3];
            static double gyro_data_filter_pre[3];
            double gyro_data_filter[3];
            double imu_temperature, imu_timestamp;
            string imu_flag;

            ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>imu_flag>>acc_data_raw[0]>>acc_data_raw[1]>>acc_data_raw[2]
                    >>gyro_data_raw[0]>>gyro_data_raw[1]>>gyro_data_raw[2]>>imu_temperature;
            imu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
            m_imu_attitude_estimate.AccDataCalibation(acc_data_raw, acc_data_ned);// 原始数据校正
            m_imu_attitude_estimate.GyrocDataCalibation(gyro_data_raw, gyro_data_ned);

            if(m_is_first_read_gsensor){
                m_is_first_read_gsensor = 0;
                m_pre_imu_timestamp = imu_timestamp;
                memcpy(acc_data_filter_pre, acc_data_ned, sizeof(double)*3);
                memcpy(gyro_data_filter_pre, gyro_data_ned, sizeof(double)*3);
            }else{
                double dt_imu = imu_timestamp - m_pre_imu_timestamp;
                dt_imu = 1/m_imu_sample_hz; // 100hz
                m_imu_attitude_estimate.LowpassFilter3f(acc_data_filter_pre, acc_data_ned, dt_imu, m_acc_filt_hz, acc_data_filter);
                memcpy(acc_data_filter_pre, acc_data_filter, sizeof(double)*3);

                m_imu_attitude_estimate.LowpassFilter3f(gyro_data_filter_pre, gyro_data_ned, dt_imu, m_gyro_filt_hz, gyro_data_filter);
                memcpy(gyro_data_filter_pre, gyro_data_filter, sizeof(double)*3);
                m_pre_imu_timestamp = imu_timestamp;
            }

            // 更新数据
            for(int i = 0; i<3; i++){
                imu_data.acc[i] = acc_data_filter[i];
                imu_data.gyro[i] = gyro_data_filter[i];
                imu_data.acc_raw[i] = acc_data_ned[i];
                imu_data.gyro_raw[i] = gyro_data_ned[i];
            }
            imu_data.timestamp = imu_timestamp;
            imu_data.temp = imu_temperature;
            m_vector_imu_data.push_back(imu_data); // 保存IMU data的 vector
            UpdateCurrentDataTimestamp(imu_timestamp);
            m_data_gsensor_update = 1;

        }else if(data_flag == "brake_signal"){
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
    return 1;
}



// 在线读取imu数据
int DataFusion::ReadImuOnline( )
{
    int imu_fifo_total;
    int fifo_reseted = 0;
    double time_imu_read; // 从IMU的FIFO中读取数据的当前系统时间
    struct timeval time_imu;
    struct GsensorData data_raw[80];
    struct StructImuData imu_data;

    imu_fifo_total = read_gsensor(data_raw, sizeof(data_raw)/sizeof(data_raw[0]));
    gettimeofday(&time_imu, NULL);
    time_imu_read = time_imu.tv_sec + time_imu.tv_usec*1e-6;

    // IMU数据读取异常判断
    if (imu_fifo_total <= 0){
        if( GSENSOR_READ_AGAIN == imu_fifo_total)
            return GSENSOR_READ_AGAIN;
        else if (GSENSOR_FIFO_RESET == imu_fifo_total) {
            fifo_reseted = 1 ;// IMU FIFO 溢出
            LOG(ERROR) << "ReadImuOnline: IMU fifo leak \n";
            return GSENSOR_FIFO_RESET;
        }
        else
            return GSENSOR_READ_FAIL;
    }

    for (int imu_data_index = 0; imu_data_index < imu_fifo_total; imu_data_index++){
        double acc_data_raw[3]; // acc原始坐标系下的
        double acc_data_ned[3]; // 大地坐标系
        static double acc_data_filter_pre[3];
        double acc_data_filter[3];
        double gyro_data_raw[3];
        double gyro_data_ned[3];
        static double gyro_data_filter_pre[3];
        double gyro_data_filter[3];
        double imu_timestamp, imu_temperature;

        for(int k=0; k<3; k++){
            acc_data_raw[k] = data_raw[imu_data_index].accel[k];
            gyro_data_raw[k] = data_raw[imu_data_index].gyro[k];
        }
        imu_temperature = Raw2Degree(data_raw[imu_data_index].temp);
        // 利用FIFO数据长度和当前的时间戳，进行IMU时间逆推
        imu_timestamp = time_imu_read - (imu_fifo_total-1-imu_data_index)*m_imu_dt_set;

        m_imu_attitude_estimate.AccDataCalibation(acc_data_raw, acc_data_ned);// 原始数据校正
        m_imu_attitude_estimate.GyrocDataCalibation(gyro_data_raw, gyro_data_ned);

        if(m_is_first_read_gsensor){
            // 第一次进入函数的初始化
            memcpy(acc_data_filter_pre, acc_data_ned, sizeof(double)*3);
            memcpy(gyro_data_filter_pre, gyro_data_ned, sizeof(double)*3);
            m_is_first_read_gsensor = 0;
        }else{
            m_imu_attitude_estimate.LowpassFilter3f(acc_data_filter_pre, acc_data_ned, m_imu_dt_set, m_acc_filt_hz, acc_data_filter);
            memcpy(acc_data_filter_pre, acc_data_filter, sizeof(double)*3);

            m_imu_attitude_estimate.LowpassFilter3f(gyro_data_filter_pre, gyro_data_ned, m_imu_dt_set, m_gyro_filt_hz, gyro_data_filter);
            memcpy(gyro_data_filter_pre, gyro_data_filter, sizeof(double)*3);
        }

        // 更新数据
        imu_data.timestamp = imu_timestamp;
        imu_data.temp = imu_temperature;
        for(int i = 0; i<3; i++){
            imu_data.acc[i] = acc_data_filter[i];
            imu_data.gyro[i] = gyro_data_filter[i];
            imu_data.acc_raw[i] = acc_data_ned[i];
            imu_data.gyro_raw[i] = gyro_data_ned[i];
        }
        m_vector_imu_data.push_back(imu_data); // 保存IMU data的 vector
        m_data_gsensor_update = 1;
        fifo_reseted = 0;

        // test
        if(m_is_print_imu_data){
            char buf1[256];
            snprintf(buf1, sizeof(buf1), "imu %f %f %f %f %f %f %f %f %d %d", imu_data.timestamp,
                        imu_data.acc[0], imu_data.acc[1], imu_data.acc[2],
                        imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2], imu_data.temp, imu_fifo_total, fifo_reseted);

            if (imu_fifo_total >= 1)
                printf("#%02d %s\n", imu_data_index, buf1);
        }
    }
    UpdateCurrentDataTimestamp(time_imu_read);

    return imu_fifo_total;
}

// 读取speed数据
int DataFusion::ReadSpeedOnline( )
{
    struct timeval time_speed;
    gettimeofday(&time_speed, NULL);
    double speed_time_t = time_speed.tv_sec + time_speed.tv_usec*1e-6;
    double speed_can = HalIO::Instance().GetSpeed()/3.6;

    // 更新数据
    m_can_speed_data.timestamp = speed_time_t;
    m_can_speed_data.speed = speed_can;

    if(m_is_print_speed_data){
        char buf1[256];
        snprintf(buf1, sizeof(buf1), "speed %f %f", m_can_speed_data.timestamp, speed_can);
        printf("# %s\n",  buf1);
    }

    return 0;
}


// 更新当前data的时间戳，用于控制读取数据的长度
void DataFusion::UpdateCurrentDataTimestamp( double data_timestample)
{
    if(m_is_first_data_timestamp){
        m_cur_data_timestamp = data_timestample;
        m_is_first_data_timestamp = 0;
    }else{
        if(m_cur_data_timestamp < data_timestample)
            m_cur_data_timestamp = data_timestample;
    }
}


// 判断是否还要继续读取数据，如果run_fusion在进行操作的时候，不读数据
bool  DataFusion::UpdateRreadDataState( )
{
    double dt  = m_cur_data_timestamp - m_call_predict_timestamp;
    // 提前读取data_save_length长度的数据
    if(dt >= m_data_save_length)  // 时间超过了
        m_is_continue_read_data = 0; //  暂停读取数据
    else
        m_is_continue_read_data = 1;

    return m_is_continue_read_data;
}

// 根据设定最长记忆时间的历史数据，删除多余数据
void DataFusion::DeleteOldData( )
{
    double dt;
    int att_data_length = m_vector_att.size();
    int delete_conter = 0;
    double cur_timestamp = 0;

    #if defined(DATA_FROM_LOG)
    {
        feature_rw_lock.ReaderLock();
        cur_timestamp = m_call_predict_timestamp;
        feature_rw_lock.Unlock();
    }
    #else
    {
        struct timeval time_speed;
        gettimeofday(&time_speed, NULL);
        cur_timestamp = time_speed.tv_sec + time_speed.tv_usec*1e-6;
    }
    #endif

    for(int i=0; i<att_data_length; i++){
        dt = (m_vector_att.begin()+i)->timestamp - cur_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }

    if(delete_conter > 0)
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_att.erase(m_vector_att.begin(), m_vector_att.begin()+delete_conter);


    // 汽车运动数据
    int vehicle_data_length = m_vector_vehicle_state.size();
    delete_conter = 0;
    for(int i=0; i<vehicle_data_length; i++){
        dt = (m_vector_vehicle_state.begin()+i)->timestamp - cur_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }

    if(delete_conter > 0)
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_vehicle_state.erase(m_vector_vehicle_state.begin(), m_vector_vehicle_state.begin()+delete_conter);

}


// 根据设定最长记忆时间的历史数据，删除多余转弯半径的数据
void DataFusion::DeleteOldRadiusData( )
{
    double dt;
    int R_data_length = m_vector_turn_radius.size();
    int R_delete_conter = 0;
    double R_cur_timestamp = 0;

    #if defined(DATA_FROM_LOG)
    {
        radius_rw_lock.ReaderLock();
        R_cur_timestamp = m_call_predict_timestamp;
        radius_rw_lock.Unlock();
    }
    #else
    {
        struct timeval time_R;
        gettimeofday(&time_R, NULL);
        R_cur_timestamp = time_R.tv_sec + time_R.tv_usec*1e-6;
    }
    #endif

    for(int i=0; i<R_data_length; i++){
        dt = (m_vector_turn_radius.begin()+i)->timestamp - R_cur_timestamp;
        if(dt < -m_data_save_length)
            R_delete_conter++;
        else
            break;
    }

    if(R_delete_conter > 0)
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_turn_radius.erase(m_vector_turn_radius.begin(), m_vector_turn_radius.begin()+R_delete_conter);
}


void DataFusion::EstimateAtt()
{
    StructImuData imu_data;
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));

    double cur_att_timestamp = imu_data.timestamp;
    if(m_isFirstTime_att){
        m_isFirstTime_att = 0;
        m_pre_att_timestamp = cur_att_timestamp;
    }else{
        double dt_att = cur_att_timestamp - m_pre_att_timestamp;
        dt_att = 1/m_imu_sample_hz;
        m_imu_attitude_estimate.UpdataAttitude(imu_data.acc, imu_data.gyro, dt_att);
        m_pre_att_timestamp = cur_att_timestamp;

        // save att
        m_imu_attitude_estimate.GetAttitudeAngleZ(m_struct_att.att, &(m_struct_att.angle_z));
        m_struct_att.timestamp = cur_att_timestamp;
        m_vector_att.push_back(m_struct_att);

        VLOG(VLOG_WARNING)<<"DF:EstimateAtt--"<<"gyro_data = "<<imu_data.gyro[0]<<", "<<imu_data.gyro[1]<<", "<<imu_data.gyro[2]<<", "<<endl;
        VLOG(VLOG_WARNING)<<"DF:EstimateAtt--"<<"angle_z = "<<m_struct_att.angle_z<<endl;
    }
}


void DataFusion::EstimateVehicelState()
{
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));

    // 利用imu+speed计算汽车运动
    double dt;
    double cur_vehicle_timestamp = m_struct_att.timestamp;
    if(m_is_first_speed_data){
        m_is_first_speed_data = 0;
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;

    }else{
        //dt = cur_vehicle_timestamp - m_pre_vehicle_timestamp; // 暂时没用
        dt = 1/m_imu_sample_hz; // 每次IMU更新数据便计算一次

        m_can_vehicle_estimate.UpdateVehicleStateImu(m_struct_att.angle_z, can_speed_data.speed, dt );
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;

        // save vehicle state
        m_can_vehicle_estimate.GetVehicleState(m_struct_vehicle_state.vel, m_struct_vehicle_state.pos, &(m_struct_vehicle_state.yaw));
        m_struct_vehicle_state.timestamp = cur_vehicle_timestamp;
        m_vector_vehicle_state.push_back(m_struct_vehicle_state);

        VLOG(VLOG_WARNING)<<"DF:EstimateVehicelState--"<<"speed = "<<can_speed_data.speed<<endl;
        VLOG(VLOG_WARNING)<<"DF:EstimateVehicelState--"<<"pos = "<<m_struct_vehicle_state.pos[0]<<", "<<m_struct_vehicle_state.pos[1]<<endl;
        VLOG(VLOG_WARNING)<<"DF:EstimateVehicelState--"<<"vel = "<<m_struct_vehicle_state.vel[0]<<", "<<m_struct_vehicle_state.vel[1]<<endl;
    }

}


// 根据时间戳查找对应的数据
// 1: 数据正常
// -1:int_timestamp_search < all_data_time 落后
// -2:int_timestamp_search > all_data_time 超前
int DataFusion::GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z )
{
    // m_vector_att
    bool att_data_search_ok = 0;
    bool vehicle_data_search_ok = 0;
    int att_data_length = m_vector_att.size();
    double timestamp_cur, timestamp_pre, dt_t_cur, dt_t_pre, dt_t;
    int data_search_state = 0;

    if(att_data_length >= 2){
        for(int i = 1; i<att_data_length; i++){
            timestamp_cur = (m_vector_att.end()-i)->timestamp;
            timestamp_pre = (m_vector_att.end()-i-1)->timestamp;
            dt_t = timestamp_cur - timestamp_pre;
            dt_t_cur = timestamp_cur - timestamp_search;
            dt_t_pre = timestamp_pre - timestamp_search;

            if(dt_t_pre<=0 && dt_t_cur>=0 && dt_t>=0){
                // 方法: 线性差插值
                double att_pre[3], att_cur[3], d_att[3];
                for(int k = 0; k<3; k++){
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

        if(!att_data_search_ok){
            // 判断是什么原因没匹配上
            if(dt_t_pre<0 && dt_t_cur<0){
                VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the att data time is fall behid the timestamp_search!!!\n"<<endl;
                data_search_state = -1; // vector_R中数据都落后于timestamp_search
            }else if(dt_t_pre>0 && dt_t_cur>0){
                 VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the att data time is ahead of the timestamp_search!!!\n"<<endl;
                data_search_state = -2; // vector_R中数据都早于timestamp_search
            }

            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"att, nearest_time= "<<std::fixed<<(m_vector_att.end()-1)->timestamp<<", farthest_time= "
                           <<std::fixed<<m_vector_att.begin()->timestamp<<endl;
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"att_length= "<<att_data_length<<", att:(ms) "<<"dt_t_cur= "
                           <<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl;
        }
    }else{
        VLOG(VLOG_WARNING)<<"DF:GetTimestampData--"<<"!!!WARNING:att data too less att_length= "<<att_data_length<<endl;
    }

    // m_vector_vehicle_state
    int vehicle_data_length = m_vector_vehicle_state.size();
    if(vehicle_data_length>=2){
        for(int i = 1; i<vehicle_data_length; i++){
            timestamp_cur = (m_vector_vehicle_state.end()-i)->timestamp;
            timestamp_pre = (m_vector_vehicle_state.end()-i-1)->timestamp;
            dt_t = timestamp_cur - timestamp_pre;
            dt_t_cur = timestamp_cur - timestamp_search;
            dt_t_pre = timestamp_pre - timestamp_search;
            if(dt_t_pre<=0 && dt_t_cur>=0){
                 // 方法: 线性差插值
                double pos_pre[2], pos_cur[2], d_pos[2] ;
                for(int k=0; k<2; k++){
                    pos_pre[k] = (m_vector_vehicle_state.end()-i-1)->pos[k];
                    pos_cur[k] = (m_vector_vehicle_state.end()-i)->pos[k];
                    d_pos[k] = pos_cur[k] - pos_pre[k];
                    vehicle_pos[k] = pos_pre[k] + (fabs(dt_t_pre)/fabs(dt_t))*d_pos[k];     // 线性插值
                }
                vehicle_data_search_ok = 1;
                break;
            }
        }

        if(!vehicle_data_search_ok){
            // 判断是什么原因没匹配上
            if(dt_t_pre<0 && dt_t_cur<0)
                VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the vehicle_data time is fall behid the timestamp_search!!!\n"<<endl;
            else if(dt_t_pre>0 && dt_t_cur>0)
                VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the vehicle_data time is ahead of the timestamp_search!!!\n"<<endl;
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"vehicle_state, nearest_time= "<<(m_vector_vehicle_state.end()-1)->timestamp
                           <<", farthest_time= "<<m_vector_vehicle_state.begin()->timestamp<<endl;
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"vehicle_length= "<<vehicle_data_length<<", pos:(ms) "
                           <<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl;
        }
    }else{
        VLOG(VLOG_WARNING)<<"DF:GetTimestampData--"<<"!!!WARNING:vehicle data too less vehicle_data_length= "<<vehicle_data_length<<endl;
    }

    if(att_data_search_ok && vehicle_data_search_ok){
        return 1;
    }else{
        return data_search_state;
    }
}


// 给外部调用的接口:特征点预测
// 1: 数据正常
// -1:int_timestamp_search < all_data_time 落后
// -2:int_timestamp_search > all_data_time 超前
int DataFusion::GetPredictFeature( const std::vector<cv::Point2f>& vector_feature_pre ,int64 image_timestamp_pre, int64 image_timestamp_cur,
                                               std::vector<cv::Point2f>* vector_feature_predict)
{
    int data_search_state_cur=0, data_search_state_pre = 0; // 搜索指定时间戳的数据是否成功
    double att_cur[3], att_pre[3], vehicle_pos_cur[2], vehicle_pos_pre[2];
    double image_timestamp_cur_t = image_timestamp_cur/1000.0;
    double image_timestamp_pre_t = image_timestamp_pre/1000.0;
    int predict_state = 0;

    feature_rw_lock.WriterLock();
    m_call_predict_timestamp = image_timestamp_cur/1000.0; // 更新时间戳
    feature_rw_lock.Unlock();

    // 寻找跟需求的 timestamp对应的att,vehicle数据
    data_search_state_pre = GetTimestampData( image_timestamp_pre_t, vehicle_pos_pre, att_pre, &m_angle_z_pre);
    data_search_state_cur = GetTimestampData( image_timestamp_cur_t, vehicle_pos_cur, att_cur, &m_angle_z_cur);

    VLOG(VLOG_DEBUG)<<"DF:GetPredictFeature--"<<"call: pre(s) = "<<std::fixed<<image_timestamp_pre_t<<", cur(s): "<<std::fixed<<image_timestamp_cur_t<<endl;
    VLOG(VLOG_DEBUG)<<"DF:GetPredictFeature--"<<"call: dt(ms) = "<<image_timestamp_cur - image_timestamp_pre<<endl;

    if(data_search_state_cur && data_search_state_pre){
        FeaturePredict( vector_feature_pre , vehicle_pos_pre, att_pre, m_angle_z_pre, vehicle_pos_cur, att_cur, m_angle_z_pre, vector_feature_predict);
        return 1;
    }else if(data_search_state_pre == -1 || data_search_state_cur == -1){
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"state_pre = "<<data_search_state_pre<<"state_cur = "<<data_search_state_cur<<endl;
        predict_state = -1;
        return predict_state;
    }else if(data_search_state_pre == -2 || data_search_state_cur == -2){
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"state_pre = "<<data_search_state_pre<<"state_cur = "<<data_search_state_cur<<endl;
        predict_state = -2;
        return predict_state;
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


    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<", "<<vehicle_pos_pre[1]<<endl;
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_cur: "<<vehicle_pos_cur[0]<<", "<<vehicle_pos_cur[1]<<endl;
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"d_pos: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1]<<endl;

    double dyaw = att_cur[2] - att_pre[2];
    // test
    double dangle_z = m_angle_z_cur - m_angle_z_pre;

    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_pre: "<<att_pre[0]*180/M_PI<<", "<<att_pre[1]*180/M_PI<<", "<<att_pre[2]*180/M_PI<<endl;
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_cur: "<<att_cur[0]*180/M_PI<<", "<<att_cur[1]*180/M_PI<<", "<<att_cur[2]*180/M_PI<<endl;
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dyaw: "<<dyaw*180/M_PI<<endl;
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dangle_z: "<<dangle_z*180/M_PI<<endl;

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
    }
    return 1;
}



// 计算汽车的转弯半径
void DataFusion::CalculateVehicleTurnRadius()
{
    double R;
    StructImuData imu_data;
    static double gyro_filter_R_pre[3];
    double gyro_filter_R[3]; // 一阶低通之后的数据
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));
    double dt_imu= 1/m_imu_sample_hz;
    if(m_is_first_R_filter){
        memcpy(gyro_filter_R_pre, imu_data.gyro_raw, sizeof(double)*3);
        m_is_first_R_filter = 0;
    }

    m_imu_attitude_estimate.LowpassFilter3f(gyro_filter_R_pre, imu_data.gyro_raw, dt_imu, m_gyro_R_filt_hz, gyro_filter_R);
    memcpy(gyro_filter_R_pre, gyro_filter_R, sizeof(double)*3);
//    printf("filter new = %f, filter = %f\n", imu_data.gyro_raw[2], gyro_filter_R[2]);

    // CAN speed
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));

    m_struct_turn_radius.is_imu_value_ok = false;
    if(fabs(gyro_filter_R[2]) < 30/57.3 && fabs(imu_data.gyro_raw[2]) < 30/57.3 && fabs(imu_data.acc_raw[2])<40){
        if(fabs(gyro_filter_R[2])>0.002 && fabs(can_speed_data.speed)>15/3.6){
            R = can_speed_data.speed/gyro_filter_R[2];
        }else // 太小的速度和角速度
            R = 0;
        
        if( fabs(R)>20){
            m_struct_turn_radius.is_imu_value_ok= true;
        }else if(R == 0){ 
            m_struct_turn_radius.is_imu_value_ok= true;
        }else{           
            R = 0;            
            m_struct_turn_radius.is_imu_value_ok= false;
        }
    }else{
        m_struct_turn_radius.is_imu_value_ok = false;
        R = 0;
    }



    // save R
    m_struct_turn_radius.timestamp = imu_data.timestamp;
    m_struct_turn_radius.R = R;
    m_vector_turn_radius.push_back(m_struct_turn_radius);

}


// 给外部调用的接口:特征点预测
// 1: 数据正常
// -1:int_timestamp_search < all_data_time 落后
// -2:int_timestamp_search > all_data_time 超前
// -3:imu的值异常，默认返回R=0
// -4:imu初始化失败，默认返回R=0
int DataFusion::GetTurnRadius( const int64 &int_timestamp_search, double *R)
{
    bool R_search_ok = 0;
    int R_data_length = m_vector_turn_radius.size();
    double time_cur, time_pre, dt_t_cur, dt_t_pre, dt_t;
    double R_t = 0;
    double timestamp_search = int_timestamp_search/1000.0;
    int R_search_state = 0;
    bool is_imu_value_ok = false; // 判断imu的值是否正常

    radius_rw_lock.WriterLock();
    //m_call_radius_timestamp = timestamp_search;// 更新时间戳
    m_call_predict_timestamp  = timestamp_search;// 更新时间戳
    radius_rw_lock.Unlock();

    if(R_data_length >= 2){
        for(int i = 1; i<R_data_length; i++){
           time_cur = (m_vector_turn_radius.end()-i)->timestamp;
           time_pre = (m_vector_turn_radius.end()-i-1)->timestamp;
           dt_t = time_cur - time_pre;
           dt_t_cur = time_cur - timestamp_search;
           dt_t_pre = time_pre - timestamp_search;

           if(dt_t_pre<=0 && dt_t_cur>=0 && dt_t>=0){
               // 方法: 线性差插值
               double R_pre, R_cur, d_R;
               R_pre = (m_vector_turn_radius.end()-i-1)->R;
               R_cur = (m_vector_turn_radius.end()-i)->R;
               d_R = R_cur - R_pre;
               R_t = R_pre + (fabs(dt_t_pre)/fabs(dt_t))*d_R;// 线性插值
               R_search_ok = 1;

               is_imu_value_ok = (m_vector_turn_radius.end()-i)->is_imu_value_ok && (m_vector_turn_radius.end()-i-1)->is_imu_value_ok;
               break;
           }
        }

        if(!R_search_ok){
            // 判断是什么原因没匹配上
            if(dt_t_pre<0 && dt_t_cur<0){
                VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the R data time is fall behid the timestamp_search!!!\n"<<endl;
                R_search_state = -1; // vector_R中数据都落后于timestamp_search
            }else if(dt_t_pre>0 && dt_t_cur>0){
                VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"all the R data time is ahead of the timestamp_search!!!\n"<<endl;
                R_search_state = -2; // vector_R中数据都早于timestamp_search
            }

            VLOG(VLOG_WARNING)<<"DF:GetTurnRadius--"<<"R timestamp_search = "<<std::fixed<<timestamp_search<<", R data_length = "<<R_data_length<<endl;
            VLOG(VLOG_INFO)<<"DF:GetTurnRadius--"<<"vector_turn_radius_time= ["<<std::fixed<<m_vector_turn_radius.begin()->timestamp<<", "
                           <<std::fixed<<(m_vector_turn_radius.end()-1)->timestamp<<"]"<<endl;
            VLOG(VLOG_INFO)<<"DF:GetTurnRadius--"<<"att_length= "<<R_data_length<<", att:(ms) "<<"dt_t_cur= "
                           <<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl;
        }
    }else{
        VLOG(VLOG_WARNING)<<"DF:GetTurnRadius--"<<"!!!WARNING:radius data too less length= "<<R_data_length<<endl;
    }

    if(m_init_state){  // imu 初始化是否OK
        if(is_imu_value_ok ){ // imu值是否异常
            if(R_search_ok){  
                *R = R_t;
                return 1;
            }else{
                *R = 0;
                return R_search_state;
            }
        }else{ // imu值异常
            *R = 0;
            return -3;
        }
    }else{ // imu初始化失败
        *R = 0;
        return -4;
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

        for (int i = 0; i < feature_points_num; i++) {
            x[i] = xy_feature.at<float>(0, i);
            y[i] = xy_feature.at<float>(1, i);

            for (int j = 0; j <= order; j++){
                A.at<float>(i, j) = pow(y[i], j);
            }
            b.at<float>(i) = x[i];
        }

        cv::Mat coeffs;
        int ret = cv::solve(A, b, coeffs, CV_SVD);
        if(ret<=0){
            VLOG(VLOG_INFO)<<"cv:solve error!!!"<<endl;
            return -1;
        }

        for(int i=0; i<order+1; i++){
            lane_coeffs->push_back(coeffs.at<float>(i,0));
        }
        return 1;
}


float DataFusion::Raw2Degree(short raw)
{
    return (float (raw))/340 + 36.53;
}


void DataFusion::PrintImuData(const int is_print_imu)
{
    m_is_print_imu_data = is_print_imu;
}

void DataFusion::PrintSpeedData(const int is_print_speed)
{
    m_is_print_speed_data = is_print_speed;
}

// the strategy is to average 50 points over 0.5 seconds, then do it
// again and see if the 2nd average is within a small margin of the first
int DataFusion::CalibrateGyroBias(   double gyro_bias[3] )
{
	double gyro_sum[3],gyro_avg[3],gyro_diff[3], accel_diff[3], last_average[3], best_avg[3], accel_start[3];
	double gyro_diff_norm, acc_diff_norm;
	double pre_gyro_offset[3], new_gyro_offset[3], best_gyro_diff;
	int converged, num_converged;
    bool gyro_cal_ok = 0;
    StructImuData imu_data_t;
    int gyro_sample_num = 50; // the gyro sample numbers each cycle

    double m_gyro_drift[3];
    m_imu_attitude_estimate.GetGyroBias(m_gyro_drift);
    memcpy(pre_gyro_offset, m_gyro_drift, sizeof(m_gyro_drift));
    m_imu_attitude_estimate.ClearGyroBias(); // bias清零
    memset(new_gyro_offset, 0, sizeof(new_gyro_offset));
    memset(last_average, 0, sizeof(last_average));

	converged = false;
	num_converged = 0;

    // 预先读取1s,100个数据
    for ( int i=0; i<100; i++) {
        ReadImuOnline();// Getting imu data
        imu_data_t = *(m_vector_imu_data.begin());
        m_vector_imu_data.clear();
        usleep(10000); // 10ms
     }

	// we try to get a good calibration estimate for up to 30 seconds if the gyros are stable, we should get it in 1 second
    for ( int j_cal_cycle = 0; j_cal_cycle <= 30*4 && num_converged<5; j_cal_cycle++) {
	    printf("index:  %d\n", j_cal_cycle);
		gyro_diff_norm = 0.0f;
        memset(gyro_sum, 0, sizeof(gyro_sum));
	    memcpy(accel_start, imu_data_t.acc, sizeof(imu_data_t.acc));

        for ( int i=0; i<gyro_sample_num; i++) {
            ReadImuOnline();// Getting imu data
            imu_data_t = *(m_vector_imu_data.begin());
            m_vector_imu_data.clear();

			gyro_sum[0] += imu_data_t.gyro_raw[0];
			gyro_sum[1] += imu_data_t.gyro_raw[1];
			gyro_sum[2] += imu_data_t.gyro_raw[2];
            usleep(10000); // 10ms
        }

		accel_diff[0] = imu_data_t.acc[0] - accel_start[0];
		accel_diff[1] = imu_data_t.acc[1] - accel_start[1];
		accel_diff[2] = imu_data_t.acc[2] - accel_start[2];
		acc_diff_norm = sqrtf(accel_diff[0]*accel_diff[0] + accel_diff[1]*accel_diff[1] + accel_diff[2]*accel_diff[2]);
        printf("acc_diff_norm = %f\n", acc_diff_norm);
        if (acc_diff_norm > 0.3) {
            // the accelerometers changed during the gyro sum. Skip this sample. This copes with doing gyro cal on a
            // steadily moving platform. The value 0.2 corresponds with around 5 degrees/second of rotation.
            printf("acc_diff_norm > 0.3f, please keep the device stable!!!!\n");
            sleep(1);
            continue; // -YJ- comment
        }

		gyro_avg[0] = gyro_sum[0]/ gyro_sample_num;
		gyro_avg[1] = gyro_sum[1]/ gyro_sample_num;
		gyro_avg[2] = gyro_sum[2]/ gyro_sample_num;
		gyro_diff[0] = last_average[0] - gyro_avg[0];
		gyro_diff[1] = last_average[1] - gyro_avg[1];
		gyro_diff[2] = last_average[2] - gyro_avg[2];
        gyro_diff_norm = sqrtf(gyro_diff[0]*gyro_diff[0] + gyro_diff[1]*gyro_diff[1] + gyro_diff[2]*gyro_diff[2]);

        printf("gyro_avg = %f %f %f\n", gyro_avg[0], gyro_avg[1], gyro_avg[2]);
        printf("gyro_diff_norm=%f\n", gyro_diff_norm);

		if (j_cal_cycle == 0){
			best_gyro_diff = gyro_diff_norm;
            memcpy(best_avg, gyro_avg, sizeof(gyro_avg));

        } else if (gyro_diff_norm < 0.04/57.3){
            // we want the average to be within 0.04 degrees/s
            last_average[0] = (gyro_avg[0] * 0.5f) + (last_average[0] * 0.5f);
			last_average[1] = (gyro_avg[1] * 0.5f) + (last_average[1] * 0.5f);
			last_average[2] = (gyro_avg[2] * 0.5f) + (last_average[2] * 0.5f);

			// last_average_length
			double last_average_norm = sqrtf(last_average[0]*last_average[0] + last_average[1]*last_average[1] + last_average[2]*last_average[2]);
			// new_gyro_offset_length
			double new_gyro_offset_norm = sqrtf(new_gyro_offset[0]*new_gyro_offset[0] + new_gyro_offset[1]*new_gyro_offset[1] + new_gyro_offset[2]*new_gyro_offset[2]);

			// l_last_average < new_gyro_offset_length, the first time converged=0, so will goin the if
			if (!converged || last_average_norm < new_gyro_offset_norm)
                memcpy(new_gyro_offset, last_average, sizeof(last_average));

            converged = true;
            printf("gyro calibate converg times: %d, bias= %f %f %f\n", num_converged, new_gyro_offset[0], new_gyro_offset[1], new_gyro_offset[2]);
			if(num_converged++ >= 5)// 收敛的累计
                break;
        } else if (gyro_diff_norm < best_gyro_diff) {
            best_gyro_diff = gyro_diff_norm;
            best_avg[0] = (gyro_avg[0] * 0.5f) + (last_average[0] * 0.5f);
			best_avg[1] = (gyro_avg[1] * 0.5f) + (last_average[1] * 0.5f);
			best_avg[2] = (gyro_avg[2] * 0.5f) + (last_average[2] * 0.5f);
        }
        memcpy(last_average, gyro_avg, sizeof(gyro_avg));
    }

	// we've kept the user waiting long enough - use the best pair we found so far
	if (!converged) {
	    printf("gyro did not converge: diff=%f dps\n",  best_gyro_diff*57.3);
		// flag calibration as failed for this gyro
	    gyro_cal_ok = false;
        memcpy(gyro_bias, best_avg, sizeof(best_avg));
        return -1;
	} else {
	    gyro_cal_ok = true;
        memcpy(gyro_bias, new_gyro_offset, sizeof(new_gyro_offset));
        printf("gyro calibate success, bias= %f %f %f\n", new_gyro_offset[0], new_gyro_offset[1], new_gyro_offset[2]);
        return 0;
	}

}

}

