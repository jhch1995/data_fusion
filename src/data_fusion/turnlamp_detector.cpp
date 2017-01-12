#include "turnlamp_detector.h"

namespace imu {

TurnlampDetector::TurnlampDetector()
{
    Init( );
}

TurnlampDetector::~TurnlampDetector()
{
    #if defined(DATA_FROM_LOG)
        infile_log.close();
    #endif
    m_is_turnlamp_detect_running = false;
    m_turnlamp_detect_thread.StopAndWaitForExit();
}

void TurnlampDetector::Init( )
{
    m_is_first_read_fmu = true;
    m_fmu_sample_hz = 10.0/400.0; // 从飞控板子上读取的数据的采样频率

    memset(&m_fmu_imu_data, 0, sizeof(m_fmu_imu_data));
    memset(&m_fmu_imu_data_pre, 0, sizeof(m_fmu_imu_data_pre));
    m_is_first_detect_rod_shift = true;
    m_rod_shift_state = 0;

    m_d_acc_threshold[0] = 5.0;
    m_d_acc_threshold[1] = 5.0;
    m_d_acc_threshold[2] = 9.8;

    m_d_gyro_threshold[0] = 2.0;
    m_d_gyro_threshold[1] = 1.0;
    m_d_gyro_threshold[2] = 0.0;

    memset(m_R_fmu2camera, 0 , sizeof(m_R_fmu2camera));
    m_R_fmu2camera[0][0] = 1.0;
    m_R_fmu2camera[1][1] = 1.0;
    m_R_fmu2camera[2][2] = 1.0;

    m_is_turnlamp_detect_running = false;

    m_turnlamp_state = 0;
    m_turnlamp_state_pre = 0;
    m_turnlamp_state_change_CAN = false;

    #if defined(DATA_FROM_LOG)
    {
        // read data from log
        infile_log.open("./data/doing/log.txt"); // ifstream
        LOG(ERROR) << "try open " << FLAGS_log_data_addr;
        if(!infile_log)
            LOG(ERROR) << "open " << FLAGS_log_data_addr << " ERROR!!";
        else
            LOG(ERROR) << "open " << FLAGS_log_data_addr << " OK!";
    }
    #endif

}

// 开始线程
void TurnlampDetector::StartTurnlampDetectTaskOnline()
{
    m_turnlamp_detect_thread.Start();
    m_is_turnlamp_detect_running = true;
    Closure<void>* cls = NewClosure(this, &TurnlampDetector::DetectSelfRodShiftOnline);
    m_turnlamp_detect_thread.AddTask(cls);
}

// 读取log日志，进行检测
void TurnlampDetector::RunDetectTurnlampOffline()
{
    bool is_read_new_fmu = false;
    while(!is_read_new_fmu){
        int fmu_data_update = ReadFmuDataFromLog();
        if(fmu_data_update){
            VLOG(VLOG_INFO)<<"RunDetectTurnlamp--"<<"new fmu data"<<endl;
            m_rod_shift_state = DetectRodShift();
            is_read_new_fmu = true;
        }
    }
}


// 自身检测拨杆是否可能是被拨动了
void TurnlampDetector::DetectSelfRodShiftOnline()
{
    while (m_is_turnlamp_detect_running) {
        int fmu_data_update = 0;
         while(!fmu_data_update){
            fmu_data_update = ReadRodDataOnline(); // 直到读到rod的acc数据
            usleep(1000);
        }
        
        VLOG(VLOG_INFO)<<"RunDetectTurnlamp--"<<"new fmu data"<<endl;
        m_rod_shift_state = DetectRodShift();

       usleep(10000);
    }
}


// 仅利用拨杆上ＩＭＵ检测拨杆是否可能被拨动
// 1: 可能被拨动
// 0: 没拨动
int TurnlampDetector::DetectRodShift()
{
    if(m_is_first_detect_rod_shift){
        memcpy(&m_fmu_imu_data_pre, &m_fmu_imu_data, sizeof(m_fmu_imu_data_pre));
        m_is_first_detect_rod_shift = false;
    }else{
        double d_acc[3], d_gyro[3];
        for(int i=0; i<3; i++){
            d_acc[i] = m_fmu_imu_data.acc[i] - m_fmu_imu_data_pre.acc[i];
            d_gyro[i] = m_fmu_imu_data.gyro[i] - m_fmu_imu_data_pre.gyro[i];
        }        
        if(fabs(d_acc[2])>=m_d_acc_threshold[2]){
            VLOG(VLOG_INFO)<<"DetectRodShift--"<<"fmu_d_acc = "<<d_acc[0]<<" "<<d_acc[1]<<" "<<d_acc[2]<<" "<<endl;
            printf("Detect Rod Shift: d_acc= %f %f %f\n",d_acc[0], d_acc[1], d_acc[2]);
            return 1;
        }
    }
    return 0;
}

// 0: 没有读到新数据
// 1: fmu数据更新
int TurnlampDetector::ReadFmuDataFromLog( )
{
    double log_data[2], timestamp_raw[2];
    string data_flag;
    struct StructImuData imu_data;
    bool is_fmu_data_update = false;

    getline(infile_log, buffer_log);
    ss_tmp.clear();
    ss_tmp.str(buffer_log);
    ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
    ss_log.clear();
    ss_log.str(buffer_log);

    //  拨杆imu的数据
    if(data_flag == "FMU"){
        double acc_data_raw[3], acc_data_ned[3], acc_data_filter[3];
        static double acc_data_filter_pre[3]; // 保存fliter变量
        double gyro_data_raw[3], gyro_data_ned[3], gyro_data_filter[3];
        static double gyro_data_filter_pre[3];
        double fmu_temperature, fmu_timestamp;
        string fmu_flag, tmp_str[5];

        ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>fmu_flag>>tmp_str[0]>>tmp_str[1]>>tmp_str[2]>>tmp_str[3]
              >>acc_data_raw[0]>>acc_data_raw[1]>>acc_data_raw[2]
              >>gyro_data_raw[0]>>gyro_data_raw[1]>>gyro_data_raw[2]>>fmu_temperature;
        fmu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;

        // 比例因子缩放 和 坐标系变换
        acc_data_ned[0] = acc_data_raw[2]/100.0;
        acc_data_ned[1] = acc_data_raw[0]/100.0;
        acc_data_ned[2] = acc_data_raw[1]/100.0;

        gyro_data_ned[0] = gyro_data_raw[2]/10.0*D2R;
        gyro_data_ned[1] = gyro_data_raw[0]/10.0*D2R;
        gyro_data_ned[2] = gyro_data_raw[1]/10.0*D2R;

        double acc_data_ned_new[3], gyro_data_ned_new[3];
        Array3Rotation(m_R_fmu2camera, acc_data_ned, acc_data_ned_new);
        Array3Rotation(m_R_fmu2camera, gyro_data_ned, gyro_data_ned_new);

        m_fmu_imu_data.timestamp = fmu_timestamp;
        memcpy(m_fmu_imu_data.acc, acc_data_ned_new, sizeof(m_fmu_imu_data.acc));
        memcpy(m_fmu_imu_data.gyro, gyro_data_ned_new, sizeof(m_fmu_imu_data.gyro));

        // 低通滤波
//        if(m_is_first_read_fmu){
//            m_is_first_read_fmu = false;
//            m_pre_fmu_timestamp = fmu_timestamp;
//            memcpy(acc_data_filter_pre, acc_data_ned, sizeof(double)*3);
//            memcpy(gyro_data_filter_pre, gyro_data_ned, sizeof(double)*3);
//        }else{
//            double dt_fmu = fmu_timestamp - m_pre_fmu_timestamp;
//            dt_fmu = 1/m_fmu_sample_hz; //
//            m_fmu_attitude_estimate.LowpassFilter3f(acc_data_filter_pre, acc_data_ned, dt_imu, m_acc_filt_hz, acc_data_filter);
//            memcpy(acc_data_filter_pre, acc_data_filter, sizeof(double)*3);
//
//            m_fmu_attitude_estimate.LowpassFilter3f(gyro_data_filter_pre, gyro_data_ned, dt_imu, m_gyro_filt_hz, gyro_data_filter);
//            memcpy(gyro_data_filter_pre, gyro_data_filter, sizeof(double)*3);
//            m_pre_fmu_timestamp = fmu_timestamp;
//        }
        is_fmu_data_update = true;
    }else if(data_flag == "turnlamp"){
        string raw_flag;
        double turnlamp_timestamp;

        m_turnlamp_state_pre = m_turnlamp_state; // 保存上一次的state数据
        ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>raw_flag>>m_turnlamp_state;
        turnlamp_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
        if(m_turnlamp_state == 2)
            m_turnlamp_state = -1;

        // 判断turnlamp状态是否发生变化
        if(m_turnlamp_state != m_turnlamp_state_pre){
            VLOG(VLOG_INFO)<<"ReadFmuDataFromLog--"<<"!!!new turnlamp state = old:"<<m_turnlamp_state_pre<<" new:"<<m_turnlamp_state<<endl;
            m_turnlamp_state_change_CAN = true;
        }else{
            m_turnlamp_state_change_CAN = false;
        }
        
    }
    if(is_fmu_data_update)
        return 1;
    else
        return 0;
}

// 0: 没有读到新数据
// 1: fmu数据更新
int TurnlampDetector::ReadRodDataOnline( )
{
    double acc_data_raw[3], acc_data_ned[3], acc_data_ned_new[3];
    double rod_timestamp;
     
    ROD_DATA rod_data[20];
    int read_rod_state = HalIO::Instance().read_rod_data(rod_data, 20);
    if(read_rod_state){
        // 比例因子缩放 和 坐标系变换
        for(int k=0; k<3; k++)
            acc_data_ned[k] = rod_data[0].acc[k]/100.0;

        Array3Rotation(m_R_fmu2camera, acc_data_ned, acc_data_ned_new);
        m_fmu_imu_data.timestamp = rod_data[0].tv.tv_sec + rod_data[0].tv.tv_usec*1e-6;
        memcpy(m_fmu_imu_data.acc, acc_data_ned_new, sizeof(m_fmu_imu_data.acc));

        return 1;        
    }
    return 0;
}



// 两个IMU相互之间坐标转换矩阵
void TurnlampDetector::CalculateRotationFmu2Camera(const double acc_fmu[3], const double acc_camera[3])
{
    double att_fmu[3], att_camera[3];
    double R_fmu_n2b[3][3], R_camera_n2b[3][3]; // n2b 坐标转换矩阵
    att_fmu[0] = atan2f(-acc_fmu[1], -acc_fmu[2]); // roll
    att_fmu[1] = atan2f(acc_fmu[0], sqrtf(acc_fmu[1]*acc_fmu[1] + acc_fmu[2]*acc_fmu[2])); // pitch
    att_fmu[2] = 0.0;

    att_camera[0] = atan2f(-acc_camera[1], -acc_camera[2]); //roll
    att_camera[1] = atan2f(acc_camera[0], sqrtf(acc_camera[1]*acc_camera[1] + acc_camera[2]*acc_camera[2]));
    att_camera[2] = 0.0;

    m_fmu_attitude_estimate.CalculateAtt2Rotation(att_fmu, R_fmu_n2b);
    m_fmu_attitude_estimate.CalculateAtt2Rotation(att_camera, R_camera_n2b);

    // R_fmu2camera = R_camera_n2b * R_fmu_n2b'
    for(int  k=0; k<3; k++){
        for(int j=0; j<3; j++){
            m_R_fmu2camera[k][j] = R_camera_n2b[k][0]*R_fmu_n2b[j][0] + R_camera_n2b[k][1]*R_fmu_n2b[j][1] + R_camera_n2b[k][2]*R_fmu_n2b[j][2];
        }
    }
}

// array_new = R*array_origin;
void TurnlampDetector::Array3Rotation(const double R[3][3], const double array_origin[3], double array_new[3])
{
    // array_new = R*array_origin;
    for(int  k=0; k<3; k++)
        array_new[k] = R[k][0]*array_origin[0] + R[k][1]*array_origin[1] + R[k][2]*array_origin[2];
}


}
