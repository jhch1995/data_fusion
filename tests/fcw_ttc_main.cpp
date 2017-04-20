#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <map>

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include "datafusion_math.h"
#include "ttc_module/kalman_filter.h"

using namespace std;
using namespace Eigen;  

int main(int argc, char *argv[])
{
    //解析
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
//     google::SetStderrLogging(google::INFO); //设置级别高于 google::INFO 的日志同时输出到屏幕
    FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
    FLAGS_logbufsecs = 0;        //缓冲日志输出，默认为30秒，此处改为立即输出
    google::InstallFailureSignalHandler();      //捕捉 core dumped
     
    // 设置VLOG打印等级
    #if defined(USE_GLOG)
       FLAGS_v = VLOG_DEBUG;
//         FLAGS_v = 0;
    #endif
       
    // save results
    string save_result_addr = "./data/kf_result.ini";
    ofstream fid_save_result_out(save_result_addr.c_str(), ios::out);  
    
    // 读入log和vision
    string vision_data_addr = "./data/rec_20161125_035426.mp4.critical";
    string raw_log_addr = "./data/rec_20161125_035426.mp4.log-speed.ini";
    ifstream fid_vision_log(vision_data_addr.c_str(), ios::in);  
    ifstream fid_raw_log(raw_log_addr.c_str(), ios::in);  
    string buffer_vision_data, buffer_raw_log_data;
    stringstream ss_vision_data, ss_raw_log_data;
    int vision_data_index = -1;
    bool is_first_read_data = true;
    bool is_first_read_vision_data = true;
    bool is_first_read_log_data = true;
    double time_start = 0;
    
    // radar data
    double radar_timestamp_cur = 0; 
    double radar_vertial_dist = 0;
    double radar_vertial_dist_vel = 0;
    double radar_horiz_dist = 0;
    
    // vision data
    double image_timestamp_cur = 0; 
    double image_timestamp_pre = 0;
    int image_frame_id = 0;
    double vision_vertial_dist_raw = 0;
    double vision_vertial_dist_filter = 0;
    double vision_horiz_dist_raw = 0;
    double vision_horiz_dist_filter = 0;
    int target_car_id_cur = 0;
    int target_car_id_pre = 0;
    double ttc_raw =0; 
    
    // log data
    double log_timestamp_cur = 0;
    double log_timestamp_pre= 0;
    double speed_cur = 0;
    
    // KF
    // -- vertial state
    MatrixXd KF_P0_v(5,5);
    KF_P0_v<< 100, 0, 0, 0, 0,
            0, 100, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 20, 0,
            0, 0, 0, 0, 10;
    MatrixXd KF_Q_v(5,5);
    KF_Q_v<< 25, 0, 0, 0, 0,
            0, 20, 0, 0, 0,
            0, 0, 4, 0, 0,
            0, 0, 0, 3, 0,
            0, 0, 0, 0, 3;
    MatrixXd KF_R_v(2,2);
    KF_R_v<< 10, 0, 
            0, 1;
    MatrixXd KF_Xk_v(5,1);
    KF_Xk_v.setZero();    
    MatrixXd KF_Pk_v(5,5);
    KF_Pk_v = KF_P0_v;
    
    MatrixXd KF_F_v(5,5);
    KF_F_v.setZero();
    for(int i = 0; i<5; i++)
        KF_F_v(i, i) = 1;
    
    MatrixXd KF_H_v(2,5);
    KF_H_v.setZero();
    KF_H_v(0, 0) = 1;
    KF_H_v(1, 2) = 1;
    
    // -- horizion state
    MatrixXd KF_P0_h(3,3);
    KF_P0_h <<  5, 0, 0,
                0, 4, 0,
                0, 0, 4;
    MatrixXd KF_Q_h(3, 3);
    KF_Q_h<<1, 0, 0,
            0, 1, 0,
            0, 0, 0.5;
    MatrixXd KF_R_h(1,1);
    KF_R_h<< 0.5;
    MatrixXd KF_Xk_h(3,1);
    KF_Xk_h.setZero();  
    MatrixXd KF_Pk_h(3,3);
    KF_Pk_h = KF_P0_h;
    
    MatrixXd KF_F_h(3,3);
    KF_F_h.setZero();
    for(int i = 0; i<3; i++)
        KF_F_h(i, i) = 1;
    
    MatrixXd KF_H_h(1,3);
    KF_H_h.setZero();
    KF_H_h(0, 0) = 1;
    
    bool is_KF_need_reset = true;
    double dt_image = 0;
    
    // ttc
    double ttc_est = 0;
    
    // kf class
    ttc::KalmanFilter m_ttc_kf_v;
    ttc::KalmanFilter m_ttc_kf_h;
    
    // kf map
    map<int, ttc::KalmanFilter> m_kf_v_dict;
    
    int mobileye_fcw_state = 0;
    while(fid_vision_log.is_open() && fid_raw_log.is_open() && !fid_vision_log.eof() && !fid_raw_log.eof() ){
        // 1.1. read log data
        getline(fid_vision_log, buffer_vision_data);        
        ss_vision_data.clear();
        ss_vision_data.str(buffer_vision_data);
        ss_vision_data>>vision_data_index;
        
        // if the first time read data, so record the time_start
        if(is_first_read_data){
            double time_t1, time_t2;
            // 1. first time from log
            getline(fid_raw_log, buffer_raw_log_data);  
            istringstream lines(buffer_raw_log_data);
            string data_tmp[4];
            for(int i=0; i<3; i++)
                getline(lines, data_tmp[i], ' ');
            time_t1 = atof(data_tmp[0].c_str()) + atof(data_tmp[1].c_str())*1e-6;
            
            // 3. chose the min time as the time_start
            time_start = time_t1;// min(time_t1, time_t2);
            is_first_read_data = false;
            VLOG(VLOG_DEBUG)<<"first read data, set time_start: "<< time_start<< endl;
        }
        
        if(vision_data_index == 2){
            // mobileye fcw warning
            mobileye_fcw_state = 1;
        }else if(vision_data_index == 1){
            // radar data
            istringstream lines(buffer_vision_data);
            string data_tmp[22];
            int variable_index = 0;
            while(getline(lines, data_tmp[variable_index], ','))            
                variable_index++;
            
            radar_timestamp_cur = atof(data_tmp[1].c_str()) - time_start;
            speed_cur = atof(data_tmp[21].c_str());
            radar_vertial_dist = atof(data_tmp[20].c_str());
            radar_vertial_dist_vel = atof(data_tmp[17].c_str());
            radar_horiz_dist = atof(data_tmp[19].c_str());
            
        }else if(vision_data_index == 0){
            // vision data
            istringstream lines(buffer_vision_data);
            string data_tmp[22];
            int variable_index = 0;
            while(getline(lines, data_tmp[variable_index], ','))            
                variable_index++;
            
            image_timestamp_cur =  atof(data_tmp[1].c_str()) - time_start;
            image_frame_id =  atof(data_tmp[5].c_str());
            vision_vertial_dist_raw =  atof(data_tmp[11].c_str());
            vision_horiz_dist_raw =  atof(data_tmp[12].c_str());
            target_car_id_cur =  atof(data_tmp[3].c_str());
            ttc_raw =  atof(data_tmp[15].c_str());
            
            if(is_first_read_vision_data){
                image_timestamp_pre = image_timestamp_cur;
                // init lowpass filter
                vision_vertial_dist_filter = vision_vertial_dist_raw;
                vision_horiz_dist_filter = vision_horiz_dist_raw;     
                target_car_id_pre = target_car_id_cur;
                is_first_read_vision_data = false;
                VLOG(VLOG_DEBUG)<<"first read vision data"<<endl;
            }
            
            // 1.2 match vision & log_speed timestamp
            bool is_log_match_vision_timestamp = false;
            while(!is_log_match_vision_timestamp && !fid_raw_log.eof()){
                // first read log data, try to read the first line data
                if(is_first_read_log_data){
                    getline(fid_raw_log, buffer_raw_log_data);  
                    istringstream lines(buffer_raw_log_data);
                    string data_tmp[4];
                    for(int i=0; i<4; i++)
                        getline(lines, data_tmp[i], ' ');
                    if(data_tmp[2] == "speed"){
                        log_timestamp_cur = atof(data_tmp[0].c_str()) + atof(data_tmp[1].c_str()) - time_start;
                        log_timestamp_pre = log_timestamp_cur;
                        speed_cur = atof(data_tmp[3].c_str())/3.6;
                        is_first_read_log_data = false;    
                        VLOG(VLOG_DEBUG)<<"first read speed data: "<< speed_cur<<endl;
                    }
                }
                
                // 判断上一个速度时间戳是否匹配上（因为speed更新比较慢，所以不能每次进来比较就先读取数据，很可能会一直匹配不上）
                double dt_log_vision = log_timestamp_pre - image_timestamp_cur;
                if( dt_log_vision > 0.1){
                    is_log_match_vision_timestamp = true;
                    VLOG(VLOG_DEBUG)<<"new speed: "<<speed_cur<<endl;
                }else{
                    istringstream lines(buffer_vision_data);
                    string data_tmp[4];
                    for(int i=0; i<4; i++)
                        getline(lines, data_tmp[i], ',');
                    if(data_tmp[2] == "speed"){
                        log_timestamp_cur = atof(data_tmp[0].c_str()) + atof(data_tmp[1].c_str())*1e-6 - time_start;
                        log_timestamp_pre = log_timestamp_cur;
                        speed_cur = atof(data_tmp[3].c_str())/3.6;
                    }
                }
            }
            
            // 2. KF
            // 2.1 --量测数据预处理 
            // 判断 target ID，来确认是否是一辆新的车
            bool is_new_target_car_id = false;
            dt_image = image_timestamp_cur - image_timestamp_pre;
            image_timestamp_pre = image_timestamp_cur;
            // 对于新目标，也要重置滤波 || 为了处理长时间没有有效前车的问题 dt太大则重置滤波
            if(is_KF_need_reset || target_car_id_cur != target_car_id_pre || dt_image > 0.3){
                 // reset filter
                vision_vertial_dist_filter = vision_vertial_dist_raw;
                vision_horiz_dist_filter = vision_horiz_dist_raw;
                
                // Xk_v = [vision_vertial_dist_filter, speed_t, speed_s, acc_t, acc_s]
                KF_Xk_v << vision_vertial_dist_filter, speed_cur ,speed_cur, 0 , 0;
                KF_Pk_v = KF_P0_v;
                
                // if it is a new car id, so add a new kf
                m_kf_v_dict[target_car_id_cur] = ttc::KalmanFilter();
                m_kf_v_dict[target_car_id_cur].Init(KF_Xk_v, KF_Pk_v, KF_Q_v, KF_R_v); // init kf
                
                // horizion state = [horizon vel acc]
                KF_Xk_h << vision_horiz_dist_filter, 0 ,0;
                KF_Pk_h = KF_P0_h;
                m_ttc_kf_h.Init(KF_Xk_h, KF_Pk_h, KF_Q_h, KF_R_h); // init kf
                
                is_new_target_car_id = true;
                is_KF_need_reset = false;
                target_car_id_pre = target_car_id_cur;      
                VLOG(VLOG_DEBUG)<<"new car id: "<<target_car_id_cur<<endl;   
                VLOG(VLOG_DEBUG)<<"reset kf state, dt_image: "<<dt_image<<endl;
            }else{
                // 2.2 --KF
                // measure lowpass filter            
                ttc::KalmanFilter::LowpassFilter1D(vision_vertial_dist_filter, vision_vertial_dist_raw, dt_image, 0.5, vision_vertial_dist_filter);
                ttc::KalmanFilter::LowpassFilter1D(vision_horiz_dist_filter, vision_horiz_dist_raw, dt_image, 0.2, vision_horiz_dist_filter);
                
                VLOG(VLOG_DEBUG)<<"vertial dist raw: "<<vision_vertial_dist_raw<<endl
                                <<"vertial dist filter: "<< vision_vertial_dist_filter<<endl;
                //A.  vertial dist KF
                MatrixXd Z_v(2, 1);
                Z_v<<vision_vertial_dist_filter, speed_cur;
                // set F
                KF_F_v(0,1) = dt_image;
                KF_F_v(0,2) = -dt_image;
                KF_F_v(0,3) = 0.5*dt_image*dt_image;
                KF_F_v(0,4) = -0.5*dt_image*dt_image;
                KF_F_v(1,3) = dt_image;
                KF_F_v(2,4) = dt_image;
                
//                 kf::KalmanFilter::kf_update(KF_Xk_v, KF_Pk_v, KF_Q_v, KF_R_v, KF_F_v, KF_H_v, Z_v, KF_Xk_v, KF_Pk_v);
                m_kf_v_dict[target_car_id_cur].KfUpdate(KF_F_v, KF_H_v, Z_v);
                m_kf_v_dict[target_car_id_cur].GetXk(KF_Xk_v);
                VLOG(VLOG_DEBUG)<<"KF_F_v: "<<endl<<KF_F_v<<endl;
                VLOG(VLOG_DEBUG)<<"KF_H_v: "<<endl<<KF_H_v<<endl;
                VLOG(VLOG_DEBUG)<<"kf measure: "<<endl<<Z_v<<endl;
                VLOG(VLOG_DEBUG)<<"kf state: "<<endl<<KF_Xk_v<<endl;
                
                //B.  horiz dist KF
                MatrixXd Z_h(1, 1);
                Z_h<<vision_horiz_dist_filter;
                // set F
                KF_F_h(0,1) = dt_image;
                KF_F_h(0,2) = 0.5*dt_image*dt_image;
                KF_F_h(1,2) = dt_image;                
//                 kf::KalmanFilter::kf_update(KF_Xk_h, KF_Pk_h, KF_Q_h, KF_R_h, KF_F_h, KF_H_h, Z_h, KF_Xk_h, KF_Pk_h);
                m_ttc_kf_h.KfUpdate(KF_F_h, KF_H_h, Z_h);
                m_ttc_kf_h.GetXk(KF_Xk_h);
                
                // --2.3 ttc calculate
                double vertial_dist_est = KF_Xk_v(0, 0);
                double Vt = KF_Xk_v(1, 0);
                double Vs = KF_Xk_v(2, 0);
                double acc_t = KF_Xk_v(3, 0);
                double acc_s = KF_Xk_v(4, 0);
                double vertial_vel_relative = Vt - Vs;
                double vertial_acc_relative = acc_t - acc_s;
                if(fabs(vertial_vel_relative) > 0.2)
                    ttc_est = fabs(vertial_dist_est/vertial_vel_relative);
                else
                    ttc_est = -1;
                
                VLOG(VLOG_DEBUG)<<"vertial_dist_est: "<<vertial_dist_est<<endl;
                VLOG(VLOG_DEBUG)<<"vertial_vel_relative: "<<vertial_vel_relative<<endl;
                VLOG(VLOG_DEBUG)<<"ttc_est: "<<ttc_est<<endl;

                // reset variable
                mobileye_fcw_state = 0;
                
                // save data
                if(fid_save_result_out){
                    char buffer_save[100];
                    sprintf(buffer_save, "%f %f", image_timestamp_cur, ttc_est);
                    fid_save_result_out<<buffer_save<<endl;
                }
            }
        }
    }
    
    fid_vision_log.close();
    fid_raw_log.close();
    fid_save_result_out.close();
    cout<<"exe over!"<<endl;
    return 0;
}



