#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"
#include "common/hal/halio.h"
#include "common/hal/esr.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace imu;
using namespace std;

double GetTimeS( );

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

    
// int32_t upgrade_can_filter(can_frame *cf)
// {
//     if (UPGRADE_MCU_CAN_ID == cf->id) {
//         return 0;
//     }
//     return -1;
// }

// 获取当前循环需要读取数据的时间
double GetTimeS( )
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    double timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
    return timestamp;
}

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";  
    #if defined(USE_GLOG)
        FLAGS_v = 0;
    #endif

    #if !defined(DATA_FROM_LOG)
        HalIO &halio = HalIO::Instance();
        string config_file_path = "mobileye.json"; //"./golf.json";
        HalioInitInfo hal_init_info;
        int err = LoadCANSignalConfig(config_file_path.c_str(), &hal_init_info);
        if (0 == err) {
            hal_init_info.enable_can_read_api = true;
//             hal_init_info.filter_cb = upgrade_can_filter;
            bool res = halio.Init(&hal_init_info);
            if (!res) {
                std::cerr << "HALIO init fail" << std::endl;
                exit(0);
            }
        } else {
            return -1;
        }
        
//         halio.EnableFMU(); // 使能CAN对应的filter 
        halio.EnableRadar();
    #endif
    // 进行数据融合的类
//     ImuModule::Instance().Init();

    can_frame m_can_raw_data;
    esr_full_track m_esr_radar_data;
    char buffer_data[8] = {1,2,3,4,5,6,7,8};
    while(1){
        // read radar quene
//         int time_cur = (int)GetTimeS()*1e3;
//         int res =  HalIO::Instance().ReadRadarData(&m_esr_radar_data, 1);
//         if(res > 0){
//             cout<<"radar data, ID: "<<m_esr_radar_data.id<<", range: "<<m_esr_radar_data.track.range<<endl;
//         }
//         usleep(100000);
        
        int time_cur = (int)GetTimeS()*1e3;
        int res = HalIO::Instance().read_can_frame(&m_can_raw_data, time_cur);
        if(res>=0){
            int res = get_esr_track(m_can_raw_data.id, (uchar *)&m_can_raw_data.data[0], &m_esr_radar_data);
            if(res >= 0){
                cout<<"radar data, ID: "<<m_esr_radar_data.id<<", range: "<<m_esr_radar_data.track.range<<endl;
            }
            cout<<"raw can data, id: "<<m_can_raw_data.id<<" data:";
            for(int i=0; i<8; i++)
                cout<<" "<<m_can_raw_data.data[i];
            cout<<endl;
        }
    }
    
    
    return 0;
}
