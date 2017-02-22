#ifndef IMU_MODULE_H_
#define IMU_MODULE_H_

#include <vector>
#include "common/base/singleton.h"
#include "common/base/stdint.h"

namespace imu {

class DataFusion;

class ImuModule : public SingletonBase<ImuModule> {
public:
    friend class SingletonBase<ImuModule>;

    ~ImuModule();

    void StartDataFusionTask();

    // timestamp_search in microseconds
    int GetTurnRadius(const int64_t &timestamp_search, double *R);

    int GetPredictFeature(const std::vector<cv::Point2f>& vector_feature_pre ,
            int64_t image_timestamp_pre,
            int64_t image_timestamp_cur,
            std::vector<cv::Point2f>* vector_feature_predict);
    
    int GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z, double att_gyro[3], double acc[3], double gyro[3] );

private:
    ImuModule();

private:
    DataFusion *m_data_fusion;
};

}

#endif
