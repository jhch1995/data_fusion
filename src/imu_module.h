#ifndef IMU_MODULE_H_
#define IMU_MODULE_H_

#include "common/base/singleton.h"
#include "common/base/stdint.h"
#include "data_fusion/data_fusion.h"

namespace imu {

class DataFusion;

class ImuModule : public SingletonBase<ImuModule> {
public:
    friend class SingletonBase<ImuModule>;

    ~ImuModule();

    void StartDataFusionTask();

    // timestamp_search in microseconds
    int GetTurnRadius(const int64 &timestamp_search, double *R);

    int GetPredictFeature(const std::vector<cv::Point2f>& vector_feature_pre ,
            int64_t image_timestamp_pre,
            int64_t image_timestamp_cur,
            std::vector<cv::Point2f>* vector_feature_predict);

private:
    ImuModule();

private:
    DataFusion *m_data_fusion;
};

}

#endif
