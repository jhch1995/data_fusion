#include "data_fusion/data_fusion.h"
#include "imu_module.h"

namespace imu {

ImuModule::ImuModule() {
    m_data_fusion = new DataFusion();
}

ImuModule::~ImuModule() {
    delete m_data_fusion;
    m_data_fusion = NULL;
}

void ImuModule::StartDataFusionTask() {
    m_data_fusion->StartDataFusionTask();
}

int ImuModule::GetTurnRadius(const int64 &timestamp_search, double *R) {
    return m_data_fusion->GetTurnRadius(timestamp_search / 1000, R);
}

int ImuModule::GetPredictFeature(const std::vector<cv::Point2f>& vector_feature_pre ,
        int64_t image_timestamp_pre,
        int64_t image_timestamp_cur,
        std::vector<cv::Point2f>* vector_feature_predict)
{
    return m_data_fusion->GetPredictFeature(vector_feature_pre,
            image_timestamp_pre,
            image_timestamp_cur,
            vector_feature_predict);
}

}
