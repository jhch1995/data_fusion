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

}
