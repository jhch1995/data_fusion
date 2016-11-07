#ifndef IMU_MODULE_H_
#define IMU_MODULE_H_

#include "common/base/stdint.h"


namespace imu {

class DataFusion;

class ImuModule {
public:
    ImuModule();

    ~ImuModule();

    void StartDataFusionTask();

    // timestamp_search in microseconds
    int GetTurnRadius(const int64 &timestamp_search, double *R);

private:
    DataFusion *m_data_fusion;
};

}

#endif
