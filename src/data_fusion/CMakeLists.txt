cmake_minimum_required(VERSION 3.0)

SET(SRC_LIST imu_attitude_estimate.cpp can_vehicle_estimate.cpp data_fusion.cpp) 
ADD_LIBRARY(data_fusion STATIC ${SRC_LIST})

IF (WIN32)
ELSEIF(ANDROID_ABI)
    TARGET_LINK_LIBRARIES(data_fusion halio miniglog gflags concurrency)
ELSEIF(UNIX)    
    TARGET_LINK_LIBRARIES(data_fusion glog gflags concurrency)
endif()	




