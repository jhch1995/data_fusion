cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS " ANDORID platform")
    ADD_DEFINITIONS(-DANDROID)
    ADD_DEFINITIONS(-DUSE_GLOG)

    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
        "${PROJECT_ROOT_PATH}/data_fusion/src/data_fusion"
        "${PROJECT_ROOT_PATH}"
    )
    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/lib/${PLATFORM}/"
    )    
ELSEIF (UNIX)
    MESSAGE(STATUS " UNIX platform")
    ADD_DEFINITIONS(-DDATA_FROM_LOG)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-D__unix__) # time.h
    ADD_DEFINITIONS(-DLOG_STD_COUT) # LOG相关函数直接输出到直接输出到std::cout
    #ADD_DEFINITIONS(-DUSE_RADIUS) # 当前进行转弯半径的调试 #MESSAGE(${USE_RADIUS})

    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}"
        "${PROJECT_ROOT_PATH}/data_fusion/src/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/"	
        "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/"
    )
    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/"
        "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/"
    )
ENDIF()

SET(SRC_LIST data_fusion.cpp imu_attitude_estimate.cpp can_vehicle_estimate.cpp ) 
ADD_LIBRARY(data_fusion STATIC ${SRC_LIST})

IF (WIN32)
ELSEIF(ANDROID_ABI)
    TARGET_LINK_LIBRARIES(data_fusion halio miniglog gflags concurrency)
ELSEIF(UNIX)    
    TARGET_LINK_LIBRARIES(data_fusion halio glog gflags concurrency)
endif()	
