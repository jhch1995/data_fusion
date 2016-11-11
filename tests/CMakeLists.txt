cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
        "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/include/blis"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
        "${PROJECT_SOURCE_DIR}/src/data_fusion/"
    )
    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/3rdparty/libs/arm64-v8a/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/lib/${PLATFORM}/"
        "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/lib"
        "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
    )
ELSEIF (UNIX)
    MESSAGE(STATUS "include: UNIX platform")
    ADD_DEFINITIONS(-DUSE_GLOG)
    INCLUDE_DIRECTORIES(
        #"${PROJECT_ROOT_PATH}"
        "${PROJECT_SOURCE_DIR}/src/data_fusion/"
        "${PROJECT_SOURCE_DIR}/src/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/"	
        "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/"
    )
    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/"
        "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/"
    )
ENDIF()

IF (WIN32)
ELSEIF(ANDROID_ABI)
    MESSAGE(STATUS "LINK: ANDROID")
    ADD_EXECUTABLE(imu_calibrate_main  "./imu_calibrate_main.cpp")
    TARGET_LINK_LIBRARIES(imu_calibrate_main data_fusion miniglog gflags concurrency time  )

ELSEIF (UNIX) 
    MESSAGE(STATUS "LINK: UNIX") 
    find_package(OpenCV REQUIRED)

    ADD_EXECUTABLE(lane_tracking_test_main "./lane_tracking_test_main.cpp")
    TARGET_LINK_LIBRARIES(lane_tracking_test_main imu halio
                          concurrency relative_locate time glog gflags pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turn_radius_ipm_test_main "./turn_radius_ipm_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_ipm_test_main data_fusion halio
                          concurrency relative_locate time gflags glog 
                          opencv_imgcodecs opencv_core opencv_highgui pthread)

    ADD_EXECUTABLE(turn_radius_calculate_test_main "./turn_radius_calculate_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_calculate_test_main data_fusion imu halio
                          gflags glog  pthread 
                          opencv_imgcodecs opencv_core opencv_highgui )                          
ENDIF()
