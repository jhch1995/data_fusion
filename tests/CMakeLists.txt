cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

#include
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/src/data_fusion/")
IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
        "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/include/blis"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
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
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}")
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/")	
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/")

    LINK_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/")
    LINK_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/")

ENDIF()

IF (MSVC)
ELSEIF(ANDROID_ABI)
    MESSAGE(STATUS "LINK: ANDROID")
    ADD_EXECUTABLE(imu_calibrate_main  "./imu_calibrate_main.cpp")
    TARGET_LINK_LIBRARIES(data_fusion_test_main imu miniglog gflags concurrency time  )

ELSEIF (UNIX) 
    MESSAGE(STATUS "LINK: UNIX") 
    find_package(OpenCV REQUIRED)

    ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
    TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion halio
                          concurrency relative_locate time glog gflags pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turn_radius_main "./turn_radius_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_main data_fusion halio
                          concurrency relative_locate time gflags glog 
                          opencv_imgcodecs opencv_core opencv_highgui pthread)

    ADD_EXECUTABLE(data_fusion_test_main  "./data_fusion_test_main.cpp")
    TARGET_LINK_LIBRARIES(data_fusion_test_main imu halio
                          gflags glog opencv_core  pthread  )
                          opencv_imgcodecs opencv_core opencv_highgui pthread  )
ENDIF()
