cmake_minimum_required(VERSION 2.8.2)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

######################### configuration ###########################
OPTION(READ_IMU_DATA_FROM_MURATA "switch for read imu data from murata" OFF)
IF (READ_IMU_DATA_FROM_MURATA)
    MESSAGE(STATUS "read imu data from murata")
    ADD_DEFINITIONS(-DDATA_FROM_MURATA) # read data from murata
ENDIF()

IF (WIN32)
ELSEIF(ANDROID_ABI)

    MESSAGE(STATUS "LINK: ANDROID")
    ADD_EXECUTABLE(imu_calibrate  "./imu_calibrate_main.cpp")
    TARGET_LINK_LIBRARIES(imu_calibrate
            -Wl,--start-group
            data_fusion halio
            ${ARM_LIBS}
            gflags
            -Wl,--end-group
    )
    ADD_EXECUTABLE(imu_calibrate_online  "./imu_calibrate_online.cpp")
    TARGET_LINK_LIBRARIES(imu_calibrate_online
            -Wl,--start-group
            imu halio
            ${ARM_LIBS}
            gflags
            -Wl,--end-group
    )
    ADD_EXECUTABLE(turn_radius_andriod  "./turn_radius_andriod.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_andriod
            -Wl,--start-group
            imu halio
            ${ARM_LIBS}
            gflags
            -Wl,--end-group
    )
ADD_EXECUTABLE(turnlamp_detection "./turnlamp_detection_main.cpp")
    TARGET_LINK_LIBRARIES(turnlamp_detection  
                          -Wl,--start-group
                          data_fusion halio
                          gflags   
                          ${ARM_LIBS}
                         -Wl,--end-group
                             )

ADD_EXECUTABLE(eigen_test  "./eigen_test.cpp")
    TARGET_LINK_LIBRARIES(eigen_test
            -Wl,--start-group
            data_fusion
            ${EIGEN3_LIBS}
            ${ARM_LIBS}
            -Wl,--end-group
    )


ELSEIF (UNIX)
    MESSAGE(STATUS "LINK: UNIX")
    ADD_EXECUTABLE(lane_tracking "./lane_tracking_test_main.cpp")
    TARGET_LINK_LIBRARIES(lane_tracking imu  data_fusion halio
                          concurrency relative_locate time glog gflags pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turn_radius_ipm "./turn_radius_ipm_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_ipm data_fusion imu halio
                          concurrency relative_locate time gflags glog
                          opencv_imgcodecs opencv_core opencv_highgui pthread)

    ADD_EXECUTABLE(turn_radius_calculate "./turn_radius_calculate_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_calculate imu data_fusion halio
                          gflags glog  pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(imu_calibrate_online "./imu_calibrate_online.cpp")
    TARGET_LINK_LIBRARIES(imu_calibrate_online imu data_fusion halio
                          gflags glog  pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turnlamp_detection "./turnlamp_detection_main.cpp")
    TARGET_LINK_LIBRARIES(turnlamp_detection  data_fusion halio
                          gflags glog  pthread
                          opencv_imgcodecs opencv_core opencv_highgui
                             )

#    ADD_EXECUTABLE(producer_consumer "./proudcer_consumer_test_main.cpp")
#    TARGET_LINK_LIBRARIES(producer_consumer data_fusion
#                             pthread concurrency )
    ADD_EXECUTABLE(eigen_test  "./eigen_test.cpp")
    TARGET_LINK_LIBRARIES(eigen_test
            -Wl,--start-group
            data_fusion
            ${EIGEN3_LIBS}
            -Wl,--end-group
    )
ENDIF()
