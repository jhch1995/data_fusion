IF (WIN32)
ELSEIF(ANDROID_ABI)
    MESSAGE(STATUS "LINK: ANDROID")
    ADD_EXECUTABLE(imu_calibrate  "./imu_calibrate_main.cpp")
    TARGET_LINK_LIBRARIES(imu_calibrate -Wl,--start-group
            data_fusion halio
            ${ARM_LIBS}
            gflags
            -Wl,--end-group
            )

ELSEIF (UNIX) 
    MESSAGE(STATUS "LINK: UNIX")
    ADD_EXECUTABLE(lane_tracking_test "./lane_tracking_test_main.cpp")
    TARGET_LINK_LIBRARIES(lane_tracking_test imu halio
                          concurrency relative_locate time glog gflags pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turn_radius_ipm_test "./turn_radius_ipm_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_ipm_test imu halio
                          concurrency relative_locate time gflags glog 
                          opencv_imgcodecs opencv_core opencv_highgui pthread)

    ADD_EXECUTABLE(turn_radius_calculate_test "./turn_radius_calculate_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_calculate_test imu halio
                          gflags glog  pthread 
                          opencv_imgcodecs opencv_core opencv_highgui )                          
ENDIF()
