INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/src/data_fusion/")

IF (WIN32)
ELSEIF(ANDROID_ABI)
    ADD_EXECUTABLE(data_fusion_test_main  "./data_fusion_test_main.cpp")
    TARGET_LINK_LIBRARIES(data_fusion_test_main imu miniglog gflags concurrency time  )

ELSE()
    ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
    TARGET_LINK_LIBRARIES(lane_tracking_main imu halio
                          concurrency relative_locate time glog gflags pthread
                          ${OpenCV_LIBS} )

    ADD_EXECUTABLE(turn_radius_test_main "./turn_radius_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_test_main imu halio
                          concurrency relative_locate time gflags glog 
                          ${OpenCV_LIBS} pthread)

    ADD_EXECUTABLE(data_fusion_test_main  "./data_fusion_test_main.cpp")
    TARGET_LINK_LIBRARIES(data_fusion_test_main imu halio
                          concurrency relative_locate time gflags glog 
                          ${OpenCV_LIBS} pthread  )
endif()
