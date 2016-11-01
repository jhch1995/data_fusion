cmake_minimum_required(VERSION 3.0)

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/data_fusion/")

IF (WIN32)
ELSEIF(ANDROID_ABI)
    ADD_EXECUTABLE(data_fusion_test_main  "./data_fusion_test_main .cpp")
    TARGET_LINK_LIBRARIES(data_fusion_test_main  data_fusion halio miniglog gflags 
                          concurrency time  )

ELSE()
    
    ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
    ADD_DEPENDENCIES(lane_tracking_main MINIEYE_VERSION)
    TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion halio
                          concurrency relative_locate time glog gflags pthread
                          opencv_imgcodecs opencv_core opencv_highgui )

    ADD_EXECUTABLE(turn_radius_test_main "./turn_radius_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_test_main data_fusion halio
                          concurrency relative_locate time gflags glog 
                          opencv_imgcodecs opencv_core opencv_highgui pthread)
endif()
