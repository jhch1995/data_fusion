cmake_minimum_required(VERSION 3.0)

ADD_DEFINITIONS(-D__unix__) # time.h

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/data_fusion/")

IF (WIN32)
ELSEIF(ANDROID_ABI)
    ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
    ADD_DEPENDENCIES(lane_tracking_main MINIEYE_VERSION)
    TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion halio glog gflags pthread
                          concurrency relative_locate time  
                          opencv_imgcodecs opencv_core opencv_highgui )

ELSE()
    
    ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
    ADD_DEPENDENCIES(lane_tracking_main MINIEYE_VERSION)
    TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion halio
                          concurrency relative_locate time  glog gflags
                          opencv_imgcodecs opencv_core opencv_highgui pthread)

    ADD_EXECUTABLE(turn_radius_test_main "./turn_radius_test_main.cpp")
    TARGET_LINK_LIBRARIES(turn_radius_test_main data_fusion halio
                          concurrency relative_locate time gflags glog 
                          opencv_imgcodecs opencv_core opencv_highgui pthread)
endif()
