cmake_minimum_required(VERSION 3.0)

ADD_DEFINITIONS(-D__unix__) # time.h

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/include")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/data_fusion/")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/static/x64")	

find_package(OpenCV REQUIRED)

ADD_EXECUTABLE(lane_tracking_main "./lane_tracking_main.cpp")
TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion relative_locate time gflags glog opencv_imgcodecs opencv_core opencv_highgui pthread)


