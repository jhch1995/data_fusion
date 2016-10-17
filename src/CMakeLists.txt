cmake_minimum_required(VERSION 3.0)

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
ADD_DEFINITIONS(-DUSE_GLOG)

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/include")
INCLUDE_DIRECTORIES("./modules/data_fusion/")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/static/x64")	

find_package(OpenCV REQUIRED)
ADD_SUBDIRECTORY(./modules/data_fusion)

ADD_EXECUTABLE(lane_tracking_main "./lan_tracking_main.cpp")
TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion relative_locate gflags glog opencv_imgcodecs opencv_core opencv_highgui pthread)



