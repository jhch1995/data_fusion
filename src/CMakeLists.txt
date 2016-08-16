cmake_minimum_required(VERSION 3.0)

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

ADD_LIBRARY(relative_locate STATIC linear_r3.cpp relative_locate.cpp bird_perspective_mapping.cpp imu_attitude_estimate.cpp )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/src/gflags/linux/include")	
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/src/gflags/linux/lib/x64")

MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/src/gflags/linux/include")
MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/src/gflags/linux/lib")

find_package(OpenCV REQUIRED)
#ADD_EXECUTABLE(rel_test "./relative_locate_test.cpp")
#ADD_EXECUTABLE(bird_perp_test "./bird_perspective_mapping_test.cpp")
ADD_EXECUTABLE(lan_tracking_main "./lan_tracking_main.cpp")

#TARGET_LINK_LIBRARIES(rel_test relative_locate opencv_imgcodecs opencv_core opencv_imgproc opencv_highgui)
#TARGET_LINK_LIBRARIES(bird_perp_test relative_locate gflags opencv_imgcodecs opencv_core opencv_highgui pthread) 
TARGET_LINK_LIBRARIES(lan_tracking_main relative_locate opencv_imgcodecs opencv_core opencv_imgproc opencv_highgui)