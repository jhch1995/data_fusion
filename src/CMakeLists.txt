cmake_minimum_required(VERSION 3.0)

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/src/relative_local")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")

ADD_SUBDIRECTORY(relative_local)
#ADD_LIBRARY(relative_locate STATIC linear_r3.cpp relative_locate.cpp bird_perspective_mapping.cpp  )
ADD_LIBRARY(data_fusion STATIC imu_attitude_estimate.cpp can_vehicle_estimate.cpp)		

find_package(OpenCV REQUIRED)
ADD_EXECUTABLE(lane_tracking_main "./lan_tracking_main.cpp")
TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion relative_locate gflags opencv_imgcodecs opencv_core opencv_highgui pthread)

#ADD_EXECUTABLE(rel_test "./relative_locate_test.cpp")
#ADD_EXECUTABLE(bird_perp_test "./bird_perspective_mapping_test.cpp")

#TARGET_LINK_LIBRARIES(rel_test relative_locate opencv_imgcodecs opencv_core opencv_imgproc opencv_highgui)
#TARGET_LINK_LIBRARIES(bird_perp_test relative_locate gflags opencv_imgcodecs opencv_core opencv_highgui pthread) 







