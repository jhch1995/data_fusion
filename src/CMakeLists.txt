cmake_minimum_required(VERSION 3.0)

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")



ADD_LIBRARY(relative_locate STATIC linear_r3.cpp relative_locate.cpp bird_perspective_mapping.cpp  )
ADD_LIBRARY(data_fusion STATIC imu_attitude_estimate.cpp can_vehicle_estimate.cpp)		

SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
MESSAGE(STATUS "${PROJECT_BINARY_DIR}")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")


INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")

MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/src/gflags/linux/include")
MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/src/gflags/linux/lib")

find_package(OpenCV REQUIRED)
#ADD_EXECUTABLE(rel_test "./relative_locate_test.cpp")
#ADD_EXECUTABLE(bird_perp_test "./bird_perspective_mapping_test.cpp")
ADD_EXECUTABLE(lane_tracking_main "./lan_tracking_main.cpp")

#TARGET_LINK_LIBRARIES(rel_test relative_locate opencv_imgcodecs opencv_core opencv_imgproc opencv_highgui)
#TARGET_LINK_LIBRARIES(bird_perp_test relative_locate gflags opencv_imgcodecs opencv_core opencv_highgui pthread) 
TARGET_LINK_LIBRARIES(lane_tracking_main data_fusion relative_locate gflags opencv_imgcodecs opencv_core opencv_highgui pthread)






