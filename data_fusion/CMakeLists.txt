cmake_minimum_required(VERSION 3.0)

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/include")
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/glog/linux/static/x64")	


ADD_LIBRARY(data_fusion STATIC imu_attitude_estimate.cpp can_vehicle_estimate.cpp data_fusion.cpp ) 		




