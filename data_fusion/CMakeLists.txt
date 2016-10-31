cmake_minimum_required(VERSION 3.0)

SET(SRC_LIST imu_attitude_estimate.cpp can_vehicle_estimate.cpp data_fusion.cpp) 
ADD_LIBRARY(data_fusion STATIC ${SRC_LIST})

	




