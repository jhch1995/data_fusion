cmake_minimum_required(VERSION 3.0)

ADD_LIBRARY(data_fusion STATIC imu_attitude_estimate.cpp can_vehicle_estimate.cpp data_fusion.cpp)		




