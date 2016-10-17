cmake_minimum_required(VERSION 3.0)

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

ADD_LIBRARY(relative_locate STATIC linear_r3.cpp relative_locate.cpp bird_perspective_mapping.cpp  )

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/include")	
LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/thirdparty/gflags/linux/lib/x64")
find_package(OpenCV REQUIRED)








