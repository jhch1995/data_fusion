cmake_minimum_required(VERSION 3.0)
PROJECT (DATA_FUSION) # PROJECT_SOURCE_DIR
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

SET(DEBUG 1)
IF(DEBUG)
    SET(CMAKE_BUILD_TYPE "Debug") 
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
ELSEIF (RELEASE)
ENDIF()

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

#root
IF ( "${CMAKE_SIZEOF_VOID_P}" EQUAL "4") 
    MESSAGE(STATUS "root: x32 platform")
    SET(PLATFORM x86)
ELSEIF( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8") 
    MESSAGE(STATUS "root: x64 platform")
    SET(PLATFORM x64)
ENDIF()  

#DEFINE
IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "DEFINE: ANDORID platform")
    ADD_DEFINITIONS(-DANDROID)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-DUSE_CAMERA)
    ADD_DEFINITIONS(-DANDROID_CAM_RECORD)
    SET(USE_CAN 1)
    ADD_CUSTOM_TARGET(MINIEYE_VERSION COMMAND ${SCRIPTS_PATH}/version.sh |tee ${PROJECT_ROOT_PATH}/version.h.in)
    
ELSEIF (UNIX)
    MESSAGE(STATUS "DEFINE: UNIX platform")
    ADD_DEFINITIONS(-DDATA_FROM_LOG)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-D__unix__) # time.h
    #ADD_DEFINITIONS(-DLOG_STD_COUT) # LOG相关函数直接输出到直接输出到std::cout
    ADD_DEFINITIONS(-DUSE_RADIUS) # 当前进行转弯半径的调试 #MESSAGE(${USE_RADIUS})
ENDIF()


#include
INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}")
IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
        "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/include/blis"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
        "${PROJECT_ROOT_PATH}/thirdparty/libsvm/android/include/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
    )

    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/lane_detect/lib/android/${PLATFORM}"
        "${PROJECT_ROOT_PATH}/vehicle_detect/lib/android/${PLATFORM}"
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/3rdparty/libs/arm64-v8a/"
        "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/lib/${PLATFORM}/"
        "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/lib"
        "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
        "${PROJECT_ROOT_PATH}/thirdparty/libsvm/android/lib/${PLATFORM}/"
    )

ELSEIF (UNIX)
    MESSAGE(STATUS "include: UNIX platform")
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}")
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/")	
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/")

    LINK_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/")
    LINK_DIRECTORIES("${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/")	



ENDIF()

ADD_SUBDIRECTORY(common/time)
ADD_SUBDIRECTORY(common/base)
ADD_SUBDIRECTORY(common/system)
ADD_SUBDIRECTORY(common/string)
ADD_SUBDIRECTORY(common/relative_locate)
ADD_SUBDIRECTORY(common/concurrency)
ADD_SUBDIRECTORY(common/hal)
ADD_SUBDIRECTORY(data_fusion)
ADD_SUBDIRECTORY(tests)

IF (ANDROID_ABI)
    ADD_SUBDIRECTORY(thirdparty/miniglog)
ENDIF()






