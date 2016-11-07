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


IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
        "${PROJECT_ROOT_PATH}"
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
    )
    LINK_DIRECTORIES(
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
        "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/3rdparty/libs/arm64-v8a/"
    )
ELSEIF (UNIX)
    INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}")
ENDIF()

ADD_SUBDIRECTORY(common/time)
ADD_SUBDIRECTORY(common/base)
ADD_SUBDIRECTORY(common/system)
ADD_SUBDIRECTORY(common/string)
ADD_SUBDIRECTORY(common/relative_locate)
ADD_SUBDIRECTORY(common/concurrency)
ADD_SUBDIRECTORY(common/hal)
IF (ANDROID_ABI)
    ADD_SUBDIRECTORY(thirdparty/miniglog)
ENDIF()
ADD_SUBDIRECTORY(data_fusion)
ADD_SUBDIRECTORY(tests)








