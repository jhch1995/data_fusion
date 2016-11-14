cmake_minimum_required(VERSION 2.8.2)
PROJECT(DATA_FUSION) # PROJECT_ROOT_PATH
#cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

IF (ANDROID_ABI)
    SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -O3 -std=gnu++0x")
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -g -std=gnu++0x")
ELSEIF(UNIX)
    SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -O3 -std=gnu++0x")
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -g -std=gnu++0x")
ENDIF ()

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

#root
IF ("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
    MESSAGE(STATUS "root: x32 platform")
    SET(PLATFORM x86)
ELSEIF ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    MESSAGE(STATUS "root: x64 platform")
    SET(PLATFORM x64)
ENDIF ()

IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "DEFINE: ANDORID platform")
    ADD_DEFINITIONS(-DANDROID)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-DUSE_RADIUS) # 当前进行转弯半径的调试 #MESSAGE(${USE_RADIUS})
    ADD_CUSTOM_TARGET(MINIEYE_VERSION COMMAND ${SCRIPTS_PATH}/version.sh |tee ${PROJECT_ROOT_PATH}/version.h.in)
ELSEIF (UNIX)
    MESSAGE(STATUS "DEFINE: UNIX platform")
    SET(OpenCV_DIR "/opt/opencv/default/share/OpenCV")
    FIND_PACKAGE(OpenCV REQUIRED)
    ADD_DEFINITIONS(-DDATA_FROM_LOG)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-DUSE_GSENSOR_LOG)
ENDIF ()

IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
            "${PROJECT_ROOT_PATH}"
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
            "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/include/blis"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
            "${PROJECT_ROOT_PATH}/src/"
            "${PROJECT_ROOT_PATH}/src/data_fusion/"
            "${PROJECT_ROOT_PATH}"
    )
    LINK_DIRECTORIES(
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/3rdparty/libs/arm64-v8a/"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/lib/${PLATFORM}/"
            "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/lib"
            "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
    )
    SET(ARM_LIBS relative_locate base time concurrency opencv_core opencv_imgproc opencv_hal opencv_highgui opencv_video opencv_videoio opencv_imgcodecs libpng libtiff libjpeg libjasper libwebp IlmImf c z log miniglog blis)
    SET(OpenCV_LIBS opencv_core opencv_imgproc opencv_hal opencv_highgui opencv_video opencv_videoio opencv_imgcodecs libpng libtiff libjpeg libjasper libwebp IlmImf c z log miniglog blis)
ELSEIF (UNIX)
    MESSAGE(STATUS "include: UNIX platform")
    INCLUDE_DIRECTORIES(
            "${OpenCV_INSTALL_PATH}/include"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/"
            "${PROJECT_ROOT_PATH}/src/"
            "${PROJECT_ROOT_PATH}/src/data_fusion/"
            "${PROJECT_ROOT_PATH}"
    )
    LINK_DIRECTORIES(
            "${OpenCV_INSTALL_PATH}/lib"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/"
            "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/"
    )
ENDIF ()

ADD_SUBDIRECTORY(common/time)
ADD_SUBDIRECTORY(common/base)
ADD_SUBDIRECTORY(common/system)
ADD_SUBDIRECTORY(common/string)
ADD_SUBDIRECTORY(common/relative_locate)
ADD_SUBDIRECTORY(common/concurrency)
ADD_SUBDIRECTORY(common/hal)
IF (ANDROID_ABI)
    ADD_SUBDIRECTORY(thirdparty/miniglog)
ENDIF ()

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)
