ADD_EXECUTABLE(gflags_test "./gflags_test.cpp")
IF(ANDROID_ABI)
    TARGET_LINK_LIBRARIES(gflags_test gflags
        log
        )
ELSEIF (UNIX)
    TARGET_LINK_LIBRARIES(gflags_test gflags
        pthread
        )
ENDIF()
