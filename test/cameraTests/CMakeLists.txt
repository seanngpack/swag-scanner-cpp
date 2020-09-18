#if (BUILD_TESTS_VISUAL)
#    add_subdirectory(visual)
#endif()

target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CameraTests.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})