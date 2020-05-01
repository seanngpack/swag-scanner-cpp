project(tests)

add_subdirectory(googletest-master)

# include utils headers
include_directories(${swag_scanner_SOURCE_DIR}/include/utils)

include_directories("/opt/librealsense/include" )
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(googletest-master/googlemock/include)
link_directories("/opt/librealsense/build/librealsense2.dylib")

add_executable(tests_run AlgosTests.cpp CameraTypesTests.cpp)
target_link_libraries(tests_run
        gtest
        gtest_main
        ${PCL_COMMON_LIBRARIES}
        /opt/librealsense/build/librealsense2.dylib)
