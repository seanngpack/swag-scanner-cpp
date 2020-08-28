cmake_minimum_required(VERSION 3.17)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(TEST_MAIN unit_tests)

find_package(PCL 1.11 REQUIRED)
find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(feeling-blue REQUIRED)
find_package(realsense2 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${swag_scanner_SOURCE_DIR}/src
        ${swag_scanner_SOURCE_DIR}/src/cli
        ${swag_scanner_SOURCE_DIR}/src/controller
        ${swag_scanner_SOURCE_DIR}/src/file
        ${swag_scanner_SOURCE_DIR}/src/model/arduino
        ${swag_scanner_SOURCE_DIR}/src/model/calibration
        ${swag_scanner_SOURCE_DIR}/src/model/camera
        ${swag_scanner_SOURCE_DIR}/src/model/equations
        ${swag_scanner_SOURCE_DIR}/src/model/processing
        ${swag_scanner_SOURCE_DIR}/src/types
        ${swag_scanner_SOURCE_DIR}/src/utils
        ${swag_scanner_SOURCE_DIR}/src/view

        ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}
        ${gmock_SOURCE_DIR}/include ${gmock_SOURCE_DIR}

        ${PCL_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR}
)


file(GLOB TEST_SRC_FILES
        ${swag_scanner_SOURCE_DIR}}/test/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/calibrationTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/equationsTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/fileTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/processingTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/typesTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/utilsTests/*.cpp


        ${swag_scanner_SOURCE_DIR}/src/cli/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/cli/*.h
        ${swag_scanner_SOURCE_DIR}/src/controller/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/controller/*.h
        ${swag_scanner_SOURCE_DIR}/src/file/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/file/*.h
        ${swag_scanner_SOURCE_DIR}/src/model/arduino/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/arduino/*.h
        ${swag_scanner_SOURCE_DIR}/src/model/calibration/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/calibration/*.h
        ${swag_scanner_SOURCE_DIR}/src/model/camera/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/camera/*.h
        ${swag_scanner_SOURCE_DIR}/src/model/equations/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/equations/*.h
        ${swag_scanner_SOURCE_DIR}/src/model/processing/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/processing/*.h
        ${swag_scanner_SOURCE_DIR}/src/types/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/types/*.h
        ${swag_scanner_SOURCE_DIR}/src/utils/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/utils/*.h
        ${swag_scanner_SOURCE_DIR}/src/view/*.cpp
        ${swag_scanner_SOURCE_DIR}/src/view/*.h
        )


add_executable(${TEST_MAIN} ${TEST_SRC_FILES})

target_link_libraries(${TEST_MAIN}
        feeling-blue
        gtest
        gtest_main
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${PCL_LIBRARIES}
        ${realsense2_LIBRARY})

add_test(${TEST_MAIN} ${TEST_MAIN})
