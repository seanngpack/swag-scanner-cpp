cmake_minimum_required(VERSION 3.17)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(TEST_MAIN unit_tests)

include_directories(
        ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}
        ${gmock_SOURCE_DIR}/include ${gmock_SOURCE_DIR}
)
file(GLOB TEST_SRC_FILES
        ${swag_scanner_SOURCE_DIR}/test/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/calibrationTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/equationsTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/fileTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/processingTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/typesTests/*.cpp
        ${swag_scanner_SOURCE_DIR}/test/utilsTests/*.cpp
        )

add_executable(${TEST_MAIN} ${TEST_SRC_FILES})
target_link_libraries(${TEST_MAIN} PRIVATE swag_scanner_lib
        gtest
        gtest_main)

add_test(${TEST_MAIN} ${TEST_MAIN})
