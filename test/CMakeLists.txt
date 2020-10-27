cmake_minimum_required(VERSION 3.17)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(TEST_MAIN unit_tests)

include_directories(
        ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}
        ${gmock_SOURCE_DIR}/include ${gmock_SOURCE_DIR}
)

add_executable(${TEST_MAIN} main.cpp)
target_link_libraries(${TEST_MAIN} PRIVATE swag_scanner_lib
        gtest
        gtest_main)

add_subdirectory(calibrationTests)
add_subdirectory(cameraTests)
add_subdirectory(equationsTests)
add_subdirectory(fileTests)
add_subdirectory(processingTests)
add_subdirectory(typesTests)
add_subdirectory(utilsTests)

if (BUILD_TESTS_VISUAL)
    message("building visual tests")
    add_subdirectory(research)

endif()

add_test(${TEST_MAIN} ${TEST_MAIN})
