project(tests)

add_subdirectory(googletest-master)

# include utils headers
include_directories(${swag_scanner_SOURCE_DIR}/include/utils)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(googletest-master/googlemock/include)

add_executable(tests_run AlgosTests.cpp)
target_link_libraries(tests_run
        gtest
        gtest_main
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES})
