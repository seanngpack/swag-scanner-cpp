project(tests)
#add_subdirectory(../lib/googletest-master)
#include_directories(lib/googletest)
#include_directories(lib/googletest/include)
#include_directories(lib/googlemock)
#include_directories(lib/googlemock/include)

add_subdirectory(googletest-master)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(googletest-master/googlemock/include)

add_executable(tests_run AlgosTests.cpp)
target_link_libraries(tests_run gtest gtest_main)