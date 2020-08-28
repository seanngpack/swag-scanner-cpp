cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

option(BUILD_TESTS "Build the tests" OFF)

# --------------------------------------------------------------------------------
#                          Find libraries
# --------------------------------------------------------------------------------
find_package(PCL REQUIRED)
find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(feeling-blue REQUIRED)
find_package(realsense2 REQUIRED)


add_definitions(${PCL_DEFINITIONS})
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR})


# --------------------------------------------------------------------------------
#                          Subdirectories
# --------------------------------------------------------------------------------

add_subdirectory(extern/json)
add_subdirectory(src)

if (BUILD_TESTS)
    include(CTest)
    enable_testing()
    message("enabled building tests")
    add_subdirectory(${PROJECT_SOURCE_DIR}/extern/googletest)
    add_subdirectory(test)
endif()
