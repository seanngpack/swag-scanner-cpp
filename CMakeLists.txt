cmake_minimum_required(VERSION 3.17)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
# find pcl installed with brew
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(JSON_BuildTests OFF CACHE INTERNAL "")
# these are for qt .ui and .qrc files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(PCL_DIR /usr/local/Cellar/pcl/1.11.1_5/share/pcl-1.11/)
if(NOT Qt5_DIR)
    set(Qt5_DIR "/usr/local/opt/qt/lib/cmake/Qt5")
endif()

option(BUILD_TESTS "Build the tests" OFF)
option(BUILD_TESTS_VISUAL "Build the visual tests" OFF)


# --------------------------------------------------------------------------------
#                          Find libraries
# --------------------------------------------------------------------------------
find_package(PCL 1.11 REQUIRED)
find_package(VTK REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Quick Designer REQUIRED)
find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(feeling-blue REQUIRED)
find_package(realsense2 REQUIRED)
find_package(spdlog REQUIRED)


add_definitions(${PCL_DEFINITIONS})
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR})

# --------------------------------------------------------------------------------
#                          Subdirectories
# --------------------------------------------------------------------------------

add_subdirectory(extern/json)
add_subdirectory(extern/SwagScannerWidgets)
include_directories(extern/SwagScannerWidgets/swagscanner_widgets/include)
add_subdirectory(src)

# --------------------------------------------------------------------------------
#                          Build tests
# --------------------------------------------------------------------------------

if (BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(${PROJECT_SOURCE_DIR}/extern/googletest)
    add_subdirectory(test)
endif ()
