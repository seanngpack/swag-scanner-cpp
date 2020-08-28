cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)


# --------------------------------------------------------------------------------
#                          Add executable
# --------------------------------------------------------------------------------
add_executable(swag_scanner "")


# --------------------------------------------------------------------------------
#                          Find libraries
# --------------------------------------------------------------------------------
find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.11 REQUIRED)
find_package(feeling-blue REQUIRED)
find_package(realsense2 REQUIRED)


add_definitions(${PCL_DEFINITIONS})
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR})

#move all this external stuff in source file i think

# --------------------------------------------------------------------------------
#                          Link libraries
# --------------------------------------------------------------------------------
target_link_libraries(swag_scanner PRIVATE
        feeling-blue
        ${PCL_LIBRARIES}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${realsense2_LIBRARY}
        )

# --------------------------------------------------------------------------------
#                          Subdirectories
# --------------------------------------------------------------------------------

#add_subdirectory(test)
add_subdirectory(src)

# --------------------------------------------------------------------------------
#                          Include directories
# --------------------------------------------------------------------------------
#target_include_directories(swag_scanner PRIVATE ${realsense_INCLUDE_DIR})