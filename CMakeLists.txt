cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
set(SOURCE_FILES src/main.cpp src/model/camera/SR305.cpp include/model/camera/ICamera.h
        include/model/camera/SR305.h include/utils/CameraTypes.h
        include/utils/Algorithms.h src/model/processing/Model.cpp
        include/model/processing/Model.h)
set(CAMERA_HEADERS ./include/model/camera)
set(PROCESSING_HEADERS ./include/model/processing)
set(UTILS ./include/utils)

#set(OpenCV_DIR "/usr/local/Cellar/opencv/4.2.0_3/lib/cmake/opencv4")

find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(test)

add_executable(swag_scanner ${SOURCE_FILES} src/model/camera/SR305.cpp)

include_directories("/opt/librealsense/include" ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CAMERA_HEADERS})
include_directories(${PROCESSING_HEADERS})
include_directories(${UTILS})

link_directories("/opt/librealsense/build/librealsense2.dylib")
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(swag_scanner PUBLIC /opt/librealsense/build/librealsense2.dylib PRIVATE ${OpenCV_LIBS})
target_link_libraries(swag_scanner PUBLIC ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

