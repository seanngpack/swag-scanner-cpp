cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
set(SOURCE_FILES
        include/model/arduino/Arduino.h src/model/arduino/Arduino.cpp
        include/wrapper/SERVICES.h
        include/wrapper/CoreBluetoothWrapper.h
        include/wrapper/CoreBluetoothObjC.h
        src/wrapper/CoreBluetoothObjC.mm
        src/main.cpp
        include/model/camera/ICamera.h
        include/model/camera/SR305.h src/model/camera/SR305.cpp
        include/utils/CameraTypes.h
        include/utils/Algorithms.h
        src/model/processing/Model.cpp include/model/processing/Model.h
        include/view/Visualizer.h src/model/view/Visualizer.cpp
        include/utils/FileHandler.h src/utils/FileHandler.cpp
        include/utils/CloudType.h
        )

set(WRAPPERS ./include/wrapper)
set(VIEW_HEADERS ./include/view)
set(CAMERA_HEADERS ./include/model/camera)
set(ARDUINO_HEADERS ./include/model/arduino)
set(PROCESSING_HEADERS ./include/model/processing)
set(UTILS ./include/utils)

# double check why this fixes core bluetooth linkage issue
set(CMAKE_EXE_LINKER_FLAGS "-framework CoreBluetooth")

find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(test)

include_directories("/opt/librealsense/include" ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${WRAPPERS})
include_directories(${VIEW_HEADERS})
include_directories(${CAMERA_HEADERS})
include_directories(${ARDUINO_HEADERS})
include_directories(${PROCESSING_HEADERS})
include_directories(${UTILS})

link_directories("/opt/librealsense/build/librealsense2.dylib")
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(swag_scanner ${SOURCE_FILES})

target_link_libraries(swag_scanner PUBLIC "-framework CoreFoundation"
        "-framework CoreBluetooth")
set_target_properties(swag_scanner PROPERTIES LINK_FLAGS "-Wl,-F/Library/Frameworks")
target_link_libraries(swag_scanner PUBLIC /opt/librealsense/build/librealsense2.dylib PRIVATE ${OpenCV_LIBS})
target_link_libraries(swag_scanner PUBLIC ${PCL_LIBRARIES})