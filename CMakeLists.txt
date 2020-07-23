cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
set(SOURCE_FILES
        include/utils/json.hpp
        include/model/arduino/Arduino.h src/model/arduino/Arduino.cpp
        include/model/arduino/ArduinoEventHandler.h src/model/arduino/ArduinoEventHandler.cpp
        include/bluetooth/SERVICES.h
        include/bluetooth/CoreBluetoothWrapper.h
        include/bluetooth/CoreBluetoothObjC.h
        src/bluetooth/CoreBluetoothObjC.mm
        src/main.cpp
        include/model/camera/ICamera.h
        include/model/camera/SR305.h src/model/camera/SR305.cpp
        include/types/CameraTypes.h
        include/types/CloudType.h
        include/utils/Algorithms.h
        src/model/processing/Model.cpp include/model/processing/Model.h
        include/view/Visualizer.h src/view/Visualizer.cpp
        include/file/IFileHandler.h src/file/IFileHandler.cpp
        include/file/CalibrationFileHandler.h include/file/ScanFileHandler.h
        src/file/ScanFileHandler.cpp src/file/CalibrationFileHandler.cpp

        include/model/processing/Depth.h src/model/processing/Depth.cpp
        include/model/processing/Registration.h src/model/processing/Registration.cpp
        include/model/processing/Filtering.h src/model/processing/Filtering.cpp
        include/model/processing/Segmentation.h src/model/processing/Segmentation.cpp
        src/controller/ScanController.cpp include/controller/ScanController.h
        include/controller/ProcessingController.h
        src/controller/ProcessingController.cpp

        include/model/equations/Plane.h src/model/equations/Plane.cpp
        include/model/equations/Normal.h src/model/equations/Normal.cpp
        include/model/equations/Equations.h
        include/model/calibration/Calibration.h
        include/model/equations/Point.h src/model/equations/Point.cpp
        include/controller/CalibrationController.h src/controller/CalibrationController.cpp
        src/types/CameraTypes.cpp include/cli/ControllerFactory.h src/cli/ControllerFactory.cpp
        include/cli/CLIParser.h include/controller/IController.h
        src/cli/CLIParser.cpp include/controller/FilterTestingController.h src/controller/FilterTestingController.cpp
        include/controller/MoveController.h src/controller/MoveController.cpp
        include/controller/HomeController.h src/controller/HomeController.cpp)

set(WRAPPERS include/bluetooth)
set(VIEW_HEADERS ./include/view)
set(CAMERA_HEADERS ./include/model/camera)
set(ARDUINO_HEADERS ./include/model/arduino)
set(PROCESSING_HEADERS ./include/model/processing)
set(CONTROLLER_HEADERS ./include/controller)
set(TYPES_HEADERS ./include/types)
set(EQUATIONS_HEADERS ./include/model/equations)
set(CALIBRATION_HEADERS ./include/model/calibration)
set(FILE_HEADERS ./include/file)
set(UTILS ./include/utils)
set(CLI_HEADERS ./include/cli)

# double check why this fixes core bluetooth linkage issue
set(CMAKE_EXE_LINKER_FLAGS "-framework CoreBluetooth")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.11 REQUIRED)


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
include_directories(${CONTROLLER_HEADERS})
include_directories(${TYPES_HEADERS})
include_directories(${EQUATIONS_HEADERS})
include_directories(${CALIBRATION_HEADERS})
include_directories(${FILE_HEADERS})
include_directories(${UTILS})
include_directories(${CLI_HEADERS})

link_directories("/opt/librealsense/build/librealsense2.dylib")
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(swag_scanner ${SOURCE_FILES})

target_link_libraries(swag_scanner PUBLIC
        "-framework CoreFoundation"
        "-framework CoreBluetooth")
set_target_properties(swag_scanner PROPERTIES LINK_FLAGS "-Wl,-F/Library/Frameworks")
target_link_libraries(swag_scanner PUBLIC /opt/librealsense/build/librealsense2.dylib PRIVATE ${OpenCV_LIBS})
target_link_libraries(swag_scanner PUBLIC ${PCL_LIBRARIES})
target_link_libraries(swag_scanner PUBLIC ${Boost_PROGRAM_OPTIONS_LIBRARY})