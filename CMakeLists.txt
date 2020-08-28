cmake_minimum_required(VERSION 3.15)
project(swag_scanner)

set(CMAKE_CXX_STANDARD 17)
set(SOURCE_FILES
        src/utils/json.hpp
        src/model/arduino/Arduino.h src/model/arduino/Arduino.cpp
        src/main.cpp
        src/model/camera/ICamera.h
        src/model/camera/SR305.h src/model/camera/SR305.cpp
        src/types/CameraTypes.h
        src/types/CloudType.h
        src/utils/Algorithms.h
        src/utils/Algorithms.cpp
        src/model/processing/Model.cpp src/model/processing/Model.h
        src/view/Visualizer.h src/view/Visualizer.cpp
        src/file/IFileHandler.h src/file/IFileHandler.cpp
        src/file/CalibrationFileHandler.h src/file/ScanFileHandler.h
        src/file/ScanFileHandler.cpp src/file/CalibrationFileHandler.cpp

        src/model/processing/Depth.h src/model/processing/Depth.cpp
        src/model/processing/Registration.h src/model/processing/Registration.cpp
        src/model/processing/Filtering.h src/model/processing/Filtering.cpp
        src/model/processing/Segmentation.h src/model/processing/Segmentation.cpp
        src/controller/ScanController.cpp src/controller/ScanController.h
        src/controller/ProcessingController.h
        src/controller/ProcessingController.cpp

        src/model/equations/Plane.h src/model/equations/Plane.cpp
        src/model/equations/Normal.h src/model/equations/Normal.cpp
        src/model/equations/Equations.h
        src/model/calibration/Calibration.h
        src/model/equations/Point.h src/model/equations/Point.cpp
        src/controller/CalibrationController.h src/controller/CalibrationController.cpp
        src/types/CameraTypes.cpp src/cli/ControllerFactory.h src/cli/ControllerFactory.cpp
        src/cli/CLIParser.h src/controller/IController.h
        src/cli/CLIParser.cpp src/controller/FilterTestingController.h src/controller/FilterTestingController.cpp
        src/controller/MoveController.h src/controller/MoveController.cpp
        src/controller/HomeController.h src/controller/HomeController.cpp src/utils/Constants.h)

set(include_dirs
        "/opt/librealsense/include"
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        src/cli
        src/controller
        src/file
        src/model/arduino
        src/model/calibration
        src/model/camera
        src/model/equations
        src/model/processing
        src/types
        src/utils
        src/view
        ${WRAPPERS}
        ${VIEW_HEADERS}
        ${CAMERA_HEADERS}
        ${ARDUINO_HEADERS}
        ${PROCESSING_HEADERS}
        ${CONTROLLER_HEADERS}
        ${TYPES_HEADERS}
        ${EQUATIONS_HEADERS}
        ${CALIBRATION_HEADERS}
        ${FILE_HEADERS}
        ${UTILS}
        ${CLI_HEADERS})


# --------------------------------------------------------------------------------
#                          Add executable
# --------------------------------------------------------------------------------
add_executable(swag_scanner ${SOURCE_FILES})

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

# --------------------------------------------------------------------------------
#                          Find libraries
# --------------------------------------------------------------------------------
find_package(Boost 1.72.0 COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.11 REQUIRED)
find_package(feeling-blue REQUIRED)

#add_subdirectory(test)

link_directories("/opt/librealsense/build/librealsense2.dylib")

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

# --------------------------------------------------------------------------------
#                          Link libraries
# --------------------------------------------------------------------------------
target_link_libraries(swag_scanner PRIVATE
        feeling-blue
        /opt/librealsense/build/librealsense2.dylib
        ${PCL_LIBRARIES}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        )

# --------------------------------------------------------------------------------
#                          Include directories
# --------------------------------------------------------------------------------
target_include_directories(swag_scanner PRIVATE ${include_dirs})