project(tests)

add_subdirectory(googletest-master)
find_package(PCL 1.11 REQUIRED)
# include utils headers
# have to redefine the includes because the tests are a separate project.
include_directories(${swag_scanner_SOURCE_DIR}/include/utils
        ${swag_scanner_SOURCE_DIR}/include/model/processing
        ${swag_scanner_SOURCE_DIR}/include/wrapper
        ${swag_scanner_SOURCE_DIR}/include/view
        ${swag_scanner_SOURCE_DIR}/include/model/arduino
        ${swag_scanner_SOURCE_DIR}/include/types
        ${swag_scanner_SOURCE_DIR}/include/model/equations
        ${swag_scanner_SOURCE_DIR}/include/model/calibration
        ${swag_scanner_SOURCE_DIR}/include/file)

include_directories("/opt/librealsense/include")
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(googletest-master/googlemock/include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(tests_run
        ${swag_scanner_SOURCE_DIR}/src/view/Visualizer.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/processing/Model.cpp
        ${swag_scanner_SOURCE_DIR}/src/file/CalibrationFileHandler.cpp
        ${swag_scanner_SOURCE_DIR}/src/file/ScanFileHandler.cpp
        ${swag_scanner_SOURCE_DIR}/src/wrapper/CoreBluetoothObjC.mm
        ${swag_scanner_SOURCE_DIR}/src/model/arduino/Arduino.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/arduino/ArduinoEventHandler.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/equations/Normal.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/equations/Plane.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/equations/Point.cpp
        processingTests/ModelTests.cpp
        utilsTests/AlgosTests.cpp
        typesTests/CameraTypesTests.cpp
        fileTests/ScanFileHandlerTests.cpp
        typesTests/CloudTypeTests.cpp
        fileTests/ScanFileHandlerPhysicalTests.cpp
        WrapperTests.cpp processingTests/DepthTests.cpp
        processingTests/ModelPhysicalTests.cpp
        processingTests/SegmentationPhysicalTets.cpp
        calibrationTests/CalibrationTests.cpp
        equationsTests/NormalTests.cpp
        equationsTests/PlaneTests.cpp
        equationsTests/EquationsTests.cpp)
target_link_libraries(tests_run
        gtest
        gtest_main
        ${PCL_LIBRARIES}
        /opt/librealsense/build/librealsense2.dylib)