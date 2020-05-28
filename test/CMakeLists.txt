project(tests)

add_subdirectory(googletest-master)

# include utils headers
# have to redefine the includes because the tests are a separate project.
include_directories(${swag_scanner_SOURCE_DIR}/include/utils
                    ${swag_scanner_SOURCE_DIR}/include/model/processing
                    ${swag_scanner_SOURCE_DIR}/include/wrapper)

include_directories("/opt/librealsense/include")
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(googletest-master/googlemock/include)
#link_directories("/opt/librealsense/build/librealsense2.dylib")


add_executable(tests_run ModelTests.cpp AlgosTests.cpp CameraTypesTests.cpp
        ${swag_scanner_SOURCE_DIR}/src/model/processing/Model.cpp
        ${swag_scanner_SOURCE_DIR}/src/utils/FileHandler.cpp
        ${swag_scanner_SOURCE_DIR}/src/wrapper/CoreBluetoothObjC.mm
        FileHandlerTests.cpp mocks/MockFileHandler.cpp WrapperTests.cpp)
target_link_libraries(tests_run
        gtest
        gtest_main
        ${PCL_LIBRARIES}
        /opt/librealsense/build/librealsense2.dylib)
