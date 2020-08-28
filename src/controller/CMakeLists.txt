target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/FilterTestingController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FilterTestingController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/HomeController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/HomeController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanController.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})