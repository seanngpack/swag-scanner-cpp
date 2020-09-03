target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationControllerGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationControllerGUI.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})