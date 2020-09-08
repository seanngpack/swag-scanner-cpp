target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingControllerGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessingControllerGUI.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})