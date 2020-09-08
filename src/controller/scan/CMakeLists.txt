target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanControllerGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanControllerGUI.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})