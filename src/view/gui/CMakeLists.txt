target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/swag_gui.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/swag_gui.h
        ${CMAKE_CURRENT_SOURCE_DIR}/calibrate_controls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/calibrate_controls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/scan_controls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/scan_controls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/process_controls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/process_controls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sidebar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sidebar.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})