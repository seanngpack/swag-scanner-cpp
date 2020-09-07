target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/SwagGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SwagGUI.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrateControls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrateControls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanControls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanControls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessControls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ProcessControls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveControls.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveControls.h
        ${CMAKE_CURRENT_SOURCE_DIR}/SideBar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SideBar.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IFormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IFormsPayload.h
        ${CMAKE_CURRENT_SOURCE_DIR}/FormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FormsPayload.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveFormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveFormsPayload.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})