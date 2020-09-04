add_subdirectory(calibration)
add_subdirectory(filter_testing)
add_subdirectory(home)
add_subdirectory(move)
add_subdirectory(processing)
add_subdirectory(scan)

target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/IController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IControllerGUI.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IControllerGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerProxy.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerProxy.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})