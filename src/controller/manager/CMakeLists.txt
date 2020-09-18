target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerManager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerManager.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerManagerCache.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerManagerCache.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
