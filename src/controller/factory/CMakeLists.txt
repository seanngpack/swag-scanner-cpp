target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactoryCache.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactoryCache.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
