target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/FilterTestingController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FilterTestingController.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})