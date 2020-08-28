target_sources(swag_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Arduino.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Arduino.h
        )

target_include_directories(swag_scanner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})