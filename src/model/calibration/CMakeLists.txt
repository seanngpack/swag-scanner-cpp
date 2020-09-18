target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationModel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationModel.cpp
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})