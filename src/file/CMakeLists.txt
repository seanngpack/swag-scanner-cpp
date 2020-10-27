target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/IFileHandler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IFileHandler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationFileHandler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationFileHandler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanFileHandler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ScanFileHandler.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})