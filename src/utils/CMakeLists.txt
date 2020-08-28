target_sources(swag_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Algorithms.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Algorithms.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Constants.h
        )

target_include_directories(swag_scanner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})