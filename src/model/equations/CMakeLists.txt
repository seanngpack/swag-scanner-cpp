target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Equations.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Normal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Normal.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Plane.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Plane.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Point.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Point.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})