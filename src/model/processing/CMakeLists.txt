target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Depth.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Depth.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Filtering.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Filtering.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Model.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Model.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Registration.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Registration.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Segmentation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Segmentation.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})