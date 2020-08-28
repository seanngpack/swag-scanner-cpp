target_sources(swag_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CameraTypes.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CameraTypes.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CloudType.h
        )

target_include_directories(swag_scanner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})