add_subdirectory(arduino)
add_subdirectory(camera)
add_subdirectory(calibration)
add_subdirectory(scan)
add_subdirectory(equations)
add_subdirectory(processing)


target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/IModel.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})