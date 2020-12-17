add_subdirectory(algoDev)
add_subdirectory(depthFiltering)
add_subdirectory(meshing)
add_subdirectory(registration)
add_subdirectory(segmentation)
add_subdirectory(calibration)

target_sources(${TEST_MAIN} PRIVATE

        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})