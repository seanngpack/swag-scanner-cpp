#file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# https://stackoverflow.com/questions/40294146/alternative-to-cmake-post-build-command-when-target-is-in-subdirectory
# used the stackoverflow for the logic behind custom command outside of target definition directory


set(temp_file temp)
add_custom_command(OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/research/DepthFiltering/data/${temp_file}
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/test/research/DepthFiltering/data
        ${CMAKE_CURRENT_BINARY_DIR}/data
        DEPENDS ${TEST_MAIN}
        )

add_custom_target(copy_files ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/research/DepthFiltering/data/${temp_file}
        )

target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CompareDepthFiltering.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})