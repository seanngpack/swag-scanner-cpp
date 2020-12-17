set(algoDev_target temp)
add_custom_command(OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/research/algoDev/data/${algoDev_target}
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/test/research/algoDev/data
        ${CMAKE_CURRENT_BINARY_DIR}/data
        DEPENDS ${TEST_MAIN}
        )

add_custom_target(copy_algoDev_files ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/research/algoDev/data/${algoDev_target}
        )

target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/2DHoleFilling.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})