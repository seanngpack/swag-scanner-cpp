file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

message(" the binary is: " ${CMAKE_CURRENT_BINARY_DIR})

target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CompareDepthFiltering.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})