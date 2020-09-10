target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ModelTestsVisual.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SegmentationTestsVisual.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})