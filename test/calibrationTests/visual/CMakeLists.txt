target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationTestsVisual.cpp
        )

target_include_directories(${TEST_MAIN} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})