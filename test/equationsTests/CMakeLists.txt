target_sources(${TEST_MAIN} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/EquationsTests.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NormalTests.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PlaneTests.cpp
        )

target_include_directories(${TEST_MAIN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})