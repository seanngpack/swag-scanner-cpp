# --------------------------------------------------------------------------------
#                          Add executable
# --------------------------------------------------------------------------------
add_library(swag_scanner_lib "")

# --------------------------------------------------------------------------------
#                          Link libraries
# --------------------------------------------------------------------------------
target_link_libraries(swag_scanner_lib PRIVATE
        feeling-blue
        ${PCL_LIBRARIES}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${realsense2_LIBRARY}
        )

add_subdirectory(cli)
add_subdirectory(controller)
add_subdirectory(file)
add_subdirectory(model)
add_subdirectory(types)
add_subdirectory(utils)
add_subdirectory(view)


add_executable(swag_scanner main.cpp)
target_link_libraries(swag_scanner PRIVATE swag_scanner_lib)
