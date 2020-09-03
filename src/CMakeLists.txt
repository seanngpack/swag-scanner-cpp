# --------------------------------------------------------------------------------
#                          Add library
# --------------------------------------------------------------------------------
add_library(swag_scanner_lib "")

# --------------------------------------------------------------------------------
#                          Link libraries to library
# --------------------------------------------------------------------------------
target_link_libraries(swag_scanner_lib PRIVATE
        feeling-blue
        ${PCL_LIBRARIES}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${realsense2_LIBRARY}

        nlohmann_json::nlohmann_json
        )

# --------------------------------------------------------------------------------
#                          Add subdirectories
# --------------------------------------------------------------------------------
add_subdirectory(cli)
add_subdirectory(controller)
add_subdirectory(file)
add_subdirectory(model)
add_subdirectory(types)
add_subdirectory(utils)
add_subdirectory(view)

# --------------------------------------------------------------------------------
#                          Add executable and link library
# --------------------------------------------------------------------------------
add_executable(swag_scanner main.cpp)
target_link_libraries(swag_scanner PRIVATE swag_scanner_lib Qt5::Quick Qt5::Core Qt5::Widgets)
