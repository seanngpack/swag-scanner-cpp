# --------------------------------------------------------------------------------
#                          Add library
# --------------------------------------------------------------------------------
add_library(swag_scanner_lib "")

# --------------------------------------------------------------------------------
#                          Link libraries to library
# --------------------------------------------------------------------------------
# consider changing this to public for spdlog
target_link_libraries(swag_scanner_lib PRIVATE
        feeling-blue
        ${PCL_LIBRARIES}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${realsense2_LIBRARY}
        Qt5::Quick Qt5::Core Qt5::Widgets Qt5::Designer
        nlohmann_json::nlohmann_json
        spdlog::spdlog
        ${VTK_LIBRARIES}
        calibration_basic_widget
        scan_basic_widget
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
target_link_libraries(swag_scanner PRIVATE swag_scanner_lib
        Qt5::Quick Qt5::Core Qt5::Widgets Qt5::Designer
        spdlog::spdlog
        nlohmann_json::nlohmann_json)
