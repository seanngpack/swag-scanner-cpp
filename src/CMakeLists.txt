add_subdirectory(cli)
add_subdirectory(controller)
add_subdirectory(file)
add_subdirectory(model)
add_subdirectory(types)
add_subdirectory(utils)
add_subdirectory(view)

target_sources(swag_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(swag_scanner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})