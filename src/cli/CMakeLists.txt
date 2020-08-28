target_sources(swag_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CLIParser.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CLIParser.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ControllerFactory.h
        )

target_include_directories(swag_scanner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})