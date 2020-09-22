target_sources(swag_scanner_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/SwagGUI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SwagGUI.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IFormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IFormsPayload.h
        ${CMAKE_CURRENT_SOURCE_DIR}/FormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FormsPayload.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveFormsPayload.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MoveFormsPayload.h
        )

target_include_directories(swag_scanner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})