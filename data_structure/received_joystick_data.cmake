set(JOYSTICK_DATA received_joystick_data)

if(NOT TARGET ${JOYSTICK_DATA})
    add_library(${JOYSTICK_DATA} INTERFACE)
endif()

target_sources(${JOYSTICK_DATA} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/received_joystick_data.h
)

target_include_directories(${JOYSTICK_DATA} INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}
)