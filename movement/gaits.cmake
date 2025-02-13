set(GAITS gaits)

if(NOT TARGET ${GAITS})
    add_library(${GAITS} INTERFACE)
endif()

target_sources(${GAITS} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/gaits.cpp
)

target_include_directories(${GAITS} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${GAITS} INTERFACE
    inverse_kinematics
    received_joystick_data
)