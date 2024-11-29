set (KINEMATICS_NAME kinematics)

if(NOT TARGET ${KINEMATICS_NAME})
    add_library(${KINEMATICS_NAME} INTERFACE)
endif()

target_sources(${KINEMATICS_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/kinematics.cpp
)

target_include_directories(${KINEMATICS_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(${KINEMATICS_NAME} INTERFACE
    pico_stdlib
    servo2040
    pico_multicore
)