set(INVERSE_KINEMATICS_NAME inverse_kinematics)

if(NOT TARGET ${INVERSE_KINEMATICS_NAME})
    add_library(${INVERSE_KINEMATICS_NAME} INTERFACE)
endif()

target_sources(${INVERSE_KINEMATICS_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/inverse_kinematics.cpp
)

target_include_directories(${INVERSE_KINEMATICS_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${INVERSE_KINEMATICS_NAME} INTERFACE
    kinematics
)