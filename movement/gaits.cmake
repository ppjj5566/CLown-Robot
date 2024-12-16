set(INTERFACE_NAME gaits)

if(NOT TARGET ${INTERFACE_NAME})
    add_library(${INTERFACE_NAME} INTERFACE)
endif()

target_sources(${INTERFACE_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/gaits.cpp
)

target_include_directories(${INTERFACE_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(${INTERFACE_NAME} INTERFACE
    inverse_kinematics
)