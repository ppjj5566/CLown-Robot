set (LEG_CALIBRATION_NAME leg_calibration)

if(NOT TARGET ${LEG_CALIBRATION_NAME})
    add_library(${LEG_CALIBRATION_NAME} INTERFACE)
endif()

target_sources(${LEG_CALIBRATION_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/leg_calibration.hpp
)

target_include_directories(${LEG_CALIBRATION_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(${LEG_CALIBRATION_NAME} INTERFACE
    servo2040
)