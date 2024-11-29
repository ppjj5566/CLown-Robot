set(DRIVER_NAME pwm)

if (NOT TARGET ${DRIVER_NAME})
  add_library(${DRIVER_NAME} INTERFACE)
endif()

target_sources(${DRIVER_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/pwm.cpp
)

target_include_directories(${DRIVER_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${DRIVER_NAME} INTERFACE
    pico_stdlib
    )