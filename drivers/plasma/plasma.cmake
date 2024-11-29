set(DRIVER_NAME plasma)

if (NOT TARGET ${DRIVER_NAME})
  add_library(${DRIVER_NAME} INTERFACE)
endif()

target_sources(${DRIVER_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/apa102.cpp
  ${CMAKE_CURRENT_LIST_DIR}/ws2812.cpp
)

target_include_directories(${DRIVER_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${DRIVER_NAME} INTERFACE
    pico_stdlib
    hardware_pio
    hardware_dma
    )

if(NOT TARGET plasma_apa102_pio_h)
  pico_generate_pio_header(${DRIVER_NAME} ${CMAKE_CURRENT_LIST_DIR}/apa102.pio)
endif()
if(NOT TARGET plasma_ws2812_pio_h)
  pico_generate_pio_header(${DRIVER_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
endif()