set(DRIVER_NAME pwm_cluster)

if (NOT TARGET ${DRIVER_NAME})
  add_library(${DRIVER_NAME} INTERFACE)
endif()

target_sources(${DRIVER_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/pwm_cluster.cpp
)

target_include_directories(${DRIVER_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${DRIVER_NAME} INTERFACE
    pico_stdlib
    hardware_pio
    hardware_dma
    )

if (NOT TARGET pwm_cluster_pwm_cluster_pio_h) #조건부 타겟 생성: 타겟을 정의할 때 이미 존재하는지 확인하고, 존재하지 않을 때만 정의합니다.
  pico_generate_pio_header(${DRIVER_NAME} ${CMAKE_CURRENT_LIST_DIR}/pwm_cluster.pio)
endif()