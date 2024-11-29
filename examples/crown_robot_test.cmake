set(OUTPUT_NAME crown_robot_test)

add_executable(${OUTPUT_NAME} crown_robot_test.cpp)

target_link_libraries(${OUTPUT_NAME}
    wifi_connection
    udp_server
    pico_stdlib
    kinematics
    pico_multicore
    hardware_adc
)

pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_add_extra_outputs(${OUTPUT_NAME})