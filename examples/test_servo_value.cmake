set(OUTPUT_NAME test_servo_value)

add_executable(${OUTPUT_NAME} test_servo_value.cpp)

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        servo2040
        pico_multicore
        )

pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

pico_add_extra_outputs(${OUTPUT_NAME})