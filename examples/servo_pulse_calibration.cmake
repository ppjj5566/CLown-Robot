set(OUTPUT_NAME servo_pulse_calibration)
add_executable(${OUTPUT_NAME} servo_pulse_calibration.cpp)

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    servo2040
)

pico_enable_stdio_usb(${OUTPUT_NAME} 1)

pico_add_extra_outputs(${OUTPUT_NAME})