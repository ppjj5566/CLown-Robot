set(OUTPUT_NAME test_console)

add_executable(${OUTPUT_NAME} test_console.cpp)

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    servo2040
    hardware_flash
)

pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_add_extra_outputs(${OUTPUT_NAME})