set(UDP_SERVER udp_server)

if(NOT TARGET ${UDP_SERVER})
    add_library(${UDP_SERVER} INTERFACE)
endif()

target_sources(${UDP_SERVER} INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/udp_server.cpp
)

target_include_directories(${UDP_SERVER} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${UDP_SERVER} INTERFACE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    received_joystick_data
    FreeRTOS-Kernel-Heap4
    gaits
)