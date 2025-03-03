set(THREAD_SAFE_UDP_SERVER thread_safe_udp_server)

if(NOT TARGET ${THREAD_SAFE_UDP_SERVER})
    add_library(${THREAD_SAFE_UDP_SERVER} INTERFACE)
endif()

target_sources(${THREAD_SAFE_UDP_SERVER} INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/thread_safe_udp_server.c
)

target_include_directories(${THREAD_SAFE_UDP_SERVER} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${THREAD_SAFE_UDP_SERVER} INTERFACE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
)