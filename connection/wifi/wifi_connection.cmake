set(CONNECTION_NAME wifi_connection)

if(NOT TARGET ${CONNECTION_NAME})
    add_library(${CONNECTION_NAME} INTERFACE)
endif()

target_sources(${CONNECTION_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/wifi_connection.hpp
)
target_include_directories(${CONNECTION_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${CONNECTION_NAME} INTERFACE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)