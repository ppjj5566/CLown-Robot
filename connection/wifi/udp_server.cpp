#include "udp_server.hpp"
#include "pico/multicore.h"
#include "hardware/irq.h"

#define MAX_BUFFER_SIZE 1024

udp_server::udp_server() {
    pcb = udp_new();
    if (pcb == nullptr){
        printf("Error creating UDP PDB");
        return;
    }
}

void udp_server::start_udp_server(const int port, received_joystick_data recv_joy_data){
    err_t bind = udp_bind(pcb, RCV_FROM_IP, port);
    if(bind != ERR_OK){
        printf("UDP bind failed\n");
        return;
    }
    printf("udp server started!\n");
    udp_recv(pcb, udp_receive_callback, &recv_joy_data);

    while(true){
        cyw43_arch_poll();
    }
}

void udp_server::udp_receive_callback(void *arg, udp_pcb *pcb, pbuf *p, const ip_addr_t *addr, u16_t port) {
    received_joystick_data *recv_joy_data = (received_joystick_data *)arg;
    uint32_t received_data[MAX_ARRAY_SIZE];
    if(p->len <= MAX_BUFFER_SIZE){
        int received_data[MAX_ARRAY_SIZE/ sizeof(int)];
        memcpy(received_data, p->payload, p->len);
        recv_joy_data->x1 = received_data[0];
        recv_joy_data->y1 = received_data[1];
        recv_joy_data->z1 = received_data[2];
        printf("Received data: x = %i, y = %i, z = %i\n", recv_joy_data->x1, recv_joy_data->y1, recv_joy_data->z1);
        multicore_fifo_push_blocking((uint32_t)recv_joy_data);
    }
    else {
        printf("Error: Received data exceeds buffer size.\n");
    }
    pbuf_free(p);
}

udp_server::~udp_server()
{   
    udp_remove(pcb);
    printf("udp connection disabled!\n");
}



