#include "udp_server.hpp"
#include "pico/multicore.h"
#include "hardware/irq.h"

#define RCV_FROM_IP              IP_ADDR_ANY
#define MAX_ARRAY_SIZE           6
#define MAX_BUFFER_SIZE          1024


udp_server::udp_server() {
    pcb = udp_new();
    if (pcb == nullptr){
        printf("Error creating UDP PDB");
        return;
    }
}

void udp_server::start_udp_server(const int port, received_joystick_data *recv_joy_data){
    err_t bind = udp_bind(pcb, RCV_FROM_IP, port);

    if(bind != ERR_OK){
        printf("UDP bind failed\n");
        return;
    }

    printf("udp server started!\n");
    udp_recv(pcb, udp_receive_callback, recv_joy_data);

    while(true){
        cyw43_arch_poll();
    }
}

void udp_server::udp_receive_callback(void *arg, udp_pcb *pcb, pbuf *p, const ip_addr_t *addr, u16_t port) {
    received_joystick_data *recv_joy_data = (received_joystick_data *)arg;
    const size_t num_ints = 6;
    const size_t len = sizeof(int32_t) * num_ints;
    int32_t received_data[num_ints];
    if(p->len == len){
        memcpy(received_data, p->payload, p->len);
        recv_joy_data->x1 = received_data[0];
        recv_joy_data->y1 = received_data[1];
        recv_joy_data->z1 = received_data[2];
        recv_joy_data->roll = received_data[3];
        recv_joy_data->pitch = received_data[4];
        recv_joy_data->yaw = received_data[5];
        //printf("Received data: x = %i, y = %i, z = %i, roll = %i, pitch =%i, yaw = %i\n", 
                            // recv_joy_data->x1, recv_joy_data->y1, recv_joy_data->z1, 
                            // recv_joy_data->roll, recv_joy_data->pitch, recv_joy_data->yaw);
        multicore_fifo_push_blocking((uint32_t)&recv_joy_data);
    }
    else {
        printf("Error: Received data exceeds buffer size.\n");
    }
    pbuf_free(p);
}

void udp_server::send_data(const ip_addr_t *addr, const int port, const char *data, const int len){
    pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if(p != nullptr){
        memcpy(p->payload, data, len);
        udp_sendto(pcb, p, addr, port);
        pbuf_free(p);
    }
}

udp_server::~udp_server(){   
    udp_remove(pcb);
    printf("udp connection disabled!\n");
}



