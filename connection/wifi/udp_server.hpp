#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "kinematics.hpp"

#include "received_joystick_data.h"

class udp_server
{
private:
    struct udp_pcb* pcb;
    
public:
    udp_server();
    void start_udp_server(const int port, received_joystick_data *recv_joy_data);    
    static void udp_receive_callback(void *arg, udp_pcb *pcb, pbuf *p, const ip_addr_t *addr, u16_t port);
    ~udp_server();
}; 