#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"

#include "received_joystick_data.h"

class udp_server{
private:
    struct udp_pcb* pcb;
    ip_addr_t recv_ip;
    
public:
    udp_server();
    ip_addr_t get_recv_ip() const { return recv_ip; }
    void start_udp_server(const int port, received_joystick_data *recv_joy_data);    
    void send_data(const ip_addr_t *addr, const int port, const char *data, const int len);
    static void udp_receive_callback(void *arg, udp_pcb *pcb, pbuf *p, const ip_addr_t *addr, u16_t port);
    ~udp_server();
};