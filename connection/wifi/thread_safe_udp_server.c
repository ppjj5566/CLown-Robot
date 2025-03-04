#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/pbuf.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "FreeRTOS.h"
#include "task.h"

#include "received_joystick_data.h"

#define UDP_PORT 12345
#define UDP_SEND_PORT 12346

static struct udp_pcb *pcb; // UDP protocol control block

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{   
    struct received_joystick_data *recv_joy_data = (struct received_joystick_data *)arg;
    const size_t num_ints = 6;
    const size_t len = sizeof(int32_t) * num_ints;
    int32_t data[num_ints];
    if(p->len == len){
        memcpy(data, p->payload, len);
        recv_joy_data->x1 = data[0];
        recv_joy_data->y1 = data[1];
        recv_joy_data->z1 = data[2];
        recv_joy_data->pitch = data[3];
        recv_joy_data->roll = data[4];
        recv_joy_data->yaw = data[5];
        recv_joy_data->mode = 0;
        printf("Received data: %d,%d,%d,%d,%d,%d\n", data[0], data[1], data[2], data[3], data[4], data[5]);
        pbuf_free(p);
    }
}

void udp_task(void *pvParameters)
{
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms("ipiptime", "Park98124", CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("failed to connect\n");
        vTaskDelete(NULL);
        return;
    }
    printf("Connected to WIFI.\n");
    pcb = udp_new();

    if (!pcb)
    {
        printf("Error creating UDP PDB");
        vTaskDelete(NULL);
        return;
    }

    err_t bind = udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
    if (bind != ERR_OK)
    {
        printf("UDP bind failed\n");
        vTaskDelete(NULL);
        return;
    }

    udp_recv(pcb, udp_recv_callback, (struct received_joystick_data *)pvParameters);
    printf("UDP Server listening on port %d\n", UDP_PORT);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}