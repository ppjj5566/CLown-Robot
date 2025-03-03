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

#define UDP_PORT 12345
#define UDP_SEND_PORT 12346

static struct udp_pcb *pcb; // UDP protocol control block

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    char *data = (char *)p->payload;
    printf("Received data: %s\n", data);
    pbuf_free(p);
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

    udp_recv(pcb, udp_recv_callback, NULL);
    printf("UDP Server listening on port %d\n", UDP_PORT);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}