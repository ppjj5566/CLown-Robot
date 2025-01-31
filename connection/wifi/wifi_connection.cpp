#include <stdio.h>

#include "wifi_connection.hpp"

// singleton class
// 와이파이 연결과 해제관련해서만 프로그램밍이 될것

wifi_connection::wifi_connection(){
    if(cyw43_arch_init()){
        printf("failed to initialise\n");
    }
    printf("initialised\n");
    cyw43_arch_enable_sta_mode();
}

void wifi_connection::connect_wifi(const char *wifi_ssid, const char *wifi_pw){
    memcpy(ssid, wifi_ssid, 30);
    memcpy(pw, wifi_pw, 30);
    if(!state_connected){
        printf("Connecting to %s\n", ssid);
        if (cyw43_arch_wifi_connect_timeout_ms(ssid, pw, CYW43_AUTH_WPA2_AES_PSK, 10000)){
        printf("failed to connect\n");
        }else{
            printf("Connected.\n");
            printf("IP: %s\n", ipaddr_ntoa((const ip_addr_t *) &cyw43_state.netif[0].ip_addr));
            state_connected = true;
        }
    }else{
        printf("Already connected\n");
        return;
    }
}


wifi_connection::~wifi_connection()
{
    delete pw;
    delete ssid;
    cyw43_arch_disable_sta_mode();
    cyw43_arch_init();
    printf("disconnect wifi.");
}
