#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"

class wifi_connection{
    private:
        char *ssid = new char[30];
        char *pw = new char[30];
        bool state_connected = false;
    public:
        wifi_connection();
        void connect_wifi(const char *wifi_ssid, const char *wifi_pass);
        ~wifi_connection();
};