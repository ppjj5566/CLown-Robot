#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "tusb.h"

#include "wifi_connection.hpp"
#include "kinematics.hpp"
#include "udp_server.hpp"

#include "servo2040.hpp"

char ssid[64], pw[64];
float x, y, z;
const int port = 12345;
float v_sersor_value = 0.194579f;

using namespace servo;

const uint START_PIN = 2;
const uint END_PIN = 19;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
Servo *servos[NUM_SERVOS];
Calibration *calibration[NUM_SERVOS];

wifi_connection wifi;
udp_server server;  
Kinematics *kinematics1, *kinematics2, *kinematics3, *kinematics4, *kinematics5, *kinematics6;
received_joystick_data joy_data = {0, 0, 0, 0, 0, 0, 0, 0, 0};

const float sin60 = 0.86602540378f;
const float cos60 = 0.5f;

void get_char_from_tinyusb(char *buffer){
    size_t index = sizeof(buffer);
    int i = 0;
    while (true){
        if (tud_cdc_available()){
            char c = tud_cdc_read_char();
            if (c == '\n' || c == '\r'){
                break;
            }
            buffer[i++] = c;
        }
    }
}

void send_char_to_tinyusb(const char *message){
    while (*message){
        tud_cdc_write_char(*message++);
    }
    tud_cdc_write_flush();
}

void send_and_get_char_from_tinyusb(const char *message, char *buffer){
    send_char_to_tinyusb(message);
    get_char_from_tinyusb(buffer);
}

void get_and_send_char_to_tinyusb(char *buffer, const char *message){
    get_char_from_tinyusb(buffer);  
    send_char_to_tinyusb(message);
}

void core1_entry(){
    while(true){
        received_joystick_data *recv_joy_data = (received_joystick_data *)multicore_fifo_pop_blocking();

        x = recv_joy_data->x1;
        y = 50.0f + recv_joy_data->y1;
        z = -30.0f + recv_joy_data->z1;

        kinematics1->endpoint(x, y, z);
        kinematics2->endpoint(x, y, z);
        kinematics3->endpoint(x, y, z);
        kinematics4->endpoint(x, y, z);
        kinematics5->endpoint(x, y, z);
        kinematics6->endpoint(x, y, z);

        if (multicore_fifo_rvalid()){
            multicore_fifo_drain();
        }
    }
}

int main(){
    stdio_init_all();
    tusb_init();
    multicore_reset_core1();
    //adc_init();

    // adc_gpio_init(26);
    // adc_gpio_init(27);
    // adc_gpio_init(28);
    // adc_select_input(0);
    // adc_select_input(1);
    // adc_select_input(2);

    
    for(auto s = 0u; s < NUM_SERVOS; s++) {
        servos[s] = new Servo(s + START_PIN);
        calibration[s] = &servos[s]->calibration();
        calibration[s]->apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
        servos[s]->init();
        servos[s]->enable();
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%i Servo enabled\n", s);
        send_char_to_tinyusb(buffer);
    }

    multicore_launch_core1(core1_entry);

    kinematics1 = new Kinematics(servos[0], servos[1], servos[2], 0);
    kinematics2 = new Kinematics(servos[3], servos[4], servos[5], 1);
    kinematics3 = new Kinematics(servos[6], servos[7], servos[8], 2);
    kinematics4 = new Kinematics(servos[9], servos[10], servos[11], 3);
    kinematics5 = new Kinematics(servos[12], servos[13], servos[14], 4);
    kinematics6 = new Kinematics(servos[15], servos[16], servos[17], 5);

    sleep_ms(3000);

    send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    send_and_get_char_from_tinyusb("Enter Password: ", pw);

    wifi.connect_wifi(ssid, pw);

    server.start_udp_server(12345, joy_data);

    return 0;
}