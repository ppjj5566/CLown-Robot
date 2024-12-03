#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"

#include "wifi_connection.cpp"
#include "kinematics.hpp"
#include "udp_server.hpp"

#include "servo2040.hpp"

using namespace servo;

char ssid[64], pw[64];
const int port = 12345;
float v_sersor_value = 0.194579f;

const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_18;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
Calibration *calibration[NUM_SERVOS];
ServoCluster *servo_cluster;

wifi_connection wifi;
udp_server server;
static Kinematics *kinematics1, *kinematics2, *kinematics3, *kinematics4, *kinematics5, *kinematics6;
received_joystick_data joy_data = {0, 0, 0, 0, 0, 0, 0, 0, 0};

const float sin60 = 0.86602540378f;
const float sin30 = 0.5f;
const float cos30 = 0.86602540378f;                                                                                                                                                                                                                                                                                                                                           
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

// bool repeat_voltage_measurement(struct repeating_timer *t){
//     adc_init();
//     adc_gpio_init(26);
//     adc_select_input(0);
//     uint16_t result = adc_read() * v_sersor_value;
//     const float conversion_factor = 3.3f / (1 << 12);
//     char buffer[30];
//     sprintf(buffer, "Voltage: %f\n", conversion_factor * result);
//     send_char_to_tinyusb(buffer);
//     return true;
// }

void core1_entry(){
    while (true)
    {
        volatile float x, y, z;

        received_joystick_data *recv_joy_data = (received_joystick_data *)multicore_fifo_pop_blocking();
        
        x = recv_joy_data->x1;
        y = recv_joy_data->y1;
        z = -20.0f + recv_joy_data->z1;

        kinematics1->endpoint((cos60 * x) + (sin60 * y), (50.0f + (cos60 * y)) - (sin60 * x), z);
        kinematics2->endpoint(x, 50.0f + y, z);
        kinematics3->endpoint((cos60 * x) - (sin60 * y), (50.0f + (cos60 * y)) + (sin60 * x), z);
        kinematics4->endpoint((cos60 * -x) - (sin60 * y), (50.0f - (cos60 * y)) + (sin60 * x), z);
        kinematics5->endpoint(-x, 50.0f - y, z);
        kinematics6->endpoint((cos60 * -x) + (sin60 * y), (50.0f - (cos60 * y)) - (sin60 * x), z);

        multicore_fifo_drain();
    }
}

int main(){
    // alarm_pool_t *alarm_pool;
    // repeating_timer_t *t;

    stdio_init_all();
    tusb_init();
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);
    // alarm_pool_add_repeating_timer_ms(alarm_pool, 200, repeat_voltage_measurement, NULL, t); // stoping hole program

    sleep_ms(3000);
    send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    send_and_get_char_from_tinyusb("Enter Password: ", pw);
    
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    servo_cluster->enable_all();

    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
        servo_cluster->calibration(i).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }

    kinematics1 = new Kinematics(servo_cluster, 0);
    kinematics2 = new Kinematics(servo_cluster, 1);
    kinematics3 = new Kinematics(servo_cluster, 2);
    kinematics4 = new Kinematics(servo_cluster, 3);
    kinematics5 = new Kinematics(servo_cluster, 4);
    kinematics6 = new Kinematics(servo_cluster, 5);

    wifi.connect_wifi(ssid, pw);
    server.start_udp_server(12345, joy_data);

    return 0;
}