#include <stdio.h>
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/flash.h"

#include "wifi_connection.cpp"
//#include "inverse_kinematics.cpp"
#include "udp_server.hpp"
#include "gaits.cpp"

#include "servo2040.hpp"

#define FLASH_TARGET_OFFSET (256 * 1024)

using namespace servo;

char ssid[64], pw[64];
const int port = 12345;

const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_18;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

const uint8_t *flash_target_servo_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);


repeating_timer timer;

Calibration *calibration[NUM_SERVOS];
ServoCluster *servo_cluster;
wifi_connection wifi;
udp_server server;
Kinematics *kinematics1, *kinematics2, *kinematics3, *kinematics4, *kinematics5, *kinematics6;
inverse_kinematics *ik;
gaits *gait;
received_joystick_data *joy_data;

volatile int x, y, z, roll, pitch, yaw;

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

void setup_voltage_sensor(){
    adc_gpio_init(27);
    adc_select_input(1);
}   

void setup_amp_sensor(){
    adc_gpio_init(26);
    adc_select_input(0);
}

void setup_temp_sensor(){
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}



bool adc_callback(repeating_timer_t *rt){
    const float conversion_factor = 3.3f / (1 << 12);
    adc_select_input(1);
    uint16_t result = adc_read();
    adc_select_input(0);
    uint16_t result1 = adc_read();
    adc_select_input(4);
    uint16_t result2 = adc_read();
    printf("Consumption: %.2fA, Batt: %.2fV, MCU Temperature: %.1f°C\n",  
            (((float)result * conversion_factor) - 1.65f) / 0.09f,  
            (float)result1 * conversion_factor * 8.5f,  
            27 - ((((float)result2 * conversion_factor) - 0.706)/0.001721));
    return true;
}


void core1_entry(){ //calculate inverse kinematics on core1
    while (true)
    {
        multicore_fifo_pop_blocking();
        x = joy_data->x1;
        y = joy_data->y1;
        z = -20.0f + joy_data->z1;
        roll = joy_data->roll;
        pitch = joy_data->pitch;
        yaw = joy_data->yaw;
        gait->move(0, x, y, z);
        auto recv_ip = server.get_recv_ip();
        server.send_data(&recv_ip, port, (char *)joy_data, sizeof(received_joystick_data));
        multicore_fifo_drain();
    }
}

int main(){
    stdio_init_all();
    tusb_init();
    adc_init();
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    setup_amp_sensor();
    setup_voltage_sensor();
    setup_temp_sensor();

    sleep_ms(3000);
    send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    send_and_get_char_from_tinyusb("Enter Password: ", pw);

    add_repeating_timer_ms(100, adc_callback, NULL, &timer);
    
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    servo_cluster->enable_all();

    joy_data = new received_joystick_data();

    for (size_t i = 0; i < NUM_SERVOS; i++){
        servo_cluster->calibration(i).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }

    kinematics1 = new Kinematics(servo_cluster, 0);
    kinematics2 = new Kinematics(servo_cluster, 1);
    kinematics3 = new Kinematics(servo_cluster, 2);
    kinematics4 = new Kinematics(servo_cluster, 3);
    kinematics5 = new Kinematics(servo_cluster, 4);
    kinematics6 = new Kinematics(servo_cluster, 5);
    ik = new inverse_kinematics(kinematics1, kinematics2, kinematics3, kinematics4, kinematics5, kinematics6);
    gait = new gaits(ik);

    wifi.connect_wifi(ssid, pw);
    server.start_udp_server(12345, joy_data);

    return 0;
}