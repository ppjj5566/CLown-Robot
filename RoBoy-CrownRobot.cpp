#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/flash.h"

#include "wifi_connection.cpp"
#include "usb_connection.c"
#include "udp_server.hpp"
#include "servo2040.hpp"
#include "gaits.h"

using namespace servo;

char ssid[64], pw[64];
const int port = 12345;

const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_18;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

repeating_timer timer;

Calibration *calibration[NUM_SERVOS];
ServoCluster *servo_cluster;
inverse_kinematics *i_k = new inverse_kinematics(servo_cluster);
wifi_connection *wifi = new wifi_connection();
udp_server *server = new udp_server();
gaits *gait;
received_joystick_data *joy_data;

int x, y, z, roll, pitch, yaw;

bool adc_callback(repeating_timer_t *rt){
    const float conversion_factor = 3.3f / (1 << 12);
    adc_select_input(1);
    uint16_t consumtion_res = adc_read();
    adc_select_input(0);
    uint16_t batt_voltage_res = adc_read();
    adc_select_input(4);
    uint16_t mcu_temp_res = adc_read();
    printf("Consumption: %.2fA, Batt: %.2fV, MCU Temperature: %.1f°C\n",  
            (((float)consumtion_res * conversion_factor) - 1.65f) / 0.09f,  
            (float)batt_voltage_res * conversion_factor * 8.5f,
            27 - ((((float)mcu_temp_res * conversion_factor) - 0.706)/0.001721));
    return true;
}

void core1_entry(){ //calculate inverse kinematics on core1
    while (true){
        multicore_fifo_pop_blocking();
        x = joy_data->x1;
        y = joy_data->y1;
        z = -20.0f + joy_data->z1;
        roll = joy_data->roll;
        pitch = joy_data->pitch;
        yaw = joy_data->yaw;
        gait->move(0, x, y, z);
        //auto recv_ip = server->get_recv_ip();
        //server->send_data(&recv_ip, port, (char *)joy_data, sizeof(received_joystick_data));
        if (multicore_fifo_rvalid()){
            multicore_fifo_drain();
        }
    }
}

int main(){
    stdio_init_all();
    tusb_init();
    adc_init();
    multicore_reset_core1();
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    for (size_t i = 0; i < NUM_SERVOS; i++){
        servo_cluster->calibration(i).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }
    servo_cluster->enable_all();
    servo_cluster->all_to_value(90.0f);

    joy_data = new received_joystick_data();
    i_k = new inverse_kinematics(servo_cluster);
    gait = new gaits(i_k);

    setup_amp_sensor();
    setup_voltage_sensor();
    setup_temp_sensor();

    sleep_ms(3000);
    send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    send_and_get_char_from_tinyusb("Enter Password: ", pw);

    multicore_launch_core1(core1_entry);

    //add_repeating_timer_ms(1000, adc_callback, NULL, &timer);
    
    wifi->connect_wifi(ssid, pw);
    server->start_udp_server(12345, joy_data);

    return 0;
}