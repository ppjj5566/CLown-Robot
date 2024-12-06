#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"

#include "wifi_connection.cpp"
#include "inverse_kinematics.cpp"
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
Kinematics *kinematics1, *kinematics2, *kinematics3, *kinematics4, *kinematics5, *kinematics6;
inverse_kinematics *ik;
received_joystick_data *joy_data;

const float basic_leg_length = 65.0f;
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

void core1_entry(){
    while (true)
    {
        int x, y, z, roll, pitch, yaw;
        multicore_fifo_pop_blocking();
        x = joy_data->x1;
        y = joy_data->y1;
        z = -20.0f + joy_data->z1;
        roll = joy_data->roll;
        pitch = joy_data->pitch;
        yaw = joy_data->yaw;

        ik->body_inverse_kinematics(x, y, z, roll, pitch, yaw);
        // kinematics1->endpoint((cos60 * x) + (sin60 * y), (basic_leg_length + (cos60 * y)) - (sin60 * x), z);
        // kinematics2->endpoint(x, basic_leg_length + y, z);
        // kinematics3->endpoint((cos60 * x) - (sin60 * y), (basic_leg_length + (cos60 * y)) + (sin60 * x), z);
        // kinematics4->endpoint((cos60 * -x) - (sin60 * y), (basic_leg_length - (cos60 * y)) + (sin60 * x), z);
        // kinematics5->endpoint(-x, basic_leg_length - y, z);
        // kinematics6->endpoint((cos60 * -x) + (sin60 * y), (basic_leg_length - (cos60 * y)) - (sin60 * x), z);
        multicore_fifo_drain();
    }
}

int main(){

    stdio_init_all();
    tusb_init();
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    sleep_ms(3000);
    send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    send_and_get_char_from_tinyusb("Enter Password: ", pw);
    
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    servo_cluster->enable_all();

    joy_data = new received_joystick_data();

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
    ik = new inverse_kinematics(kinematics1, kinematics2, kinematics3, kinematics4, kinematics5, kinematics6);

    wifi.connect_wifi(ssid, pw);
    server.start_udp_server(12345, joy_data);

    return 0;
}