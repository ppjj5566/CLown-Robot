#include <stdio.h>
#include "pico/stdlib.h"
// #include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/flash.h"
#include "usb_connection.c"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"

#include "wifi_connection.cpp"
#include "udp_server.hpp"
#include "servo2040.hpp"
#include "gaits.h"

using namespace servo;

char ssid[64], pw[64];
const int port = 12345;

const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_18;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

extern xSemaphoreHandle mutex;
Calibration *calibration[NUM_SERVOS];
wifi_connection *wifi = new wifi_connection();
udp_server *server = new udp_server();
received_joystick_data *joy_data = new received_joystick_data();

ServoCluster *servo_cluster;

inverse_kinematics *i_k;
static gaits *gait;
int x, y, z, roll, pitch, yaw;

void adc_task(void *pvParameters)
{
    setup_amp_sensor();
    setup_voltage_sensor();
    setup_temp_sensor();

    while (true)
    {
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
               27 - ((((float)result2 * conversion_factor) - 0.706) / 0.001721));
        vTaskDelay(5000);
    }
}

void server_task(void *pvParameters)
{
    wifi->connect_wifi("ipiptime", "Park98124");
    server->udp_server_task(joy_data);
}

void init_servos()
{
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();

    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
        servo_cluster->calibration(i).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }

    servo_cluster->enable_all();

    i_k = new inverse_kinematics(servo_cluster);
    gait = new gaits(i_k);
}

void movement_order_task(void *pvParameters)
{
    init_servos();

    while (true)
    {
        x = joy_data->x1;
        y = joy_data->y1;
        z = joy_data->z1;
        roll = joy_data->roll;
        pitch = joy_data->pitch;
        yaw = joy_data->yaw;
        if (x != 0 || y != 0 || z != 0)
            gait->move(joy_data);
        // auto recv_ip = server->get_recv_ip();
        // server->send_data(&recv_ip, port, (char *)joy_data, sizeof(received_joystick_data));
    }
}

int main()
{
    stdio_init_all();
    adc_init();

    // send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    // send_and_get_char_from_tinyusb("Enter Password: ", pw);
    

    xTaskCreate(server_task, "server_task", 1024, NULL, 0, NULL);
    xTaskCreate(movement_order_task, "movement_order_task", 1024, NULL, 1, NULL);
    xTaskCreate(adc_task, "adc_task", 1024, NULL, 2, NULL);
    vTaskStartScheduler();

    while (true)
    {
    }

    return 0;
}