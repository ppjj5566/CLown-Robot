#include <stdio.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/flash.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

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

// char ssid[64], pw[64];

received_joystick_data *joy_data = new received_joystick_data();

gaits *gait;

void adc_task(void *pvParameters)
{
    setup_amp_sensor();
    setup_voltage_sensor();
    setup_temp_sensor();
    const float conversion_factor = 3.3f / (1 << 12);

    while (true)
    {
        adc_select_input(1);
        uint16_t result = adc_read();
        adc_select_input(0);
        uint16_t result1 = adc_read();
        adc_select_input(4);
        uint16_t result2 = adc_read();

        float current = (((float)result * conversion_factor) - 1.65f) / 0.09f;
        float voltage = (float)result1 * conversion_factor * 8.5f;
        float temp = 27 - ((((float)result2 * conversion_factor) - 0.706) / 0.001721);

        printf("Consumption: %.2fA, Batt: %.2fV, MCU Temperature: %.1f°C\n",
               current - 1.65f, voltage * 8.5f, temp);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void server_task(void *pvParameters)
{
    wifi_connection *wifi = new wifi_connection();
    udp_server *server = new udp_server();
    wifi->connect_wifi("ipiptime", "Park98124");
    server->udp_server_task(joy_data);
}

void init_servos()
{
    const uint START_PIN = servo2040::SERVO_1;
    const uint END_PIN = servo2040::SERVO_18;
    const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

    ServoCluster *servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
        servo_cluster->calibration(i).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }
    servo_cluster->enable_all();

    inverse_kinematics *i_k = new inverse_kinematics(servo_cluster);
    gait = new gaits(i_k);
}

void movement_order_task(void *pvParameters)
{
    init_servos();
    int x, y, z, roll, pitch, yaw;
    
    while (true)
    {
        x = joy_data->x1;
        y = joy_data->y1;
        z = joy_data->z1;
        roll = joy_data->roll;
        pitch = joy_data->pitch;
        yaw = joy_data->yaw;
        if (x != 0 || y != 0)
            gait->move(joy_data);
    }
}

int main()
{
    stdio_init_all();
    adc_init();

    // send_and_get_char_from_tinyusb("Enter SSID: ", ssid);
    // send_and_get_char_from_tinyusb("Enter Password: ", pw);
    TaskHandle_t handleA, handleB;

    xTaskCreate(server_task, "server_task", 256, NULL, 0, &handleA);
    xTaskCreate(movement_order_task, "movement_order_task", 512, NULL, 0, &handleB);
    xTaskCreate(adc_task, "adc_task", 256, NULL, 1, &handleA);

    vTaskCoreAffinitySet(handleA, (1 << 0));
    vTaskCoreAffinitySet(handleB, (1 << 1));

    vTaskStartScheduler();

    return 0;
}