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

#include "thread_safe_udp_server.c"
#include "servo2040.hpp"
#include "gaits.h"

using namespace servo;
using namespace plasma;

// char ssid[64], pw[64];

received_joystick_data *joy_data = new received_joystick_data();
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
gaits *gait;

// LED control structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool is_moving;
    uint32_t move_interval;
} led_control_t;

led_control_t led_control = {255, 0, 0, true, 1000}; // Default: red, moving, 1 second interval

// Task handles for control
TaskHandle_t handleA, handleB, handleC, handleD, handleE;
sys_mutex_t *udp_mutex;

void led_console_task(void *pvParameters) {
    char input[32];
    int input_idx = 0;
    
    printf("\nLED Control Console\n");
    printf("Commands:\n");
    printf("  color R G B  - Set LED color (0-255)\n");
    printf("  move on/off  - Enable/disable LED movement\n");
    printf("  speed X      - Set movement interval in ms\n");
    printf("  test         - Test all LEDs\n");
    printf("  block TASK   - Block a task (server, movement, adc, led, console)\n");
    printf("  unblock TASK - Unblock a task\n");
    printf("  tasks        - Show task status\n");
    printf("  help         - Show this help\n");
    
    while (true) {
        if (input_idx < sizeof(input) - 1) {
            int c = getchar_timeout_us(0);
            if (c != PICO_ERROR_TIMEOUT) {
                if (c == '\r' || c == '\n') {
                    input[input_idx] = '\0';
                    
                    // Process command
                    if (strncmp(input, "color ", 6) == 0) {
                        int r, g, b;
                        if (sscanf(input + 6, "%d %d %d", &r, &g, &b) == 3) {
                            led_control.r = r;
                            led_control.g = g;
                            led_control.b = b;
                            printf("Color set to R:%d G:%d B:%d\n", r, g, b);
                        } else {
                            printf("Invalid color format. Use: color R G B\n");
                        }
                    }
                    else if (strncmp(input, "move", 5) == 0) {
                        if (strcmp(input + 5, "on") == 0) {
                            led_control.is_moving = true;
                            printf("LED movement enabled\n");
                        }
                        else if (strcmp(input + 5, "off") == 0) {
                            led_control.is_moving = false;
                            printf("LED movement disabled\n");
                        }
                        else {
                            printf("Invalid move command. Use: move on/off\n");
                        }
                    }
                    else if (strncmp(input, "speed", 6) == 0) {
                        int speed;
                        if (sscanf(input + 6, "%d", &speed) == 1) {
                            led_control.move_interval = speed;
                            printf("Movement interval set to %d ms\n", speed);
                        } else {
                            printf("Invalid speed format. Use: speed X\n");
                        }
                    }
                    else if (strcmp(input, "test") == 0) {
                        printf("Testing all LEDs...\n");
                        for (int i = 0; i < servo2040::NUM_LEDS; i++) {
                            led_bar.set_rgb(i, 255, 255, 255);
                        }
                        led_bar.update();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        for (int i = 0; i < servo2040::NUM_LEDS; i++) {
                            led_bar.set_rgb(i, 0, 0, 0);
                        }
                        led_bar.update();
                    }
                    else if (strncmp(input, "block ", 6) == 0) {
                        char* task = input + 6;
                        if (strcmp(task, "server") == 0) {
                            vTaskSuspend(handleA);
                            printf("Server task blocked\n");
                        }
                        else if (strcmp(task, "movement") == 0) {
                            vTaskSuspend(handleB);
                            printf("Movement task blocked\n");
                        }
                        else if (strcmp(task, "adc") == 0) {
                            vTaskSuspend(handleC);
                            printf("ADC task blocked\n");
                        }
                        else if (strcmp(task, "led") == 0) {
                            vTaskSuspend(handleD);
                            printf("LED task blocked\n");
                        }
                        else if (strcmp(task, "console") == 0) {
                            vTaskSuspend(handleE);
                            printf("Console task blocked\n");
                        }
                        else {
                            printf("Invalid task name. Use: server, movement, adc, led, console\n");
                        }
                    }
                    else if (strncmp(input, "unblock ", 8) == 0) {
                        char* task = input + 8;
                        if (strcmp(task, "server") == 0) {
                            vTaskResume(handleA);
                            printf("Server task unblocked\n");
                        }
                        else if (strcmp(task, "movement") == 0) {
                            vTaskResume(handleB);
                            printf("Movement task unblocked\n");
                        }
                        else if (strcmp(task, "adc") == 0) {
                            vTaskResume(handleC);
                            printf("ADC task unblocked\n");
                        }
                        else if (strcmp(task, "led") == 0) {
                            vTaskResume(handleD);
                            printf("LED task unblocked\n");
                        }
                        else if (strcmp(task, "console") == 0) {
                            vTaskResume(handleE);
                            printf("Console task unblocked\n");
                        }
                        else {
                            printf("Invalid task name. Use: server, movement, adc, led, console\n");
                        }
                    }
                    else if (strcmp(input, "tasks") == 0) {
                        printf("\nTask Status:\n");
                        printf("----------------------------------------\n");
                        printf("Task Name      | State     | Priority\n");
                        printf("----------------------------------------\n");
                        
                        // Server Task
                        printf("Server         | %-9s | %d\n", 
                               eTaskGetState(handleA) == eSuspended ? "Blocked" : 
                               eTaskGetState(handleA) == eRunning ? "Running" : "Other",
                               uxTaskPriorityGet(handleA));
                        
                        // Movement Task
                        printf("Movement       | %-9s | %d\n", 
                               eTaskGetState(handleB) == eSuspended ? "Blocked" : 
                               eTaskGetState(handleB) == eRunning ? "Running" : "Other",
                               uxTaskPriorityGet(handleB));
                        
                        // ADC Task
                        printf("ADC            | %-9s | %d\n", 
                               eTaskGetState(handleC) == eSuspended ? "Blocked" : 
                               eTaskGetState(handleC) == eRunning ? "Running" : "Other",
                               uxTaskPriorityGet(handleC));
                        
                        // LED Task
                        printf("LED            | %-9s | %d\n", 
                               eTaskGetState(handleD) == eSuspended ? "Blocked" : 
                               eTaskGetState(handleD) == eRunning ? "Running" : "Other",
                               uxTaskPriorityGet(handleD));
                        
                        // Console Task
                        printf("Console        | %-9s | %d\n", 
                               eTaskGetState(handleE) == eSuspended ? "Blocked" : 
                               eTaskGetState(handleE) == eRunning ? "Running" : "Other",
                               uxTaskPriorityGet(handleE));
                        
                        printf("----------------------------------------\n");
                        printf("Note: Priority 4 is highest, 1 is lowest\n");
                    }
                    else if (strcmp(input, "help") == 0) {
                        printf("Commands:\n");
                        printf("  color R G B  - Set LED color (0-255)\n");
                        printf("  move on/off  - Enable/disable LED movement\n");
                        printf("  speed X      - Set movement interval in ms\n");
                        printf("  test         - Test all LEDs\n");
                        printf("  block TASK   - Block a task (server, movement, adc, led, console)\n");
                        printf("  unblock TASK - Unblock a task\n");
                        printf("  tasks        - Show task status\n");
                        printf("  help         - Show this help\n");
                    }
                    else {
                        printf("Unknown command. Type 'help' for available commands.\n");
                    }
                    
                    input_idx = 0;
                    printf("> ");
                }
                else {
                    input[input_idx++] = c;
                    putchar(c);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void neo_pixel_task(void *pvParameters)
{
    uint led_count = 0;
    uint32_t counter = 0;
    const uint32_t MOVE_COUNT = 33; // 33 * 30ms ≈ 1 second
    
    printf("Starting LED task...\n");
    printf("Number of LEDs: %d\n", servo2040::NUM_LEDS);
    
    // Initialize LED strip
    led_bar.start();
    printf("LED strip initialized\n");
    
    while (true)
    {
        counter++;
        
        // Check if it's time to move to next LED
        if (counter >= MOVE_COUNT && led_control.is_moving) {
            // Clear all LEDs first
            for (int i = 0; i < servo2040::NUM_LEDS; i++) {
                led_bar.set_rgb(i, 0, 0, 0);
            }
            
            // Light up the current LED with current color
            led_bar.set_rgb(led_count, led_control.r, led_control.g, led_control.b);
            
            // Update the LED strip
            led_bar.update();
            
            // Move to next LED
            led_count = (led_count + 1) % servo2040::NUM_LEDS;
            
            // Reset counter
            counter = 0;
        }
        
        // Main loop delay of 30ms
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void adc_task(void *pvParameters)
{
    setup_amp_sensor();
    setup_voltage_sensor();
    setup_temp_sensor();
    sys_mutex_t *mutex = (sys_mutex_t *)pvParameters;
    const float conversion_factor = 3.3f / (1 << 12);
    struct pbuf *p;
    ip_addr_t dest_addr;
    IP4_ADDR(&dest_addr, 192, 168, 0, 29);
    TickType_t xLastWakeTime;

    printf("\nADC Monitoring Started\n");
    printf("----------------------------------------\n");
    printf("Time(ms) | I(A) | V(V) | T(°C)\n");
    printf("----------------------------------------\n");

    while (true)
    {
        char buffer[100];
        adc_select_input(1);
        uint16_t result = adc_read();
        adc_select_input(0);
        uint16_t result1 = adc_read();
        adc_select_input(4);
        uint16_t result2 = adc_read();

        float current = (((float)result * conversion_factor) - 1.65f) / 0.09f;
        float voltage = (float)result1 * conversion_factor * 8.5f;
        float temp = 27 - ((((float)result2 * conversion_factor) - 0.706) / 0.001721);

        // Format the values with fixed width and precision
        sprintf(buffer, "%7lu | %4.2f | %4.2f | %4.1f\n",
                to_ms_since_boot(get_absolute_time()),
                current, voltage, temp);
        printf("%s", buffer);

        sys_mutex_lock(mutex);
        p = pbuf_alloc(PBUF_TRANSPORT, strlen(buffer), PBUF_RAM);
        if (p != NULL)
        {
            memcpy(p->payload, buffer, strlen(buffer));
            udp_sendto(pcb, p, &dest_addr, UDP_SEND_PORT);
            pbuf_free(p);
        }
        else
        {
            printf("Failed to allocate pbuf\n");
        }
        sys_mutex_unlock(mutex);
        xLastWakeTime = xTaskGetTickCount();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
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
    // servo_cluster->all_to_mid();

    inverse_kinematics *i_k = new inverse_kinematics(servo_cluster);
    gait = new gaits(i_k);
}

void movement_order_task(void *pvParameters)
{
    init_servos();
    gait->stop();

    while (true)
    {
        switch (joy_data->mode)
        {
        case 0:
            gait->move(joy_data);
            break;
        default:
            break;
        }
    }
}

int main()
{
    stdio_init_all();
    adc_init();
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return -1;
    }
    printf("cyw43 initialised\n");

    sys_mutex_new(udp_mutex);

    xTaskCreate(udp_task, "server_task", 1024, joy_data, 4, &handleA);
    xTaskCreate(movement_order_task, "movement_order_task", 2048, NULL, 3, &handleB);
    xTaskCreate(adc_task, "adc_task", 256, udp_mutex, 2, &handleC);
    xTaskCreate(neo_pixel_task, "neo_pixel_task", 512, NULL, 1, &handleD);
    xTaskCreate(led_console_task, "led_console_task", 512, NULL, 1, &handleE);

    vTaskCoreAffinitySet(handleA, (1 << 0));
    vTaskCoreAffinitySet(handleB, (1 << 1));

    vTaskStartScheduler();

    return 0;
}