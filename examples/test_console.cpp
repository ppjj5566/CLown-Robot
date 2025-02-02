#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "tusb.h"
#include "hardware/flash.h"
#define FLASH_TARGET_OFFSET (256 * 1024)

using namespace servo;

const uint START_PIN = 2;
const uint END_PIN = 19;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

ServoCluster *servo_cluster;
Calibration *calibration[NUM_SERVOS];

int servo_state[6][3] = { // coxa tibia femur
    {0, 0, 0},//1
    {0, 0, 0},//2
    {0, 0, 0},//3
    {0, 0, 0},//4
    {0, 0, 0},//5
    {0, 0, 0},//6
    };

void flash_data_in_memory(){
    printf("Erasing target region...\n");
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_PAGE_SIZE);
}


void send_char_to_tinyusb(const char *message){
    while (*message){
        tud_cdc_write_char(*message++);
        sleep_us(100);
    }
    tud_cdc_write_flush();
}

void menu_page(){
    send_char_to_tinyusb("--select the menu--\n");
    send_char_to_tinyusb("c: print name of company\n");
}

void crown_robot(){
    send_char_to_tinyusb(
        " .d8888b.                                              8888888b.          888               888    \n"
        "d88P  Y88b                                             888   Y88b         888               888    \n"
        "888    888                                             888    888         888               888    \n"
        "888        888d888 .d88b.  888  888  888 88888b.       888   d88P .d88b.  88888b.   .d88b.  888888 \n"
        "888        888P\"  d88\"\"88b 888  888  888 888 \"88b      8888888P\" d88\"\"88b 888 \"88b d88\"\"88b 888\n"
        "888    888 888    888  888 888  888  888 888  888      888 T88b  888  888 888  888 888  888 888    \n"
        "Y88b  d88P 888    Y88..88P Y88b 888 d88P 888  888      888  T88b Y88..88P 888 d88P Y88..88P Y88b.  \n"
        " \"Y8888P\"  888     \"Y88P\"   \"Y8888888P\"  888  888      888   T88b \"Y88P\"  88888P\"   \"Y88P\"   \"Y888 \n"
        );
}

void ro_boy(){
    send_char_to_tinyusb(
    "                        _/_/_/                     \n"
    "                       _/    _/    _/_/            \n"
    "                      _/_/_/    _/    _/           \n"
    "                     _/    _/  _/    _/            \n"
    "                    _/    _/    _/_/               \n"
    "                        _/_/_/                     \n"
    "                       _/    _/    _/_/    _/    _/\n"
    "                      _/_/_/    _/    _/  _/    _/ \n"
    "                     _/    _/  _/    _/  _/    _/  \n"
    "                    _/_/_/      _/_/      _/_/_/   \n"
    "                                             _/    \n"
    "                                        _/_/       \n"
    );
}

void crown_robot_servo_angle_status(){
    tud_cdc_read_flush();
    char buffer[1200];
    snprintf(buffer, sizeof(buffer),
        "                             ▄                 ▄            \n"
        "                           %u \\              / %u          \n"
        "                               \\            /              \n"
        "                             %u ▄           ▄ %u            \n"
        "                                 \\        /                \n"
        "                               %u @ ──── @ %u               \n"
        "                                 / 1    6 \\                \n"
        "                                /          \\               \n"
        "                               /            \\              \n"
        "                 -- ▄ ─ ─ ▄ ─ @ 2          5 @ ─ ▄ ─ ─ ▄ -- \n"
        "                    %u    %u %u\\            / %u  %u    %u \n"
        "                                \\          /               \n"
        "                                 \\ 3    4 /                \n"
        "                               %u @ ──── @ %u               \n"
        "                                 /        \\                \n"
        "                             %u ▄          ▄ %u             \n"
        "                               /            \\              \n"
        "                          %u ▄/              \\▄ %u         \n",
        90 + servo_state[0][2], 90 + servo_state[5][2], 90 + servo_state[0][1],
        90 + servo_state[5][1], 90 + servo_state[0][0], 90 + servo_state[5][0],
        90 + servo_state[1][2], 90 + servo_state[1][1], 90 + servo_state[1][0],
        90 + servo_state[4][0], 90 + servo_state[4][1], 90 + servo_state[4][2],
        90 + servo_state[2][0], 90 + servo_state[3][0], 90 + servo_state[2][1],
        90 + servo_state[3][1], 90 + servo_state[2][2], 90 + servo_state[3][2]
        );
    send_char_to_tinyusb(buffer);
    send_char_to_tinyusb("Enter leg number (1-6): \n");
}

void leg_calibration_status(uint leg_number){
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
        "  Leg Number: %u \n"
        "                %u        \n"
        "┌─ ─ ─ ┐       @          \n"
        "│      │      /  \\       \n"
        "└─ ─ ┬ ┘ %u  /    \\      \n"
        "     ├ ─ ─ ─/      \\     \n"
        "     │     @│ %u    \\    \n"
        "     └ ─ ─ ─┘        \\   \n"
        "                      \\  \n"
        "                       \\ \n"
        "                        \\\n",
        leg_number + 1,
        90 + servo_state[leg_number][2],
        90 + servo_state[leg_number][0],
        90 + servo_state[leg_number][1]
        );
    send_char_to_tinyusb(buffer);
    send_char_to_tinyusb("Coxa: ");
    snprintf(buffer, sizeof(buffer), "%d", 90 + servo_state[leg_number][0]);
    send_char_to_tinyusb(buffer);
    send_char_to_tinyusb("\nTibia: ");
    snprintf(buffer, sizeof(buffer), "%d", 90 + servo_state[leg_number][1]);
    send_char_to_tinyusb(buffer);
    send_char_to_tinyusb("\nFemur: ");
    snprintf(buffer, sizeof(buffer), "%d", 90 + servo_state[leg_number][2]);
    send_char_to_tinyusb(buffer);
    send_char_to_tinyusb("\nEnter the number of the servo you want to calibrate (1: Coxa, 2: Tibia, 3: femur): \n");
}

int read_integer_from_usb() {
    char buffer[16];
    uint32_t count = tud_cdc_read(buffer, sizeof(buffer) - 1);
    buffer[count] = '\0'; // Null-terminate the string
    return atoi(buffer); // Convert string to integer
}


void calibrate_servo(char *buffer, uint32_t count, int servo_state[6][3], int leg_number, int servo_number, bool &repeat){
    send_char_to_tinyusb("Enter the value of the servo: \n");
    tud_cdc_read_flush();
    while (!tud_cdc_available()){
        tud_task();
    }
    count = tud_cdc_read(buffer, sizeof(buffer) - 1);
    buffer[count] = '\0'; // Null-terminate the string
    if (buffer[0] == '\n'){
        send_char_to_tinyusb("go back to the leg_calibration_menu");
        bool repeat = false;
        return;
    }
    servo_state[leg_number][servo_number] += atoi(buffer);
    sprintf(buffer, "%d", 90 + servo_state[leg_number][servo_number]);
    send_char_to_tinyusb(buffer);
    repeat = true;
}

void set_leg_calibration(uint leg_number, int servo_number, char *buffer, uint32_t count, int servo_state[6][3], bool &repeat){
    tud_cdc_read_flush();
    leg_calibration_status(leg_number);
    while (!tud_cdc_available()){
        tud_task();
    }
    servo_number = tud_cdc_read_char();
        switch (servo_number) {
        case '1':
            calibrate_servo(buffer, count, servo_state, leg_number, 0, repeat);
            repeat = true;
            break;
        case '2':
            calibrate_servo(buffer, count, servo_state, leg_number, 1, repeat);
            repeat = true;
            break;
        case '3':
            calibrate_servo(buffer, count, servo_state, leg_number, 2, repeat);
            repeat = true;
            break;
        default:
            send_char_to_tinyusb("Invalid servo number\n");
            repeat = false;
            break;
        }
}


void repeat_servo_angle(){
    servo_cluster->enable_all();
    for(auto s = 0u; s < NUM_SERVOS; s++) {
        servo_cluster->value(s, 90.0f + (float)servo_state[s / 3][s % 3]);
    }
}

int main() {
    stdio_init_all();
    tusb_init();
    menu_page();
    servo_cluster = new ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    servo_cluster->init();
    for(auto s = 0u; s < NUM_SERVOS; s++) {
        servo_cluster->calibration(s).apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
    }

    while (true) {
        tight_loop_contents();
        if (tud_cdc_connected()) {
            if(tud_cdc_available()){
                send_char_to_tinyusb("Enter command: \n");
                char c = tud_cdc_read_char();
                char leg_number;
                int servo_number;
                char buffer[16];
                bool repeat_calib_menu;
                uint32_t count;
            switch (c){
                case 'h':
                    ro_boy();
                    servo_cluster->disable_all();
                    break;
                case 'c':
                    crown_robot_servo_angle_status();
                    repeat_servo_angle();
                    while (!tud_cdc_available()){
                        tud_task();
                    }
                    leg_number = tud_cdc_read_char();
                    do{
                       switch (leg_number) {
                        case '1':
                            repeat_calib_menu = true;
                            set_leg_calibration(0, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        case '2':
                            repeat_calib_menu = true;
                            set_leg_calibration(1, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        case '3':
                            repeat_calib_menu = true;
                            set_leg_calibration(2, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        case '4':
                            repeat_calib_menu = true;
                            set_leg_calibration(3, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        case '5':
                            repeat_calib_menu = true;
                            set_leg_calibration(4, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        case '6':
                            repeat_calib_menu = true;
                            set_leg_calibration(5, servo_number, buffer, count, servo_state, repeat_calib_menu);
                            repeat_servo_angle();
                            break;
                        default:
                            repeat_calib_menu = false;
                            send_char_to_tinyusb("Invalid leg number\n");
                            break;
                        }
                    } while (repeat_calib_menu);
                    break;
                default:
                    send_char_to_tinyusb("Invalid command\n");
                    break;
                }
            }
        }      
    }
}