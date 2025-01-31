#include "pico/stdlib.h"
#include "tusb.h"

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
