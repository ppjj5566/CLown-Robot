#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "calibration.hpp"

using namespace servo;

const uint SERVO_PIN = servo2040::SERVO_1;

Servo *servos;

int main(){
    stdio_init_all();

    servos = new Servo(SERVO_PIN);
    servos->init();
    servos->enable();
    
    servos->to_min();
    sleep_ms(2000);

    servos->to_mid();
    sleep_ms(2000);
    
    servos->to_max();
    sleep_ms(2000);

    servos->disable();
return 1;
}
