#include "pico/stdlib.h"
#include <stdio.h>

#include "servo2040.hpp"
#include "pico/multicore.h"

using namespace servo;

const uint START_PIN = 2;
const uint END_PIN = 19;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
Servo *servos[NUM_SERVOS];
Calibration *calibration[NUM_SERVOS];

float servo_state[6][3] = { // coxa tibia femur
    {-5.0f,10.0f,-5.0f},//1
    {5.0f,-5.0f,0.0f},//2
    {-3.0f,-5.0f,0.0f},//3
    {0.0f,0.0f,0.0f},//4
    {0.0f,-10.0f,-10.0f},//6 ???
    {-5.0f,-10.0f,-5.0f},//5 ???
    };

int main(){
    stdio_init_all();
    for(auto s = 0u; s < NUM_SERVOS; s++) {
        servos[s] = new Servo(s + START_PIN);
        calibration[s] = &servos[s]->calibration();
        calibration[s]->apply_three_pairs(460.0f, 1430.0f, 2400.0f, 0.0f, 90.0f, 180.0f);
        servos[s]->init();
        servos[s]->enable();
    }

    while(true){
        for(auto s = 0u; s < NUM_SERVOS - 1 && (s / 3) < 6; s++) {
            servos[s]->value(90.0f + servo_state[s / 3][s % 3]);
        }
        sleep_ms(1000);
    }
}