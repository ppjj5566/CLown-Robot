#include "leg_calibration.hpp"

leg_calibration::leg_calibration(Servo **servos)
{
    this->servos = servos;
    for (int i = 0; i < size_t(servos); i++){
        servos[i]->value(90.0f + servo_state[i / 3][i % 3]);
    }
}