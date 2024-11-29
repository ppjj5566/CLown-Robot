#pragma once

#include "servo2040.hpp"
#include "pico/stdlib.h"

using namespace servo;

class Kinematics
{
private:
    Servo *servos[3];
    int leg_num;
    const float L1 = 24.8f;
    const float L2 = 37.0f;
    const float L3 = 66.5f;
    float r1, r2, r3, phi2, phi3, theta1, theta2, theta3;

public:
    Kinematics(Servo *servo1, Servo *servo2, Servo *servo3, int leg_number);    
    void endpoint(float x, float y, float z);
    ~Kinematics();
};