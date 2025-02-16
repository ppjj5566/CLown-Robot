#include <stdio.h>

#include "kinematics.hpp"
#include "pico/stdlib.h"
//TODO: make servos a pointer array

float servo_state[6][3] = { // coxa tibia femur
    {6.0f,2.0f,10.0f},         //1
    {0.0f,-5.0f,0.0f},           //2
    {0.0f,-8.0f,3.0f},          //3
    {-7.0f,-4.0f,-7.0f},         //4
    {7.0f,4.0f,7.0f},          //6
    {-5.0f,12.0f,-7.0f},        //5
    };

void Kinematics::endpoint(float x, float y, float z, int leg_num){
    if(x != 0.0f || y != 0.0f || z != 0.0f){
        r1 = sqrt(pow(x,2) + pow(y,2));
        r2 = r1 - L1;
        r3 = sqrt(pow(r2,2) + pow(z,2));

        phi2 = acos((pow(r3,2) + pow(L2,2) - pow(L3,2))/ (2 * r3 * L2));
        phi3 = atan2(z, r2);

        theta1 = atan2(y, x) * 180.0f / M_PI;
        theta2 = (phi2 + phi3) * 180.0f / M_PI;
        theta3 = acos((pow(L2,2) + pow(L3,2) - pow(r3,2))/ (2 * L3 * L2)) * 180.0f / M_PI;

        servos->value(leg_num * 3, theta1 + servo_state[leg_num][0]);
        servos->value(leg_num * 3 + 1, theta2 + 90.0f + servo_state[leg_num][1]);
        servos->value(leg_num * 3 + 2, theta3 + servo_state[leg_num][2]);
    }
}