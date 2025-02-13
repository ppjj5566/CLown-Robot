#ifndef RECEIVED_JOYSTICK_DATA_H
#define RECEIVED_JOYSTICK_DATA_H // 한번만 정의해주기 위한 헤더가드

struct received_joystick_data{
    volatile int x1, y1, z1, pitch, roll, yaw;
    volatile int mode, connection_mode, led_data;
};

#endif // RECEIVED_JOYSTICK_DATA_H