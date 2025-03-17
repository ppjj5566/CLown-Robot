#include "inverse_kinematics.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "received_joystick_data.h"

class gaits
{
private:
    bool sequence;
    int gait;
    inverse_kinematics *i_k;
    struct gaits_last_position_data{
        int x, y, roll, pitch, yaw;
        int z = -30;
        void set(int x, int y, int z){
            this->x = x;
            this->y = y;
            this->z = z;
        }
        void set(int x, int y, int z, int roll, int pitch, int yaw){
            this->x = x;
            this->y = y;
            this->z = z;
            this->roll = roll;
            this->pitch = pitch;
            this->yaw = yaw;
        }
    } last_position;

public:
    gaits(inverse_kinematics *ik): i_k(ik), sequence(true), gait(0), last_position({0, 0, 0}){
        gaits_mutex = xSemaphoreCreateBinary();
    };

    template<typename T> int lerp(T start, T end, int t, int step = 30);
    template<typename T> int bazier_curve(T start, T end, T height, int t);
    void move(received_joystick_data *joy_data);
    void stop();
    ~gaits() {};
};
