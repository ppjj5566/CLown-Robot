#include "kinematics.hpp"
#include <cmath>

class inverse_kinematics{
private:
    Kinematics *kinematics1, *kinematics2, *kinematics3, *kinematics4, *kinematics5, *kinematics6;
    const float basic_leg_length = 65.0f;
    const float sin60 = 0.86602540378f;
    const float sin30 = 0.5f;
    const float cos30 = 0.86602540378f;
    const float cos60 = 0.5f;
    const float center_to_leg = 64.25f;
    const float sin60_to_center_leg = 55.6421f;
    const float center_to_leg_half = center_to_leg / 2;

public:
    inverse_kinematics(Kinematics *kinematics1, Kinematics *kinematics2,
    Kinematics *kinematics3, Kinematics *kinematics4, 
    Kinematics *kinematics5, Kinematics *kinematics6)
     : kinematics1(kinematics1), kinematics2(kinematics2), 
     kinematics3(kinematics3), kinematics4(kinematics4), 
     kinematics5(kinematics5), kinematics6(kinematics6) {};

    void inline body_inverse_kinematics(int x, int y, int z, int roll_x, int pitch_y, int yaw_z); // without inline keyword, nultidefine error occurs
    ~inverse_kinematics() {};
};


void inverse_kinematics::body_inverse_kinematics(int x, int y, int z, int roll_x, int pitch_y, int yaw_z){
    float roll = tan(roll_x * (M_PI / 180.0f));
    float pitch = tan(pitch_y * (M_PI / 180.0f));
    float yaw = center_to_leg * sin((yaw_z * 2) * (M_PI / 180.0f));
    float yaw_cos = center_to_leg - (center_to_leg * cos((yaw_z) * (M_PI / 180.0f)));
    
    kinematics1->endpoint(((cos60 * x + yaw) + (sin60 * y)),  (basic_leg_length + (cos60 * y)) - (sin60 * x) - yaw_cos, z + center_to_leg_half * roll + (sin60_to_center_leg * pitch));
    kinematics2->endpoint((float) x + yaw,                     basic_leg_length + y - yaw_cos,                          z + center_to_leg * roll);
    kinematics3->endpoint(((cos60 * x + yaw) - (sin60 * y)),  (basic_leg_length + (cos60 * y)) + (sin60 * x) - yaw_cos, z + center_to_leg_half * roll -  (sin60_to_center_leg * pitch));
    kinematics4->endpoint(((cos60 * -x + yaw) - (sin60 * y)), (basic_leg_length - (cos60 * y)) + (sin60 * x) - yaw_cos, z - center_to_leg_half * roll -  (sin60_to_center_leg * pitch));
    kinematics5->endpoint((float) -x + yaw,                    basic_leg_length - y - yaw_cos,                          z - center_to_leg * roll);    
    kinematics6->endpoint(((cos60 * -x + yaw) + (sin60 * y)), (basic_leg_length - (cos60 * y)) - (sin60 * x) - yaw_cos, z - center_to_leg_half * roll + (sin60_to_center_leg * pitch));
}