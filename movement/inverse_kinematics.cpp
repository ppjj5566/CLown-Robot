#include <cmath>
#include "inverse_kinematics.hpp"

void inverse_kinematics::body_kinematics(int x, int y, int z, int roll_x, int pitch_y, int yaw_z, int leg_num){ // inverse kinematics for the body
    float roll = tan(roll_x * (M_PI / 180.0f));
    float pitch = tan(pitch_y * (M_PI / 180.0f));
    float yaw = center_to_leg * sin((yaw_z * 2) * (M_PI / 180.0f));
    float yaw_cos = center_to_leg - (center_to_leg * cos((yaw_z) * (M_PI / 180.0f)));
    switch (leg_num){
        case 0:
            endpoint(((cos60 * x + yaw) + (sin60 * y)),  (basic_leg_length + (cos60 * y)) - (sin60 * x) - yaw_cos, z + center_to_leg_half * roll + (sin60_to_center_leg * pitch), leg_num); // leg1
        break;
        case 1:
            endpoint((float) x + yaw,                     basic_leg_length + y - yaw_cos,                          z + center_to_leg * roll, leg_num); // leg2
        break;
        case 2:
            endpoint(((cos60 * x + yaw) - (sin60 * y)),  (basic_leg_length + (cos60 * y)) + (sin60 * x) - yaw_cos, z + center_to_leg_half * roll -  (sin60_to_center_leg * pitch), leg_num); //leg3
        break;
        case 3:
            endpoint(((cos60 * -x + yaw) - (sin60 * y)), (basic_leg_length - (cos60 * y)) + (sin60 * x) - yaw_cos, z - center_to_leg_half * roll -  (sin60_to_center_leg * pitch), leg_num); //leg4
        break;
        case 4:
            endpoint((float) -x + yaw,                    basic_leg_length - y - yaw_cos,                          z - center_to_leg * roll, leg_num); //leg5
        break;
        case 5:
            endpoint(((cos60 * -x + yaw) + (sin60 * y)), (basic_leg_length - (cos60 * y)) - (sin60 * x) - yaw_cos, z - center_to_leg_half * roll + (sin60_to_center_leg * pitch), leg_num); // leg6
        break;
    }
}