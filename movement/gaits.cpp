#include "gaits.h"

int gaits::lerp(int start, int end, int t){
    float ft = t/30.0f;
    float endpoint = start + ((end - start) * ft);
    return (int) endpoint;
}

int gaits::bazier_curve(int start, int end, int height, int t){
    int a = lerp(start, height, t);
    int b = lerp(height, end, t);
    return lerp(a, b, t);
}

void gaits::move(int gait, int x, int y, int z){
    this->gait = gait;
    switch (gait){
    case 0:
        if(sequence){
            for (int t = 0; t <= 30; t++){
                int position_x = lerp(last_position.x, x, t);
                int position_y = lerp(last_position.y, y, t);
                int position_z = bazier_curve(last_position.z, z, last_position.z + 30, t);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 0);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 1);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 2);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 3);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 4);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 5);
                sleep_ms(5);
                }
            last_position.set(x, y, z);
            this->sequence = false; 
        }
        else{
            for (int t = 0; t <= 30; t++){
                int position_x = lerp(-last_position.x, x, t);
                int position_y = lerp(-last_position.y, y, t);
                int position_z = bazier_curve(last_position.z, z, last_position.z + 50, t);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 0);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 1);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 2);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 3);
                i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 4);
                i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 5);
                sleep_ms(5);
            }
            last_position.set(-x, -y, z);
            this->sequence = true;
        }
        break;
    }
}
