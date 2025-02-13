#include "gaits.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

SemaphoreHandle_t gaits_mutex = NULL;

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

void gaits::move(received_joystick_data *joy_data){
    while (true){
        if (xSemaphoreTake(gaits_mutex, 0) == pdTRUE){
            this->gait = gait;
            switch (gait){
                case 0:
                    if(sequence){
                        for (int t = 0; t <= 30; t++){
                            int position_x = lerp(last_position.x, joy_data->x1 , t);
                            int position_y = lerp(last_position.y, joy_data->y1, t);
                            int position_z = bazier_curve(last_position.z, joy_data->z1, last_position.z + 30, t);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 0);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 1);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 2);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 3);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 4);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 5);
                            vTaskDelay(5);
                            }
                        last_position.set(joy_data->x1, joy_data->y1, joy_data->z1);
                        this->sequence = false; 
                        }
                    else{
                        for (int t = 0; t <= 30; t++){
                            int position_x = lerp(-last_position.x, joy_data->x1, t);
                            int position_y = lerp(-last_position.y, joy_data->y1, t);
                            int position_z = bazier_curve(last_position.z, joy_data->z1, last_position.z + 50, t);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 0);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 1);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 2);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 3);
                            i_k->body_kinematics(position_x, position_y, position_z, 0, 0, 0, 4);
                            i_k->body_kinematics(-position_x, -position_y, last_position.z, 0, 0, 0, 5);
                            vTaskDelay(5);
                            }
                        last_position.set(-joy_data->x1, -joy_data->y1, joy_data->z1);
                        this->sequence = true;
                        }
                break;
            }
        }
    }
}
