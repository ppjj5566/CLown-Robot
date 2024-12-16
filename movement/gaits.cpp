#include "inverse_kinematics.cpp"

class gaits
{
private:
    bool sequence;
    int gait;
    inverse_kinematics *ik;
    struct gaits_last_position_data{
        int x, y, z, roll, pitch, yaw;
        void set(int x, int y, int z){
            this->x = x;
            this->y = y;
            this->z = z;
        }
    } last_position;

public:
    gaits(inverse_kinematics *ik);
    inline void move(int gaits, int x, int y, int z);
    //void move(int gaits, int x, int y, int z, int roll, int pitch, int yaw);
    int lerp(int start, int end, int t);
    int bazier_curve(int start, int end, int height, int t);
    ~gaits();
};

gaits::gaits(inverse_kinematics *ik){
    sequence = true;
    this->ik = ik;
    last_position.set(0, 0, 0);
}

int gaits::lerp(int start, int end, int t){
    return start + (end - start) * (t/10);
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
            for (int t = 0; t <= 10; t++){
                int position_x = lerp(last_position.x, x, t);
                int position_y = lerp(last_position.y, y, t);
                int position_z = bazier_curve(last_position.z, z, last_position.z + 30, t);
                ik->leg_inverse_kinematics(0, -position_x, -position_y, position_z, 0, 0, 0);
                ik->leg_inverse_kinematics(1, position_x, position_y, z, 0, 0, 0);
                ik->leg_inverse_kinematics(2, -position_x, -position_y, position_z, 0, 0, 0);
                ik->leg_inverse_kinematics(3, position_x, position_y, z, 0, 0, 0);
                ik->leg_inverse_kinematics(4, -position_x, -position_y, position_z, 0, 0, 0);
                ik->leg_inverse_kinematics(5, position_x, position_y, z, 0, 0, 0);
                sleep_ms(200);
                }
            last_position.set(x, y, z);
            this->sequence = false;
        }
        else{
            for (int t = 0; t <= 10; t++){
                int position_x = lerp(last_position.x, x, t);
                int position_y = lerp(last_position.y, y, t);
                int position_z = bazier_curve(last_position.z, z, last_position.z + 30, t);
                ik->leg_inverse_kinematics(0, position_x, position_y, z, 0, 0, 0);
                ik->leg_inverse_kinematics(1, -position_x, -position_y, position_z, 0, 0, 0);
                ik->leg_inverse_kinematics(2, position_x, position_y, z, 0, 0, 0);
                ik->leg_inverse_kinematics(3, -position_x, -position_y, position_z, 0, 0, 0);
                ik->leg_inverse_kinematics(4, position_x, position_y, z, 0, 0, 0);
                ik->leg_inverse_kinematics(5, -position_x, -position_y, position_z, 0, 0, 0);
                sleep_ms(200);
            }
            last_position.set(x, y, z);
            this->sequence = true;
        }
        break;
    }
}

gaits::~gaits(){
}
