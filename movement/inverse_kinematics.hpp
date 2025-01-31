#include "kinematics.hpp"

class inverse_kinematics: public Kinematics
{
private:
    const float basic_leg_length = 65.0f;
    const float sin60 = 0.86602540378f;
    const float sin30 = 0.5f;
    const float cos30 = 0.86602540378f;
    const float cos60 = 0.5f;
    const float center_to_leg = 64.25f;
    const float sin60_to_center_leg = 55.6421f;
    const float center_to_leg_half = center_to_leg / 2;
    
public:
    inverse_kinematics(ServoCluster *servos): Kinematics(servos) {};
    void body_kinematics(int x, int y, int z, int roll_x, int pitch_y, int yaw_z, int leg_num); 
    ~inverse_kinematics() {};
};