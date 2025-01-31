#include "servo2040.hpp"

using namespace servo;

class Kinematics{

private:
    ServoCluster *servos;
    const float L1 = 24.8f;
    const float L2 = 37.0f;
    const float L3 = 66.5f;
    float r1, r2, r3, phi2, phi3, theta1, theta2, theta3;

public:
    Kinematics(ServoCluster *servo_cluster): servos(servo_cluster) {};    
    void endpoint(float x, float y, float z, int leg_num);
    ~Kinematics();
};