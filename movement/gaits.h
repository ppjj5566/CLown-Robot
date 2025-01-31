#include "inverse_kinematics.hpp"

class gaits
{
private:
    bool sequence;
    int gait;
    inverse_kinematics *i_k;
    struct gaits_last_position_data{
        int x, y, z, roll, pitch, yaw;
        void set(int x, int y, int z){
            this->x = x;
            this->y = y;
            this->z = z;
        }
    } last_position;

public:
    gaits(inverse_kinematics *ik): i_k(ik), sequence(true), gait(0), last_position({0, 0, 0}){};
    int lerp(int start, int end, int t);
    int bazier_curve(int start, int end, int height, int t);
    void move(int gaits, int x, int y, int z);
    ~gaits() {};
};
