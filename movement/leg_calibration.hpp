#include "servo2040.hpp"

class leg_calibration
{
private:
    Servo *servos;
public:
    leg_calibration(Servo **servos);
    ~leg_calibration(){};
};