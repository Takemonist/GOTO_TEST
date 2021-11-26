#ifndef MOVEMENT_MOTOR
#define MOVEMENT_MOTOR

#include "mbed.h"
#include <vector>

namespace ARRC{
 
class Motor{
public:
    Motor(PinName plus,PinName minus,int period,double max,double min,int ID);
    void setPWM(double pwm);
    unsigned getID();
    static std::vector<Motor*> mtrs; 
    static unsigned count;
private:
    PwmOut Plus;
    PwmOut Minus;

    double Max;
    double Min;
    unsigned ID;
};

}
#endif