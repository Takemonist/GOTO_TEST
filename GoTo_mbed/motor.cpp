#include "motor.hpp"

namespace ARRC
{
    
std::vector<Motor*> Motor::mtrs;
unsigned Motor::count;

Motor::Motor(PinName plus,PinName minus,int period,double max,double min,int ID):ID(ID),Plus(plus),Minus(minus),Max(max),Min(min)
{
    mtrs.push_back(this);
    count++;
    Plus.period_us(period);
    Minus.period_us(period);
}
void Motor::setPWM(double pwm)
{
    if(pwm >= Max)   pwm = Max;
    if(pwm <= Min)   pwm = Min;
    if(pwm < 0.0) {
        Plus = 0.0;
        Minus = pwm*-1;
    } else {
        Minus = 0.0;
        Plus = pwm;
    }
}

unsigned Motor::getID()   {return ID;}

}