#include "pid.hpp"

namespace ARRC
{

PID::PID(double Kp,double Ki,double Kd,double max,double min,int ID,bool addmv):Kp(Kp),Ki(Ki),Kd(Kd),Max(max),Min(min),ID(ID),mv(0.0),integral(0.0),addmv(addmv)
{
    e[0] = 0.0;
    e[1] = 0.0;
}
void PID::update(double val,double tgt,double dt)
{
    e[1] = tgt - val;
    P = e[1];
    integral += (e[1]+e[0])*dt/2.0;
    I = integral;
    D = (e[1]-e[0])/dt;
    if(addmv){
        if(tgt == val) mv += 0.0;
        else mv += ((Kp*P)+(Ki*I)+(Kd*D));
    }
    else {
        if(tgt == val) mv = 0.0;
        else mv = ((Kp*P)+(Ki*I)+(Kd*D));
    }
    e[0] = e[1];
    if(mv<Min) mv = Min;
    if(mv>Max) mv = Max;
    if(integral<Min) integral = Min;
    if(integral>Max) integral = Max;
}
double PID::getmv() {return mv;}
unsigned PID::getID()   {return ID;}

}