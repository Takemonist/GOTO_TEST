#include "imc.hpp"

namespace ARRC{
    
IMC::IMC(double K,double T,double max,double min,unsigned ID):K(K),T(T),Max(max),Min(min),ID(ID),x(0.0),mv(0.0){
}
void IMC::update(double val,double tgt,double dt){
    mv = (tgt - val + simulate(x,mv,dt))*(1.0/K);
    if(tgt == val);
    if(mv<Min) mv = Min;
    if(mv>Max) mv = Max;
}

double IMC::getmv(){return mv;}

unsigned IMC::getID(){return ID;}

double IMC::simulate(double x,double u,double dt){
    this->x = x + (-x + K*u)/T*dt;
    return this->x;
}

}