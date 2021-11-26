#include "encoder.hpp"


namespace ARRC{
    
std::vector<Encoder*> Encoder::encs;
unsigned Encoder::count;

Encoder::Encoder(PinName pinA,PinName pinB,int resolution,int mode,int ID):pin_A(pinA),pin_B(pinB),resolution(resolution),mode(mode),ID(ID){
    encs.push_back(this);
    count++;
    pulse = 0;
    theta[0] = 0.0;
    theta[1] = 0.0;
    omega[0] = 0.0;
    omega[1] = 0.0;
    alpha[0] = 0.0;
    alpha[1] = 0.0;
    jerk = 0.0;
    init();
}
Encoder::~Encoder(){
    pin_B.disable_irq();
    pin_B.disable_irq();
}

void Encoder::update(double dt){
    theta[1] = 2.0*M_PI * pulse / resolution / mode;
    omega[1] = (theta[1]-theta[0])/dt;
    alpha[1] = (omega[1]-omega[0])/dt;
    jerk     = (alpha[1]-alpha[0])/dt;
    theta[0] = theta[1];
    omega[0] = omega[1];
    alpha[0] = alpha[1];
}

double Encoder::getTheta() {return theta[1];}
double Encoder::getOmega() {return omega[1];}
double Encoder::getAlpha() {return alpha[1];}
double Encoder::getJerk()  {return jerk;}
double Encoder::getID()    {return ID;}

void Encoder::init(){
    pulse = 0;
    pin_A.mode(PullUp);
    pin_B.mode(PullUp);
    if(mode <= 0 or 4 < mode) mode = 4;
    if(1 <= mode) pin_A.rise(callback(this,&Encoder::riseA));
    if(2 <= mode) pin_A.fall(callback(this,&Encoder::fallA));
    if(3 <= mode) pin_B.rise(callback(this,&Encoder::riseB));
    if(4 <= mode) pin_B.fall(callback(this,&Encoder::fallB));
}

void Encoder::riseA(void){pin_B.read() ? pulse-- : pulse++;}
void Encoder::riseB(void){pin_A.read() ? pulse++ : pulse--;}
void Encoder::fallA(void){pin_B.read() ? pulse++ : pulse--;}
void Encoder::fallB(void){pin_A.read() ? pulse-- : pulse++;}

}