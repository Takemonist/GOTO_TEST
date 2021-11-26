#include "movement.hpp"

namespace ARRC
{

Movement::Movement(Robot* robot,double dt):dt(dt),t(0){
    this->robot = robot;    
}
void Movement::setVelocity(double Vx,double Vy,double Vr)
{
    Matrix<double> W = robot->getWheelVelocity(Vx,Vy,Vr);
    
    for(int i = 0; i < Encoder::count; i++) Controller::ctrls.at(i)->update(Encoder::encs.at(i)->getOmega(),W.at(i+1,1),dt);
    for(int i = 0; i < Motor::count; i++) Motor::mtrs.at(i)->setPWM(Controller::ctrls.at(i)->getmv());
}
void Movement::setVelocity(Matrix<double> Vw)
{
    Matrix<double> W = robot->getWheelVelocity(Vw);
    for(int i = 0; i < Encoder::count; i++) Controller::ctrls.at(i)->update(Encoder::encs.at(i)->getOmega(),W.at(i+1,1),dt);
    for(int i = 0; i < Motor::count; i++) Motor::mtrs.at(i)->setPWM(Controller::ctrls.at(i)->getmv());
}
Matrix<double> Movement::getStatus()
{
    Matrix<double> W(Encoder::count,1,0.0);
    for(int i = 0; i < Encoder::count; i++) {
        Encoder::encs.at(i)->update(dt);
        W.at(i+1,1) = Encoder::encs.at(i)->getOmega();
    }
    robot->getVelocity(W);
    return robot->getPosition();
}
void Movement::setPWM(double pwm,int id) {Motor::mtrs.at(id)->setPWM(pwm);}
void Movement::START(){timer.start();}
bool Movement::LOOP(){
    bool ret = true;
    long long temp = 0;
    for(temp = 0;temp <= dt*1000.0*1000.0;temp = timer.read_us() - t){
        if(temp < 0.0){
            ret = false;
            printf("WARN!:Out of cycle:%lld(us)\n",temp);
        }
    }
    t = timer.read_us();
    return ret;
}
void Movement::WAIT(double wt){while(timer.read_us()-t <= wt*1000.0*1000.0);}

double Movement::getms(){return t/1000.0;}
}