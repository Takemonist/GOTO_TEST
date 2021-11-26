#ifndef CORE_ROBOT
#define CORE_ROBOT

#include "ArrcMatrix.hpp"

namespace ARRC
{

class Robot
{
public:
    Matrix<double> vel,pos;
    double &x,&y,&r;
    double &vx,&vy,&w;
    Robot():vel(3,1),pos(3,1),x(pos.at(1,1)),y(pos.at(2,1)),r(pos.at(3,1)),vx(vel.at(1,1)),vy(vel.at(2,1)),w(vel.at(3,1)){}
    virtual Matrix<double> getWheelVelocity(double Vx,double Vy,double Vr) = 0;
    virtual Matrix<double> getWheelVelocity(Matrix<double> Vw) = 0;
    virtual Matrix<double> getVelocity(Matrix<double> W) = 0;
    
    Matrix<double> getVelocity(){return vel;}
    void setVelocity(Matrix<double> vel)
    {
        this->vel = vel;
    }
    void setVelocity(double vel, ...)
    {
        va_list elem;
        va_start(elem, vel);
        this->vel.set(vel,elem);
    }
    Matrix<double> getPosition(){return pos;}
    void setPosition(Matrix<double> pos)
    {
        this->pos = pos;
    }
    void setPosition(double pos, ...)
    {
        va_list elem;
        va_start(elem, pos);
        this->pos.set(pos,elem);
    }
private:
};

}

#endif
