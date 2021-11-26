#ifndef CORE_MOVE
#define CORE_MOVE

#include "ArrcMatrix.hpp"
#include "controller.hpp"
#include "robot.hpp"
#include "motor.hpp"
#include "encoder.hpp"

namespace ARRC
{

class Movement
{
public:
    Movement(Robot* bot,double dt);
    void setVelocity(double Vx,double Vy,double Vr);
    void setVelocity(Matrix<double> Vw);
    Matrix<double> getStatus();
    void setPWM(double pwm,int id);
    void START();
    bool LOOP();
    void WAIT(double wt);
    double getms();
private:
    Timer timer;
    Robot* robot;
    double dt;
    long long t;
};

}

#endif