#ifndef CONTROLLER_PID
#define CONTROLLER_PID

#include "controller.hpp"

namespace ARRC
{

class PID : public Controller
{
public:
    PID(double Kp,double Ki,double Kd,double max,double min,int ID,bool addmv = true);
    virtual void update(double val,double tgt,double dt);
    virtual double getmv();
    unsigned getID();
private:
    bool addmv;
    double e[2];
    double integral;
    double P,I,D;
    double Kp;
    double Ki;
    double Kd;
    double mv,dmv;

    double Max;
    double Min;
    unsigned ID;
};

}
#endif