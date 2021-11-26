#ifndef CONTROLLER_IMC
#define CONTROLLER_IMC

#include "controller.hpp"

namespace ARRC
{
 
class IMC : public Controller{
public:
    IMC(double K,double T,double max,double min,unsigned ID);
    virtual void update(double val,double tgt,double dt);
    virtual double getmv();
    unsigned getID();
private:
    double simulate(double x,double u,double dt);
    double x;
    double K;
    double T;
    double mv;

    double Max;
    double Min;
    unsigned ID;
};

}
#endif