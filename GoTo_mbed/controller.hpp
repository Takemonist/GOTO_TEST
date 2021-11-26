#ifndef CONTROLLER
#define CONTROLLER

#include <vector>

namespace ARRC
{

class Controller
{
public:
    Controller();
    virtual void update(double val,double tgt,double dt) = 0;
    virtual double getmv() = 0;
    static std::vector<Controller*> ctrls;
    static unsigned count;
private:
};

}

#endif