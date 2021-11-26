#include "controller.hpp"

namespace ARRC{

std::vector<Controller*> Controller::ctrls;
unsigned Controller::count;
    
Controller::Controller(){
    ctrls.push_back(this);
    count++;
}

}