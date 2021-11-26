#include "mbed.h"
#include "movement.hpp"
#include "omni4.hpp"
#include "imc.hpp"
#include "pid.hpp"
#include "Unicycle.hpp"

const double dt = 0.08;

using namespace ARRC;

//OMNI4 BOT(50.8,50.8,322.5,259.75,dt);

int main(){
Unicycle BOT(50.8,dt);
Movement core(&BOT,dt);

Encoder e1(PC_10,PC_11,512,4,1);
IMC c1(40.0,0.1,0.5,-0.5,1);
//PID p1(0.0042,0.000,0.00008,0.5,-0.5,1);
Motor m1(PB_14,PB_13,2048,0.5,-0.5,1);
Matrix<double> pos(3,1);
    core.START();
    double tgt = 0;
    double t = 0;
    while(true){
        t = core.getms()/400.0;
        tgt = 1.0*sin(t/0.8)+ 3.5*sin(1.0/2*t)*cos(t/0.8) + 5.0;
        tgt *= 50.8;
        core.getStatus();
        core.setVelocity(0.0,tgt,0.0);
        //printf("v:%5lf tgt:%5lf\n",BOT.vel.at(2,1),100.0);
        printf("v:%5lf tgt:%5lf\n",BOT.vy,tgt);
        core.LOOP();
    }
}