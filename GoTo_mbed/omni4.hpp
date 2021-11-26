#include "robot.hpp"
#include "ArrcMatrix.hpp"
#include "ArrcMatrix_ext.hpp"

namespace ARRC
{

class OMNI4 : public Robot
{
public:
    OMNI4(double Wr1,double Wr2,double L1,double L2,double dt);
    virtual Matrix<double> getWheelVelocity(double Vx,double Vy,double Vr);
    virtual Matrix<double> getWheelVelocity(Matrix<double> Vw);
    virtual Matrix<double> getVelocity(Matrix<double> W);
private:
    double Wr1,Wr2;
    double L1,L2;
    double dt;
};

OMNI4::OMNI4(double Wr1,double Wr2,double L1,double L2,double dt):Wr1(Wr1),Wr2(Wr2),L1(L1),L2(L2),dt(dt){}

Matrix<double> OMNI4::getWheelVelocity(double Vx,double Vy,double Vr){
        Matrix<double> Vw(3,1);
        Vw.set(Vx,Vy,Vr);
        Matrix<double> A(4,3);
        A.set(0.0,1.0,L1, -1.0,0.0,L1, 0.0,-1.0,L1, 1.0,0.0,L1);
        return A*R(-r)*Vw/Wr1;
    }
Matrix<double> OMNI4::getWheelVelocity(Matrix<double> Vw){
        Matrix<double> A(4,3);
        A.set(0.0,1.0,L1, -1.0,0.0,L1, 0.0,-1.0,L1, 1.0,0.0,L1);
        return A*R(-r)*Vw/Wr1;
    }
Matrix<double> OMNI4::getVelocity(Matrix<double> W){
        Matrix<double> A(3,4);
        A.set(0.0,-1.0/2.0,0.0,1.0/2.0, 1.0/2.0,0,-1.0/2.0,0.0, 1.0/(4.0*L2),1.0/(4.0*L2),1.0/(4.0*L2),1.0/(4.0*L2));
        vel = R(r)*A*W*Wr2;
        pos = pos + vel*dt;
        return vel;
    }

}