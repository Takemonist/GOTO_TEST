#include "robot.hpp"
#include "ArrcMatrix.hpp"
#include "ArrcMatrix_ext.hpp"

namespace ARRC
{

class Unicycle : public Robot
{
public:
    Unicycle(double Wr,double dt);
    virtual Matrix<double> getWheelVelocity(double Vx,double Vy,double Vr);
    virtual Matrix<double> getWheelVelocity(Matrix<double> Vw);
    virtual Matrix<double> getVelocity(Matrix<double> W);
private:
    double Wr;
    double dt;
};

Unicycle::Unicycle(double Wr,double dt):Wr(Wr),dt(dt){}

Matrix<double> Unicycle::getWheelVelocity(double Vx,double Vy,double Vr){
        Matrix<double> Vw(3,1);
        Vw.set(Vx,Vy,Vr);
        Matrix<double> A(1,3);
        A.set(0.0,1.0,0.0);
        return A*Vw/Wr;
    }
Matrix<double> Unicycle::getWheelVelocity(Matrix<double> Vw){
        Matrix<double> A(1,3);
        A.set(0.0,1.0,0.0);
        return A*Vw/Wr;
    }
Matrix<double> Unicycle::getVelocity(Matrix<double> W){
        Matrix<double> A(3,1);
        A.set(0.0, 1.0, 0.0);
        vel = A*W*Wr;
        pos = pos + vel*dt;
        return vel;
    }

}