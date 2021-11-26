#ifndef MOVEMENT_ENCODER
#define MOVEMENT_ENCODER

#include "mbed.h"
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

namespace ARRC
{

class Encoder
{
public:
    Encoder(PinName pinA,PinName pinB,int resolution,int mode,int id);
    ~Encoder();
    void update(double dt);
    double getTheta();
    double getOmega();
    double getAlpha();
    double getJerk();
    double getID();
    long long int pulse;
    static std::vector<Encoder*> encs;
    static unsigned count;
private:
    InterruptIn pin_A;
    InterruptIn pin_B;
    double theta[2];
    double omega[2];
    double alpha[2];
    double jerk;

    int resolution;
    int mode;

    void init();
    void riseA(void);
    void riseB(void);
    void fallA(void);
    void fallB(void);
    int ID;
};

}
#endif
