#ifndef ARRC_MATRIX_EXTEND
#define ARRC_MATRIX_EXTEND

#include "ArrcMatrix.hpp"
namespace ARRC{

Matrix<double> E(unsigned n){
    Matrix<double> ret(n,n,0.0);
        for(int i = 1;i <= n;i++) ret.at(i,i) = 1.0;
    return ret; 
}

Matrix<double> R(double r){
    Matrix<double> ret(3.0,3.0,0.0);
    ret.set(cos(r),-sin(r),0.0,
            sin(r), cos(r),0.0,
            0.0     ,0.0     ,1.0);
    return ret; 
}

}
#endif