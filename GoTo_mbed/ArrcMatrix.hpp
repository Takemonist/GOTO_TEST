#ifndef ARRC_MATRIX
#define ARRC_MATRIX

#include <vector>
#include <iostream>
#include <cmath>
#include<stdarg.h>

namespace ARRC
{

template <class type> class Matrix
{
public:
    Matrix(unsigned R,unsigned C,type init_val = 0):R(R),C(C),elem(R*C,init_val) {}
    void show()
    {
        for(int i = 1; i <= R; i++) {
            for(int j = 1; j <= C; j++) {
                printf("%3.2lf ",this->at(i,j));
            }
            printf("\n");
        }
        printf("\n");
    }
    void setsize(unsigned r,unsigned c)
    {
        R = r;
        C = c;
        elem.resize(R*C);
    }
    unsigned getrows()
    {
        return R;
    }
    unsigned getcols()
    {
        return C;
    }
    unsigned getelems()
    {
        return R*C;
    }
    std::vector<type>& getelem()
    {
        return elem;
    }
    type& at(unsigned r,unsigned c)
    {
        return elem[C*(r-1)+c-1];
    }
    void fill(type num){
        for(int i = 0; i < R*C; i++) elem[i] = num;
    }
    void set(type num, ...){
        int n = 0;
        va_list elem;
        va_start(elem, num);
        this->elem[n] = num;
        for(n = 1;n < R*C;n++) this->elem[n] = va_arg(elem, type);
        va_end(elem);
    }
    
    void set(type num,va_list elem){
        int n = 0;
        this->elem[n] = num;
        for(n = 1;n < R*C;n++) this->elem[n] = va_arg(elem, type);
        va_end(elem);
    }
    
    Matrix<type>& operator = (Matrix<type> mat)
    {
        setsize(mat.getrows(),mat.getcols());
        for(int i = 0; i < R*C; i++) this->elem[i] = mat.elem[i];
        return *this;
    }
    Matrix<type> operator + (Matrix<type> mat)
    {
        if(!isSame(mat)) std::cout << "is not match rows or cols" << std::endl;
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] + mat.elem[i];
        return ret;
    }
    Matrix<type> operator - (Matrix<type> mat)
    {
        if(!isSame(mat)) std::cout << "rows or cols does not match" << std::endl;
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] - mat.elem[i];
        return ret;
    }

    Matrix<type> operator * (Matrix<type> mat)
    {
        if(C != mat.getrows()) std::cout << "rows or cols does not match" << std::endl;
        Matrix<type> ret(R,mat.getcols());
        for(int i = 1; i <= R; i++) {
            for(int j = 1; j <= mat.getcols(); j++) {
                type temp = 0;
                for(int k = 1; k <= C; k++) {
                    temp += this->at(i,k) * mat.at(k,j);
                }
                ret.at(i,j) = temp;
            }
        }
        return ret;
    }

    Matrix<type> operator + (type scalar)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] + scalar;
        return ret;
    }
    Matrix<type> operator - (type scalar)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] - scalar;
        return ret;
    }
    Matrix<type> operator * (type scalar)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] * scalar;
        return ret;
    }
    Matrix<type> operator / (type scalar)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] / scalar;
        return ret;
    }
    Matrix<type> operator % (type scalar)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = std::remainder(this->elem[i],scalar);
        return ret;
    }
    Matrix<type>& operator += (type scalar)
    {
        for(int i = 0; i < R*C; i++) elem[i] = this->elem[i] + scalar;
        return *this;
    }
    Matrix<type>& operator -= (type scalar)
    {
        for(int i = 0; i < R*C; i++) elem[i] = this->elem[i] - scalar;
        return *this;
    }
    Matrix<type>& operator *= (type scalar)
    {
        for(int i = 0; i < R*C; i++) elem[i] = this->elem[i] * scalar;
        return *this;
    }
    Matrix<type>& operator /= (type scalar)
    {
        for(int i = 0; i < R*C; i++) elem[i] = this->elem[i] / scalar;
        return *this;
    }
    Matrix<type>& operator %= (type scalar)
    {
        for(int i = 0; i < R*C; i++) elem[i] = std::remainder(this->elem[i],scalar);
        return *this;
    }
    Matrix<type> operator * (std::vector<type> elem)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] * elem[i];
        return ret;
    }
    Matrix<type> operator / (std::vector<type> elem)
    {
        Matrix<type> ret(R,C);
        for(int i = 0; i < R*C; i++) ret.elem[i] = this->elem[i] / elem[i];
        return ret;
    }
    Matrix<type> T()
    {
        Matrix<type> ret(C,R);
        for(int i = 1; i <= C; i++) {
            for(int j = 1; j <= R; j++) {
                ret.at(i,j) = this->at(j,i);
            }
        }
        return ret;
    }
    bool isSame(Matrix<type> mat)
    {
        return (R == mat.getrows() and C == mat.getcols());
    }


private:
    std::vector<type> elem;
    unsigned R,C;
};



}

#endif