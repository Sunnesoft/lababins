#ifndef BMOTION_H
#define BMOTION_H

#include "math/mat.h"
#include "include/bmodel.h"
#include "include/bconv.h"

class BMotion
{
private:
    /*Indeces:
     *0 - x
     *1 - y
     *2 - z
     *3 - vx
     *4 - vy
     *5 - vz
     */
    Math::mat   _x{6,1};

    /*Indeces:
     *0 - pitch
     *1 - roll
     *2 - yaw
     */
    Math::mat   _orientation{3,1};

    Math::LD    _time;

public:
    BMotion(){}
    virtual ~BMotion(){}

    void setX(const Math::mat &x){_x = x;}
    void setTime(Math::LD t){_time = t;}
    void setOrientation(const Math::mat &a){_orientation = a;}

    Math::mat x(){ return _x; }
    Math::mat orientation(){ return _orientation; }
    Math::LD time(){ return _time; }

    void next(Math::LD step)
    {
        Math::mat sph{3,1,Math::mat::Undefined};
        Math::mat ax{6,1,Math::mat::Undefined};

        Math::LinAlg::cartToSph(_x,sph);

        auto a = BModel::gravity(_x,sph)
          - BModel::centrifugal(_x)
          - BModel::coriolis(_x,sph)
          + BModel::thrust(_x,sph);

        ax[0] = _x[3];
        ax[1] = _x[4];
        ax[2] = _x[5];
        ax[3] = a[0];
        ax[4] = a[1];
        ax[5] = a[2];

        _x = _x + step*ax;
        _time += step;
    }
};

#endif // BMOTION_H
