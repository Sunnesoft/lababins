#ifndef BBINS_H
#define BBINS_H

#include "math/mat.h"
#include "include/bconv.h"
#include "include/bmodel.h"
#include "include/bpuasson.h"

/*
 * Block with 3 accelerometers
 *
 */
class BAccBlock
{
public:
    BAccBlock(){}
    virtual ~BAccBlock(){}

    Math::mat measure(const Math::mat &x,
                 const Math::mat &sph,
                 const Math::mat &a)
    {
        auto acc = BConv::greenwichToNav(sph[1],sph[2])
                 * BConv::navToAircraft(a[0],a[1],a[2])
                 * BModel::thrust(x,sph);

        return acc;
    }
};

/*
 * Block with 3 Angular velocity sensors
 */
class BAvsBlock
{
public:
    BAvsBlock(){}
    virtual ~BAvsBlock(){}

    Math::mat measure(const Math::mat &x,
                 const Math::mat &sph,
                 const Math::mat &a)
    {
        auto ld = BModel::latDot(sph[0],sph[1],x[3],x[5]);

        Math::mat avs{3,1};
        avs[0] = -ld;
        avs[1] = Math::PZ9011::w*cosl(sph[1]);
        avs[2] = Math::PZ9011::w*sinl(sph[1]);

        avs = BConv::greenwichToNav(sph[1],sph[2])
            * BConv::navToAircraft(a[0],a[1],a[2])
            * avs;

        return avs;
    }
};

class BBins
{
public:
    BBins(){}
    virtual ~BBins(){}

    void init(Math::LD startTime,
              const Math::mat &x,
              const Math::mat &angles)
    {
        _x = x;
        _orientation.setTime(startTime);

        Math::mat sph{3,1,Math::mat::Undefined};
        Math::LinAlg::cartToSph(_x,sph);

        auto C = (BConv::greenwichToNav(sph[1],sph[2])
                  *BConv::navToAircraft(angles[0],angles[1],angles[2])).tr();

        _orientation.setC(C);
    }

    Math::mat x(){ return _x; }
    Math::mat orientation(){ return _orientation.C(); }

    void next(Math::LD step,
              const Math::mat& acc,
              const Math::mat& w)
    {
        Math::mat wie{3,1};
        wie[2] = Math::PZ9011::w;

        _orientation.next(step,w,wie);

        auto bodyToGreenwich = _orientation.C();

        Math::mat sph{3,1,Math::mat::Undefined};
        Math::mat ax{6,1,Math::mat::Undefined};

        Math::LinAlg::cartToSph(_x,sph);

        auto a = BModel::gravity(_x,sph)
          - BModel::centrifugal(_x)
          - BModel::coriolis(_x,sph)
          + bodyToGreenwich*acc;

        ax[0] = _x[3];
        ax[1] = _x[4];
        ax[2] = _x[5];
        ax[3] = a[0];
        ax[4] = a[1];
        ax[5] = a[2];

        _x = _x + step*ax;
    }

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

    BPuasson    _orientation;
};

#endif // BBINS_H
