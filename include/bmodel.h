#ifndef BMODEL_H
#define BMODEL_H

#include "math/mat.h"
#include "math/linalg.h"
#include "math/pz9011/meta.h"
#include <math.h>

class BModel
{
public:
    BModel(){}
    virtual ~BModel(){}

    static Math::mat centrifugal(const Math::mat &x)
    {
        Math::mat a{3,1};

        a[0] = -Math::PZ9011::w*Math::PZ9011::w*x[0];
        a[1] = -Math::PZ9011::w*Math::PZ9011::w*x[1];
        a[2] = 0.;

        return a;
    }

    static Math::mat coriolis(const Math::mat &x, const Math::mat &sph)
    {
        Math::mat a{3,1};
        Math::LD ld = BModel::latDot(sph[0],sph[1],x[3],x[5]);

        Math::LD wx = ld*sinl(sph[2]);
        Math::LD wy = ld*cosl(sph[2]);
        Math::LD wz = 2.*Math::PZ9011::w;

        Math::LD vx = x[3], vy = x[4], vz = x[5];

        a[0] = wy*vz - wz*vy;
        a[1] = wz*vx - wx*vz;
        a[2] = wx*vy - wy*vx;

        return a;
    }

    static Math::mat gravity(const Math::mat &x, const Math::mat &sph)
    {
        Math::mat a{3,1};
        Math::LD r3 = sph[0]*sph[0]*sph[0];

        a[0] = -Math::PZ9011::fM*x[0]/r3;
        a[1] = -Math::PZ9011::fM*x[1]/r3;
        a[2] = -Math::PZ9011::fM*x[2]/r3;

        return a;
    }

    static Math::LD latDot(Math::LD r, Math::LD lat, Math::LD Vx, Math::LD Vz)
    {
        return (-sinl(lat)*Vx + cosl(lat)*Vz)/r;
    }

    static Math::mat thrust(const Math::mat &x, const Math::mat &sph)
    {
        Math::mat a{3,1};
        Math::LD ld = BModel::latDot(sph[0],sph[1],x[3],x[5]);

        a[0] = -ld*ld*x[0];
        a[1] = 0.0;
        a[2] = -ld*ld*x[2];

        return a + BModel::coriolis(x,sph) + BModel::centrifugal(x) - BModel::gravity(x,sph);
    }
};

#endif // BMODEL_H
