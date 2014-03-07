#ifndef BCONV_H
#define BCONV_H

#include "math/mat.h"
#include <math.h>

class BConv
{
public:
    BConv(){}
    virtual ~BConv(){}

    /*
     * lat - latitude
     * lon - longitude
     */

    static Math::mat greenwichToNav(Math::LD lat, Math::LD lon)
    {
        Math::LD slt = sinl(lat),
                 clt = cosl(lat),
                 sln = sinl(lon),
                 cln = cosl(lon);

        Math::mat c{3,3};
        c.to(0,0,-sln);
        c.to(0,1,cln);
        c.to(1,0,-slt*cln);
        c.to(1,1,-slt*sln);
        c.to(1,2,clt);
        c.to(2,0,cln*clt);
        c.to(2,1,sln*clt);
        c.to(2,2,slt);

        return c;
    }

    /*
     * p - pitch
     * r - roll
     * y - yaw
     */

    static Math::mat navToAircraft(Math::LD p, Math::LD r, Math::LD y)
    {
        Math::LD sp = sinl(p),
                 cp = cosl(p),
                 sr = sinl(r),
                 cr = cosl(r),
                 sy = sinl(y),
                 cy = cosl(y);

        Math::mat c{3,3};
        c.to(0,0,cr*cy+sp*sr*sy);
        c.to(0,1,-sy*cr + sp*sr*cy);
        c.to(0,2,sr*cp);
        c.to(1,0,cp*sy);
        c.to(1,1,cp*cy);
        c.to(1,2,-sp);
        c.to(2,0,-sr*sp + sp*cr*sy);
        c.to(2,1,sr*sy+sp*cr*cy);
        c.to(2,2,cr*cp);

        return c;
    }
};

#endif // BCONV_H
