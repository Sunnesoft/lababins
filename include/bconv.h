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

        Math::mat c{3,3}, cz{3,3}, cx{3,3}, cy{3,3};

        cz.to(0,0,cos(-y));
        cz.to(0,1,-sin(-y));
        cz.to(0,2,0);
        cz.to(1,0,sin(-y));
        cz.to(1,1,cos(-y));
        cz.to(1,2,0);
        cz.to(2,0,0);
        cz.to(2,1,0);
        cz.to(2,2,1);

        cx.to(0,0,1);
        cx.to(0,1,0);
        cx.to(0,2,0);
        cx.to(1,0,0);
        cx.to(1,1,cos(p));
        cx.to(1,2,-sin(p));
        cx.to(2,0,0);
        cx.to(2,1,sin(p));
        cx.to(2,2,cos(p));

        cy.to(0,0,cos(r));
        cy.to(0,1, 0);
        cy.to(0,2, sin(r));
        cy.to(1,0, 0);
        cy.to(1,1, 1);
        cy.to(1,2, 0);
        cy.to(2,0, -sin(r));
        cy.to(2,1, 0);
        cy.to(2,2, cos(r));

        c = cz*cx*cy;
        /*ToDo
         В каком порядке перемножать
         опытно проверил в матлабе, должно быть cx*cy*cz
        */

        return c;
    }
};

#endif // BCONV_H
