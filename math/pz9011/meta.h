#ifndef MATH_PZ9011_META_H
#define MATH_PZ9011_META_H

#include "math/config.h"

namespace Math
{
    namespace PZ9011
    {
        /*
         *position: km
         *velocity: km/sec
         */
         
        static constexpr LD fM  = 398600.4418;
        static constexpr LD w   = 7.292115e-5;
        static constexpr LD a   = 6378.136;
        static constexpr LD c   = 299792.458;
        
        static constexpr LD J20 = 1082.62575e-6;
        static constexpr LD J40 = -2.37089e-6;
        static constexpr LD J60 = 6.08e-9;
        static constexpr LD J80 = -1.40e-11;

        static constexpr LD C20 = -0.44721359549995793928183473374626*J20;
        static constexpr LD C40 = -0.33333333333333333333333333333333*J40;
        static constexpr LD C60 = -0.2773500981126145610091708667285*J60;
        static constexpr LD C80 = -0.24253562503633297351890646211612*J80;
    }
}

#endif // PZ9011_META_H
