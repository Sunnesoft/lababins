#ifndef MATH_LINALG_H
#define MATH_LINALG_H

#include <math.h>
#include <stdexcept>

#include "math/config.h"
#include "math/mat.h"

namespace Math
{
    class LinAlg;
}

class Math::LinAlg
{
typedef Math::LD T;
public:
	LinAlg(){}
	virtual ~LinAlg(){}

    /*РД 50-25645.325-89*/
    static void precession(T t, mat3 &m)
    {
        t /= 36525.;

        T   t2  = t*t,
            t3  = t2*t;

        T   e = 0.0111808609*t + 0.146356e-5*t2 + 0.872e-7*t3,
            z = 0.0111808609*t + 0.53072e-5*t2 + 0.883e-7*t3,
            o = 0.97171735e-2*t - 0.20685e-5*t2 - 0.2028e-6*t3,
            se = sinl(e),
            ce = cosl(e),
            sz = sinl(z),
            cz = cosl(z),
            so = sinl(o),
            co = cosl(o);

        m._00 = ce*cz*co - se*sz;
        m._01 = -se*cz*co - ce*sz;
        m._02 = -cz*so;
        m._10 = ce*sz*co + se*cz;
        m._11 = -se*sz*co + ce*cz;
        m._12 = -sz*so;
        m._20 = ce*so;
        m._21 = -se*so;
        m._22 = co;
    }

    /*РД 50-25645.325-89*/
    static void nutation(T t, mat3 &m)
    {
        t /= 36525.;
        
        T   t2  = t*t,
            t3  = t2*t;

        T   o  = 2.182438624 - 33.757045936*t + 3.61429e-5*t2 + 3.88e-8*t3,
            d  = 5.19846951 + 7771.37714617*t - 3.34085e-5*t2 + 9.21e-8*t3,
            f  = 1.62790193 + 8433.46615831*t - 6.42717e-5*t2 + 5.33e-8*t3,
            l  = 6.24003594 + 628.30195602*t - 2.7974e-6*t2 - 5.82e-8*t3,
            Nt = -0.83386e-4*sinl(o) + 0.9997e-6*sinl(2.*o) - 0.63932e-5*sinl(2.*(f-d+o)) + 0.6913e-6*sinl(l)-0.11024e-5*sinl(2*(f+o)),
            Ne = 0.44615e-4*cosl(o) + 0.27809e-5*cosl(2.*(f-d+o)) + 0.474e-6*cosl(2*(f+o)),
            e0 = 0.4090928042 - 0.2269655e-3*t - 0.29e-8*t2+0.88e-8*t3,
            e  = e0+Ne,
            ce = cosl(e),
            se = sinl(e);

        m._00 = 1;
        m._01 = -Nt*ce;
        m._02 = -Nt*se;
        m._10 = Nt*ce;
        m._11 = 1;
        m._12 = -Ne;
        m._20 = Nt*se;
        m._21 = Ne;
        m._22 = 1;
    }	

    static void rotZ(T S, mat3 &m)
    {
        T  cs = cosl(S),
           ss = sinl(S);

        m._00 = cs;
        m._01 = ss;
        m._02 = 0;
        m._10 = -ss;
        m._11 = cs;
        m._12 = 0;
        m._20 = 0;
        m._21 = 0;
        m._22 = 1;    	
    }

    static void tr(mat3 &from, mat3 &to)
    {
        T temp = from._01;
        to._01 = from._10;
        to._10 = temp;

        temp = from._02;
        to._02 = from._20;
        to._20 = temp;

        temp = from._12;
        to._12 = from._21;
        to._21 = temp;

        to._00 = from._00;
        to._11 = from._11;
        to._22 = from._22;
    }

    static void mult(mat3 &a, mat3 &b, mat3 &c)
    {
 		T 	a00 = a._00,
 			a01 = a._01,
 			a02 = a._02,
 			a10 = a._10,
 			a11 = a._11,	
 			a12 = a._12,
 			a20 = a._20,
 			a21 = a._21,
 			a22 = a._22;

 		T   b00 = b._00,
 			b01 = b._01,
 			b02 = b._02,
 			b10 = b._10,
 			b11 = b._11,
 			b12 = b._12,
 			b20 = b._20,
 			b21 = b._21,
 			b22 = b._22;

 		c._00 = a00*b00 + a01*b10 + a02*b20;
 		c._01 = a00*b01 + a01*b11 + a02*b21;
 		c._02 = a00*b02 + a01*b12 + a02*b22;
 		c._10 = a10*b00 + a11*b10 + a12*b20;
 		c._11 = a10*b01 + a11*b11 + a12*b21;
 		c._12 = a10*b02 + a11*b12 + a12*b22;
 		c._20 = a20*b00 + a21*b10 + a22*b20;
 		c._21 = a20*b01 + a21*b11 + a22*b21;
 		c._22 = a20*b02 + a21*b12 + a22*b22; 		
    }

    static void mult(const mat3 &a, vec3 &b, vec3 &c)
    {
    	T 	x = b.x,
    		y = b.y,
    		z = b.z;

    	c.x = a._00*x + a._01*y + a._02*z;
    	c.y = a._10*x + a._11*y + a._12*z;
    	c.z = a._20*x + a._21*y + a._22*z;
    }

    static void mult(vec3 &a, const mat3 &b, vec3 &c)
    {
    	T 	x = a.x,
    		y = a.y,
    		z = a.z;

    	c.x = x*b._00+y*b._10+z*b._20;
    	c.y = x*b._01+y*b._11+z*b._21;
    	c.z = x*b._02+y*b._12+z*b._22;
    }

    /*РД 50-25645.325-89 without pole*/
    static void grToAbs(vec3 &gr, vec3 &inr,T time, T starTime)
    {
    	mat3 p,n,r;

    	LinAlg::nutation(time,n);
    	LinAlg::precession(time,p);
    	LinAlg::rotZ(starTime,r);

    	LinAlg::tr(n,n);
    	LinAlg::tr(p,p);
    	LinAlg::tr(r,r);

    	LinAlg::mult(p,n,p);
    	LinAlg::mult(p,r,p);
    	LinAlg::mult(p,gr,inr);
    }

    /*РД 50-25645.325-89 without pole*/
    static void absToGr(vec3 &inr, vec3 &gr, T time, T starTime)
    {
    	mat3 p,n,r;

    	LinAlg::nutation(time,n);
    	LinAlg::precession(time,p);
    	LinAlg::rotZ(starTime,r);

    	LinAlg::mult(r,n,r);
    	LinAlg::mult(r,p,r);
    	LinAlg::mult(r,inr,gr);    	
    }

    /*ICD GLONASS 5.1*/
    static void absToGr(const Math::mat &inr, Math::mat &gr, LD time, LD starTime)
    {
        if(gr.size()<6 || inr.size()<6){ throw std::out_of_range("from{to} size < 6"); }

        mat3 p,n,r;

        LinAlg::nutation(time,n);
        LinAlg::precession(time,p);
        LinAlg::rotZ(starTime,r);

        LinAlg::mult(r,n,r);
        LinAlg::mult(r,p,r);

        vec3 x,v;
        x.x = inr[0];
        x.y = inr[1];
        x.z = inr[2];

        v.x = inr[3];
        v.y = inr[4];
        v.z = inr[5];

        LinAlg::mult(r,x,x); 
        LinAlg::mult(r,v,v);

        gr[0] = x.x;
        gr[1] = x.y;
        gr[2] = x.z;
        gr[3] = v.x + 7.292115e-5*x.y;
        gr[4] = v.y - 7.292115e-5*x.x;
        gr[5] = v.z;        
    }

    /*ICD GLONASS 5.1*/
    static void grToAbs(const Math::mat &gr, Math::mat &inr, LD time, LD starTime)
    {
        if(gr.size()<6 || inr.size()<6){ throw std::out_of_range("from{to} size < 6"); }

        mat3 p,n,r;

        LinAlg::nutation(time,n);
        LinAlg::precession(time,p);
        LinAlg::rotZ(starTime,r);

        LinAlg::tr(n,n);
        LinAlg::tr(p,p);
        LinAlg::tr(r,r);

        LinAlg::mult(p,n,p);
        LinAlg::mult(p,r,p);

        vec3 x,v;
        x.x = gr[0];
        x.y = gr[1];
        x.z = gr[2];

        v.x = gr[3];
        v.y = gr[4];
        v.z = gr[5];

        LinAlg::mult(p,x,x);
        LinAlg::mult(p,v,v);

        inr[0] = x.x;
        inr[1] = x.y;
        inr[2] = x.z;
        inr[3] = v.x - x.y*7.292115e-5;
        inr[4] = v.y + x.x*7.292115e-5;
        inr[5] = v.z;
    }

    static void sphToCart(const Math::mat &from, Math::mat &to)
    {
        if(from.size()<3 || to.size()<3){ throw std::out_of_range("from{to} size < 3"); }

        LD r = from[0], p = from[1], l = from[2];
        LD cy = cosl(p);
        to[0] = r*cy*cosl(l);
        to[1] = r*cy*sinl(l);
        to[2] = r*sinl(p);
    }

    static void cartToSph(const Math::mat &from, Math::mat &to)
    {
        if(from.size()<3 || to.size()<3){ throw std::out_of_range("from{to} size < 3"); }

        LD x = from[0], y = from[1], z = from[2];
        LD r = x*x+y*y;
        to[0] = sqrtl(r+z*z);
        to[1] = atanl(z/sqrtl(r));
        to[2] = atan2l(y,x);
    }

    /*Martin Losch - December 5, 2003*/
    static void geogrToCart(const Math::mat &from, Math::mat &to, LD a, LD e)
    {
        if(from.size()<3 || to.size()<3){ throw std::out_of_range("from{to} size < 3"); }

        LD  p = from[1], 
            l = from[2],
            h = from[0];
        LD  cp  = cosl(p),
            sp  = sinl(p),
            r   = a/sqrtl(1 - e*e*sp*sp) + h;

        to[0] = r*cp*cosl(l);
        to[1] = r*cp*sinl(l);
        to[2] = r*sp*(1.-e*e);        
    }

    /*PZ9011 - from geodetics CS a to CS b*/
    static void aToB(const Math::mat &from, Math::mat &to, const Math::mat &g)
    {
        if(from.size()<3 || to.size()<3){ throw std::out_of_range("from{to} size < 3"); }
        if(g.size()<7){ throw std::out_of_range("transph. coeffic. mat size < 7"); }

        LD  x = from[0], y = from[1], z = from[2],
            m = 1.+g[6];

        to[0] = m*(x+y*g[2]-z*g[1]) + g[3];
        to[1] = m*(-x*g[2]+y+z*g[0]) + g[4];
        to[2] = m*(x*g[1]-y*g[0]+z) + g[5];
    }

    /*a - radius-vector point on Earth, b - observable point*/
    static LD elevation(const Math::mat &a, const Math::mat &b)
    {
        if(a.size()<3 || b.size()<3){ throw std::out_of_range("from{to} size < 3"); }

        LD  dx      = a[0] - b[0],
            dy      = a[1] - b[1],
            dz      = a[2] - b[2],
            R_2     = b[0]*b[0]+b[1]*b[1]+b[2]*b[2],
            D       = sqrtl(dx*dx+dy*dy+dz*dz),
            h       = sqrtl(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        
        return asinl((R_2-h*h-D*D)/(2.*h*D));       
    }
};

#endif //MATH_LINALG_H
