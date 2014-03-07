#ifndef MATH_MAT_H
#define MATH_MAT_H

#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <string>

#include "math/config.h"

/*

using namespace Math;

int main()
{  
   mat a(4,4,mat::MatType::Idty);
   mat b(4,4,mat::MatType::Null);
   
   std::cout <<"Identity:\n"<< (std::string)a << std::endl; 
   std::cout <<"Null:\n"<< (std::string)b << std::endl; 
   
   mat::copy(a,b);
   
   std::cout <<"Identity:\n"<< (std::string)b << std::endl;
   
   mat c(4,4);
   c = a+b;
   std::cout <<"Identity+Identity:\n"<< (std::string)c << std::endl;
   
   mat d = a+b;
   std::cout <<"Same:\n"<< (std::string)d << std::endl;
   std::cout <<"Pre==PrePre:\n"<< ((std::string)d == (std::string)c)<< std::endl;
   
   c = a - b;
   std::cout <<"Identity-Identity:\n"<< (std::string)c << std::endl;
   
   d.to(1,2,3.3);
   d.to(2,3,5);
   d.to(3,0,8.6);
   d.to(0,2,0.35);
   std::cout <<"Some mat:\n"<< (std::string)d << std::endl;
   
   c = d.inv();
   std::cout <<"Pre.inv():\n"<< (std::string)c << std::endl;
   
   c = d*c;
   std::cout <<"Pre*Pre.inv(){== Identity}:\n"<< (std::string)c << std::endl;
   
   std::cout << (d.at(1,2) == 3.3) << std::endl;
   std::cout << (d.at(2,3) == 5) << std::endl;
   std::cout << (d.at(0,0) == 2) << std::endl;
   
   d.at(1)      = 345;
   d.at(2,3)    = 545;
   d.at(0,0)    = 4000;
   d.at(0,2)    = 123; 
   
   std::cout << (d.at(0,1) == 345) << std::endl;
   std::cout << (d.at(2,3) == 545) << std::endl;
   std::cout << (d.at(0,0) == 4000) << std::endl;   
   std::cout << (d.at(0,2) == 123) << std::endl; 
   std::cout << (d[1] == 345) << std::endl; 
   
   d[1] = 456789;
   
   std::cout << (d[1] == 456789) << std::endl;

   try
   {
       mat(5,5,mat::MatType::Null).inv();
   }
   catch( std::exception &e)
   {
       std::cout << (e.what() == std::string("det is null")) << std::endl;
   }
   
   mat l(10,1);
   l[0] = 3.3;
   l[1] = 4;
   l[2] = 456;
   l[3] = 3.3;
   l[4] = 4;
   l[5] = 456;
   l[6] = 3.3;
   l[7] = 4;
   l[8] = 456;
   l[9] = 9000;
   
   std::cout <<"Vector:\n"<< (std::string)l << std::endl;
   
   mat k = l.tr();
   std::cout <<"Tr Vector:\n"<< (std::string)k << std::endl;
   
   try
   {
       k.inv();
   }
   catch( std::exception &e)
   {
       std::cout << (e.what() == std::string("determinant isset only at square matrix")) << std::endl;
   }    
   
   return 0;
}
*/

namespace Math
{
    class mat;
}

class Math::mat
{
protected:
    typedef Math::LD        T;
    typedef Math::ptrLD     ptr_T;
    typedef Math::pptrLD    pptr_T;
    typedef Math::cptrLD    cptr_T;
    typedef Math::I         I;
    typedef Math::CI        CI;

    ptr_T   _d;
    CI      _m;
    CI      _n;

public:
    enum MatType
    {
        Null,
        Idty,
        Undefined
    };

    mat(I m, I n, MatType t = MatType::Null) : _m(m), _n(n)
    {
        if(_m<=0 || _n<=0)
        {
            throw std::out_of_range("m{n} is incorrect");
        }
        
        I s = _m*_n;
        _d = new T[s];
        
        switch(t)
        {
        case MatType::Null: null(); break;
        case MatType::Idty: idty(); break;
        case MatType::Undefined: break;
        default: break;
        }  
    }

    virtual ~mat(){ delete[] _d; _d = 0; }

    inline I m() const noexcept { return _m; }
    inline I n() const noexcept { return _n; }
    inline I size() const noexcept { return _m*_n; }

    T at(I i) const 
    {
        if(i>=_m*_n || i<0){ throw std::out_of_range("i >= M*N || i<0"); }
        return _d[i];
    }

    T& at(I i)
    {
        if(i>=_m*_n || i<0){ throw std::out_of_range("i >= M*N || i<0"); }
        return _d[i];
    }

    T& at(I i, I j){ return at(i*_n+j); }
    T at(I i, I j) const { return at(i*_n+j); }
    T operator [](I i) const { return at(i); }
    T& operator [](I i) { return at(i); }

    mat& to(I i, T val)
    {
        if(i>=_m*_n || i<0){ throw std::out_of_range("i >= M*N || i<0"); }
        _d[i] = val;
        return *this;
    }

    mat& to(I i, I j, T val){ return to(i*_n+j,val); }   

    static void copy(const mat& a, mat& b)
    {
        if(!&a || !&b){ throw std::logic_error("parameters of copy is null_ptr"); }
        if(b.m()!=a.m()||b.n()!=a.n()){ throw std::logic_error("a and b have different size"); }

        I s = b.m()*b.n();
        for(I i = 0;i<s;i++){ b[i] = a[i]; }
    }

    void copy(ptr_T a, I size)
    {
        if(!a){ throw std::logic_error("parameters of copy is null_ptr"); }
        if(size > _m*_n){ throw std::out_of_range("size>M*N"); }

        for(I i = 0;i<size;i++){ _d[i] = a[i]; }        
    }

    void copy(pptr_T a)
    {
        if(!a){ throw std::logic_error("parameters of copy is null_ptr"); }

        for(I i = 0;i<_m;i++)
        {
            for(I j = 0;j<_n;j++)
            {
                _d[i*_n+j] = a[i][j];
            }
        }
    }

    mat(const mat& a) :_m(a._m), _n(a._n)
    {
        if(!&a){ throw std::logic_error("parameters of copy is null_ptr"); }

        if(_m<=0 || _n<=0)
        {
            throw std::out_of_range("m{n} is incorrect");
        }

        I s = _m*_n;
        _d = new T[s];

        for(I i = 0;i<s;i++){ _d[i] = a._d[i]; }        
    }

    mat& operator =(const mat& a)
    {
        if(&a == this) return *this;
        mat::copy(a,*this);
        return *this;        
    }

    mat tr() const
    {
        mat t(_n,_m, MatType::Undefined);

        for(I i = 0;i<_n;i++)
        {
            for(I j = 0;j<_m;j++)
            {
                t.to(i,j,_d[j*_n+i]);
            }
        }
        return t;
    }

    mat& idty()
    {
        for(I i = 0;i<_m;i++)
        {
            for(I j = 0;j<_n;j++)
            {
                _d[i*_n+j] = 0.0;
            }
            _d[i*_n+i] = 1.0;
        }
        return *this;
    }

    mat& null()
    {
        I s = _m*_n;
        for(I i = 0;i<s;i++) _d[i] = 0.0;
        return *this;
    }

    T det() const
    {
        if(_m != _n){ throw std::logic_error("determinant isset only at square matrix"); }

        if(_m == 1) return _d[0];
        if(_m == 2) return _d[0]*_d[3]-_d[1]*_d[2];
        if(_m == 3) return _d[0]*_d[4]*_d[8]+_d[3]*_d[7]*_d[2]+_d[1]*_d[5]*_d[6]-_d[6]*_d[4]*_d[2]-_d[3]*_d[1]*_d[8]-_d[7]*_d[5]*_d[0];

        T d = 0.;
        I r = _m-1;

        mat t(r,r);

        for(I i = 0;i<_m;i++)
        {
            for(I j = 1;j<_m;j++)
            {
                I m = j-1;
                for(I k = 0,n = 0;k<_m;k++)
                {
                    if(k == i) continue;

                    t.to(m,n,_d[j*_n+k]);
                    n++;
                }
            }

            d += (i%2==0?1:-1)*_d[i]*t.det();
        }

        return d;
    }

    mat inv() const
    {
        if(_m == _n && _m > 3)
        {
            I s = _m*_n;
            T res[s];
            T data[s];

            I rep = 1;
            T det = 1;
            T koef = 0.0;

            for(I i = 0;i<_m;i++)
            {
                for(I j = 0;j<_n;j++)
                {
                    res[i*_n+j] = 0;
                    data[i*_n+j] = _d[i*_n+j];
                }
                res[i*_n+i] = 1;
            }

            for(I i = 0;i<_m;i++)
            {
                I p = i;

                if(data[p*_n+i] == 0.)
                {
                    koef = 0.;
                    for(;p<_m;p++)
                    {
                        if(data[p*_n+i]!=0)
                        {
                            koef = data[p*_n+i];
                            break;
                        }
                    }

                    if(0. == koef){ throw std::logic_error("det is null"); }

                }

                koef = data[p*_n+i];

                if(p!=i) rep *= -1;

                for(I j = 0;j<_n;j++)
                {
                    I ij = i*_n+j;

                    if(p!=i)
                    {
                        I pj = p*_n+j;

                        T temp = data[pj];
                        data[pj] = data[ij];
                        data[ij] = temp;

                        temp = res[pj];
                        res[pj] = res[ij];
                        res[ij] = temp;
                    }

                    data[ij] /= koef;
                    res[ij] /= koef;
                }


                for(I j = i+1;j<_m;j++)
                {
                    T temp = -data[j*_n+i];
                    for(I k = 0;k<_m;k++)
                    {
                        I jk = j*_n+k, ik = i*_n+k;

                        data[jk] += data[ik]*temp;
                        res[jk] += res[ik]*temp;
                    }
                }

                det *= koef;
            }

            if(det*rep == 0.){ throw std::logic_error("det is null"); }

            for(I i = _m-1;i>=0;i--)
            {
                I p = i;
                if(data[p*_n+i] == 0.)
                {
                    koef = 0.;
                    for(;p<_m;p++)
                    {
                        if(data[p*_n+i]!=0)
                        {
                            koef = data[p*_n+i];
                            break;
                        }
                    }

                    if(0. == koef){ throw std::logic_error("det is null"); }
                }

                koef = data[p*_n+i];

                for(I j = 0;j<_m;j++)
                {
                    I ij = i*_n+j;

                    if(p!=i)
                    {
                        I pj = p*_n+j;

                        T temp = data[pj];
                        data[pj] = data[ij];
                        data[ij] = temp;

                        temp = res[pj];
                        res[pj] = res[ij];
                        res[ij] = temp;
                    }

                    data[ij] /= koef;
                    res[ij] /= koef;
                }

                for(I j = i-1;j>=0;j--)
                {
                    T temp = -data[j*_n+i];
                    for(I k = 0;k<_m;k++)
                    {
                        I jk = j*_n+k,
                             ik = i*_n+k;

                        data[jk] += data[ik]*temp;
                        res[jk] += res[ik]*temp;
                    }
                }
            }

            mat t(_m,_n,MatType::Undefined);

            for(I i = 0;i<s;i++)
            {
                t.to(i,res[i]);
            }

            return t;
        }

        T d = det();

        if(d == 0.){ throw std::logic_error("det is null"); }

        mat t(_m,_n,MatType::Undefined);

        if(_m == 1) return t.to(0,0,1./d);

        if(_m == 2)
        {
            t.to(0,_d[3]/d);    t.to(1,-_d[2]/d);
            t.to(2,-_d[1]/d);    t.to(3,_d[0]/d);
            return t;
        }

        if(_m == 3)
        {
            t.to(0,(_d[4]*_d[8]-_d[5]*_d[7])/d);
            t.to(1,-(_d[3]*_d[8]-_d[5]*_d[6])/d);
            t.to(2,(_d[3]*_d[7]-_d[6]*_d[4])/d);
            t.to(3,-(_d[1]*_d[8]-_d[2]*_d[7])/d);
            t.to(4,(_d[0]*_d[8]-_d[2]*_d[6])/d);
            t.to(5,-(_d[0]*_d[7]-_d[1]*_d[6])/d);
            t.to(6,(_d[1]*_d[5]-_d[2]*_d[4])/d);
            t.to(7,-(_d[0]*_d[5]-_d[2]*_d[3])/d);
            t.to(8,(_d[0]*_d[4]-_d[1]*_d[3])/d);
            return t;
        }

        throw std::logic_error("can't invert matrix"); 
    }

    operator std::string() const
    {
        std::ostringstream _in;
        for(I i = 0;i<_m && i<100;i++)
        {
            for(I j = 0;j<_n && j<100;j++)
            {
                _in<<std::setprecision(10)<<_d[i*_n+j]<<'\t';
            }
            _in<<std::endl;
        }   

        return _in.str();    
    }

    friend mat operator +(const mat& a, const mat& b);
    friend mat operator -(const mat& a, const mat& b);
    friend mat operator *(const mat& a, const mat& b);
    friend mat operator *(const mat& a, T b);
    friend mat operator *(T b, const mat& a);

    ptr_T ptr() noexcept { return _d; }

private:
    mat();    
};

namespace Math
{
    inline mat operator +(const mat& a, const mat& b)
    {
        mat::I n = a.n(), m = a.m();
        if(n != b.n() || m != b.m()) throw std::logic_error("+:a and b must have same size.");

        mat::I s = n*m;
        mat t(m,n,mat::MatType::Undefined);

        for(mat::I i = 0;i<s;i++) t.to(i,a[i]+b[i]);
        return t;
    }

    inline mat operator -(const mat& a, const mat& b)
    {
        mat::I n = a.n(), m = a.m();
        if(n != b.n() || m != b.m()) throw std::logic_error("-:a and b must have same size.");

        mat::I s = n*m;
        mat t(m,n,mat::MatType::Undefined);

        for(mat::I i = 0;i<s;i++) t.to(i,a[i]-b[i]);
        return t;
    }

    inline mat operator *(const mat& a, const mat& b)
    {
        mat::I m = a.m(), n = a.n(), r = b.n();
        if(n!= b.m()) throw std::logic_error("*:a.n() != b.m()");

        mat t(m,r,mat::MatType::Undefined);

        for(mat::I i = 0;i<m;i++)
        {
            for(mat::I j = 0;j<r;j++)
            {
                mat::T tp = 0;
                for(mat::I k = 0;k<n;k++)
                {
                    tp += a[i*n+k]*b[k*r+j];
                }
                t.to(i,j,tp);
            }
        }

        return t;
    }

    inline mat operator *(const mat& a, mat::T b)
    {
        mat::I m = a.m(), n = a.n();

        mat t(m,n,mat::MatType::Undefined);

        for(mat::I i = 0;i<m;i++)
        {
            for(mat::I j = 0;j<n;j++)
            {
                t.to(i,j,a.at(i,j)*b);
            }
        }

        return t;
    }

    inline mat operator *(mat::T b, const mat& a)
    {
        return operator *(a,b);
    }
}

#endif // MATH_MAT_H
