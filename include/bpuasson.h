#ifndef BPUASSON_H
#define BPUASSON_H

#include "math/mat.h"

class BPuasson
{
private:
    Math::mat _C{3,3};
    Math::LD _time;

public:
    BPuasson(){}
    virtual ~BPuasson(){}

    void setC(const Math::mat& c){_C = c;}
    void setTime(Math::LD t){ _time = t; }

    Math::LD time() const {return _time;}
    Math::mat C() const {return _C;}

    void next(Math::LD step,
              const Math::mat& w1,
              const Math::mat& w2)
    {
        Math::mat W1{3,3};
        Math::mat W2{3,3};

        W1[1] = -w1[2];
        W1[2] = w1[1];
        W1[3] = w1[2];
        W1[5] = -w1[0];
        W1[6] = -w1[1];
        W1[7] = w1[0];

        W2[1] = -w2[2];
        W2[2] = w2[1];
        W2[3] = w2[2];
        W2[5] = -w2[0];
        W2[6] = -w2[1];
        W2[7] = w2[0];

        auto C = _C*W1 - W2*_C;

        _C = _C + step*C;
        _time += step;
    }
};

#endif // BPUASSON_H
