#include "include/bsimulator.h"

BSimulator::BSimulator():
    _IS_STOPPED(0)
{

}

BSimulator::~BSimulator()
{

}

void BSimulator::run()
{
    _IS_STOPPED = false;

    BMotion motion;
    motion.setX(_initX);
    motion.setTime(_timeStart);
    motion.setOrientation(_initOrientation);

    BAvsBlock avs;
    BAccBlock acc;
    BBins bins;

    bins.init(_timeStart, _initX, _initOrientation + _errorInitOrientation);

    for(Math::LD time = _timeStart; time <= _timeEnd && !_IS_STOPPED; time += _step)
    {
        Math::mat sph{3,1,Math::mat::Undefined};

        motion.next(_step);

        auto x = motion.x();
        auto angles = motion.orientation();
        Math::LinAlg::cartToSph(x,sph);

        auto accMeasure = acc.measure(x,sph,angles);
        auto avsMeasure = avs.measure(x,sph,angles);

        bins.next(_step,accMeasure,avsMeasure);
        auto bx = bins.x();
        auto C = bins.orientation();

        emit result(toResultString(x,bx,time));
        emit report(toReportString(
            x,angles,_errorInitOrientation,
            bx,C,accMeasure,avsMeasure,time
        ),100*time/_timeEnd);
    }

    emit stopped();
}

QString BSimulator::toResultString(const Math::mat &tx,
                                   const Math::mat &bx,
                                   Math::LD time)
{
    return QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13\n")
            .arg(tx[0],0,'g',10)
            .arg(tx[1],0,'g',10)
            .arg(tx[2],0,'g',10)
            .arg(tx[3],0,'g',10)
            .arg(tx[4],0,'g',10)
            .arg(tx[5],0,'g',10)
            .arg(bx[0],0,'g',10)
            .arg(bx[1],0,'g',10)
            .arg(bx[2],0,'g',10)
            .arg(bx[3],0,'g',10)
            .arg(bx[4],0,'g',10)
            .arg(bx[5],0,'g',10)
            .arg(time,0,'g',10);
}

QString BSimulator::toReportString(const Math::mat &tx,
                                   const Math::mat &angles,
                                   const Math::mat &eangles,
                                   const Math::mat &bx,
                                   const Math::mat &C,
                                   const Math::mat &acc,
                                   const Math::mat &avs,
                                   Math::LD time)
{
    return QString(
                "Current time: %1\n"
                ">>>\tTrue motion params [x y z | vx vy vz]{km|km/sec}:\n %2"
                ">>>\tOrientation angles [pitch roll yaw]:\n %3"
                ">>>\tError of orientation angles [pitch roll yaw]:\n %4"
                ">>>\tMotion params from bins [x y z | vx vy vz]{km|km/sec}:\n %5"
                ">>>\tOrientation matrix [3x3]:\n %6"
                ">>>\tMeasures by acceler.:\n %7"
                ">>>\tMeasures by avs.:\n %8\n"
                )
            .arg((double)time,0,'g',10)
            .arg(tx.tr().operator std::string().c_str())
            .arg(angles.tr().operator std::string().c_str())
            .arg(eangles.tr().operator std::string().c_str())
            .arg(bx.tr().operator std::string().c_str())
            .arg(C.operator std::string().c_str())
            .arg(acc.tr().operator std::string().c_str())
            .arg(avs.tr().operator std::string().c_str());
}

void BSimulator::stop()
{
    _IS_STOPPED = true;
}

