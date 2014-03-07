#ifndef BSIMULATOR_H
#define BSIMULATOR_H

#include "include/bmotion.h"
#include "include/bbins.h"

#include <QObject>
#include <QRunnable>

class BSimulator : public QObject, public QRunnable
{
    Q_OBJECT

    Math::LD _timeStart;
    Math::LD _timeEnd;
    Math::LD _step;

    Math::mat _initX{6,1};
    Math::mat _initOrientation{3,1};

    bool _IS_STOPPED;

public:
    BSimulator():
        _IS_STOPPED(0)
    {}
    virtual ~BSimulator(){}

    void setTimeStart(Math::LD ts){ _timeStart = ts; }
    void setTimeEnd(Math::LD te){ _timeEnd = te; }
    void setStep(Math::LD step){ _step = step; }
    void setInitX(const Math::mat &x){ _initX = x;}
    void setInitOrientation(const Math::mat &io){ _initOrientation = io;}

    Math::LD timeStart() const { return _timeStart; }
    Math::LD timeEnd() const { return _timeEnd; }
    Math::LD step() const { return _timeEnd; }
    Math::mat initX() const {return _initX; }
    Math::mat initOrientation() const { return _initOrientation;}

    void run()
    {
        _IS_STOPPED = false;

        BMotion motion;
        motion.setX(_initX);
        motion.setTime(_timeStart);
        motion.setOrientation(_initOrientation);

        BAvsBlock avs;
        BAccBlock acc;
        BBins bins;

        bins.init(_timeStart, _initX, _initOrientation);

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

            emit result(toResultString(x,bx));
            emit report(toReportString(x,angles,bx,C,accMeasure,avsMeasure,time),100*time/_timeEnd);
        }

        emit stopped();
    }

    QString toResultString(const Math::mat &tx,
                        const Math::mat &bx)
    {
        return QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12\n")
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
                    .arg(bx[5],0,'g',10);
    }

    QString toReportString(const Math::mat &tx,
                        const Math::mat &angles,
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
            ">>>\tMotion params from bins [x y z | vx vy vz]{km|km/sec}:\n %4"
            ">>>\tOrientation matrix [3x3]:\n %5"
            ">>>\tMeasures by acceler.:\n %6"
            ">>>\tMeasures by avs.:\n %7\n"
        )
        .arg((double)time,0,'g',10)
        .arg(tx.tr().operator std::string().c_str())
        .arg(angles.tr().operator std::string().c_str())
        .arg(bx.tr().operator std::string().c_str())
        .arg(C.operator std::string().c_str())
        .arg(acc.tr().operator std::string().c_str())
        .arg(avs.tr().operator std::string().c_str());
    }

public slots:
    void stop(){_IS_STOPPED = true;}

signals:
    void result(const QString& log);
    void report(const QString& log, int progress);
    void stopped();
};

#endif // BSIMULATOR_H
