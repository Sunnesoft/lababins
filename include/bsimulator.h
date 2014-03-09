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
    Math::mat _errorInitOrientation{3,1};

    bool _IS_STOPPED;

public:
    BSimulator();
    virtual ~BSimulator();

    inline void setTimeStart(Math::LD ts){ _timeStart = ts; }
    inline void setTimeEnd(Math::LD te){ _timeEnd = te; }
    inline void setStep(Math::LD step){ _step = step; }
    inline void setInitX(const Math::mat &x){ _initX = x;}
    inline void setInitOrientation(const Math::mat &io){ _initOrientation = io;}
    inline void setErrorInitOrientation(const Math::mat &eio){ _errorInitOrientation = eio;}

    inline Math::LD timeStart() const { return _timeStart; }
    inline Math::LD timeEnd() const { return _timeEnd; }
    inline Math::LD step() const { return _timeEnd; }
    inline Math::mat initX() const {return _initX; }
    inline Math::mat initOrientation() const { return _initOrientation;}

    void run();

    QString toResultString(const Math::mat &tx,
                           const Math::mat &bx);

    QString toReportString(const Math::mat &tx,
                           const Math::mat &angles,
                           const Math::mat &bx,
                           const Math::mat &C,
                           const Math::mat &acc,
                           const Math::mat &avs,
                           Math::LD time);

public slots:
    void stop();

signals:
    void result(const QString& log);
    void report(const QString& log, int progress);
    void stopped();
};

#endif // BSIMULATOR_H
