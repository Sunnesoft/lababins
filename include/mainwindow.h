#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTextStream>
#include <QFile>
#include <QThreadPool>

#include "include/bsimulator.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QFile log;
    QFile results;
    QTextStream lstream;
    QTextStream rstream;

    bool _IS_STARTED;

public slots:
    void run();
    void stop();

    void result(const QString& log);
    void report(const QString& log, int progress);
};

#endif // MAINWINDOW_H
