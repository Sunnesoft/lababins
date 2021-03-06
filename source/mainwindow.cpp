#include "include/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _IS_STARTED(0)
{
    ui->setupUi(this);

    connect(ui->stop,SIGNAL(clicked()),this,SLOT(stop()));
    connect(ui->run,SIGNAL(clicked()),this,SLOT(run()));
    ui->progressBar->setVisible(false);
    ui->progressBar->setRange(0,100);
}

void MainWindow::run()
{
    if(_IS_STARTED)
    {
        ui->statusBar->showMessage("Failure: already processing!",3000);
        return;
    }

    _IS_STARTED = true;

    log.close();
    results.close();

    log.setFileName(ui->logpath->text());
    results.setFileName(ui->respath->text());

    if(!log.open(QFile::WriteOnly|QIODevice::Text) ||
       !results.open(QFile::WriteOnly|QIODevice::Text))
    {
        ui->statusBar->showMessage("Failure: invalid filepaths!",3000);
        return;
    }

    lstream.setDevice(&log);
    rstream.setDevice(&results);

    Math::mat x{6,1};
    x[0] = Math::PZ9011::a + ui->h->text().toDouble();
    x[1] = ui->lat->text().toDouble();
    x[2] = ui->lon->text().toDouble();

    Math::LinAlg::sphToCart(x,x);

    x[3] = ui->vx->text().toDouble();
    x[4] = ui->vy->text().toDouble();
    x[5] = ui->vz->text().toDouble()/3600.;

    Math::mat angles{3,1};
    angles[0] = ui->pitch->text().toDouble();
    angles[1] = ui->roll->text().toDouble();
    angles[2] = ui->yaw->text().toDouble();

    Math::mat errorAngles{3,1};
    errorAngles[0] = ui->err_pitch->text().toDouble() * Math::M_PI / 180;
    errorAngles[1] = ui->err_roll->text().toDouble() * Math::M_PI / 180;
    errorAngles[2] = ui->err_yaw->text().toDouble() * Math::M_PI / 180;

    BSimulator *s = new BSimulator();
    s->setTimeStart(ui->tstart->text().toDouble());
    s->setTimeEnd(ui->tend->text().toDouble());
    s->setStep(ui->step->text().toDouble());
    s->setInitX(x);
    s->setInitOrientation(angles);
    s->setErrorInitOrientation(errorAngles);

    connect(s,SIGNAL(report(QString,int)),this,SLOT(report(QString,int)));
    connect(s,SIGNAL(result(QString)),this,SLOT(result(QString)));
    connect(ui->stop,SIGNAL(clicked()),s,SLOT(stop()));
    connect(s,SIGNAL(stopped()),ui->stop,SIGNAL(clicked()));

    QThreadPool::globalInstance()->start(s);

    ui->progressBar->setVisible(true);

    _IS_STARTED = true;
}

void MainWindow::stop()
{
    if(!_IS_STARTED)
    {
        ui->statusBar->showMessage("Failure: already stopped!",3000);
        return;
    }

    _IS_STARTED = false;

    ui->progressBar->setVisible(false);
}

void MainWindow::report(const QString& log, int progress)
{
    lstream<<log;
    lstream.flush();

    ui->progressBar->setValue(progress);
}

void MainWindow::result(const QString& log)
{
    rstream<<log;
    rstream.flush();
}

MainWindow::~MainWindow()
{
    log.close();
    results.close();

    delete ui;
}
