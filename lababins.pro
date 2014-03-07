#-------------------------------------------------
#
# Project created by QtCreator 2014-02-08T23:04:45
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = lababins
TEMPLATE = app

CONFIG  += c++11

SOURCES += main.cpp\
           ./source/mainwindow.cpp \
    source/bsimulator.cpp


HEADERS  += ./include/mainwindow.h \
            ./include/bmodel.h \
            ./include/bsimulator.h \
            ./include/bconv.h \
            ./include/bmotion.h \
            ./include/bbins.h \
            ./include/bpuasson.h

FORMS    += mainwindow.ui
