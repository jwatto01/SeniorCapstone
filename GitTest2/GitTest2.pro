#-------------------------------------------------
#
# Project created by QtCreator 2017-01-25T18:02:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GitTest2
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    magnet.cpp \
    posCalculator.cpp \
    Sensor.cpp \
    alglibinternal.cpp \
    alglibmisc.cpp \
    ap.cpp \
    dataanalysis.cpp \
    diffequations.cpp \
    fasttransforms.cpp \
    integration.cpp \
    interpolation.cpp \
    linalg.cpp \
    optimization.cpp \
    solvers.cpp \
    specialfunctions.cpp \
    statistics.cpp
HEADERS  += mainwindow.h \
    magnet.h \
    posCalculator.h \
    Sensor.h \
    alglibinternal.h \
    alglibmisc.h \
    ap.h \
    dataanalysis.h \
    diffequations.h \
    fasttransforms.h \
    integration.h \
    interpolation.h \
    linalg.h \
    optimization.h \
    solvers.h \
    specialfunctions.h \
    statistics.h \
    stdafx.h

FORMS    += mainwindow.ui

INCLUDEPATH += C:\Libraries\Eigen \
               C:\Libraries\Alglib\cpp\src \

