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
    Sensor.cpp
HEADERS  += mainwindow.h \
    magnet.h \
    posCalculator.h \
    Sensor.h

FORMS    += mainwindow.ui

INCLUDEPATH += C:\Libraries\Eigen \
               C:\Libraries\Alglib\cpp\src

