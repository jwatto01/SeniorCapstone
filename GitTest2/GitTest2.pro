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
    ../../../../Senior Design/Senior Design Code/C++ Software/magnet.cpp \
    ../../../../Senior Design/Senior Design Code/C++ Software/posCalculator.cpp \
    ../../../../Senior Design/Senior Design Code/C++ Software/Sensor.cpp \
    posCalculator.cpp \
    Sensor.cpp \
    magnet.cpp

HEADERS  += mainwindow.h \
    ../../../../Senior Design/Senior Design Code/C++ Software/magnet.h \
    ../../../../Senior Design/Senior Design Code/C++ Software/Sensor.h \
    magnet.h \
    Sensor.h \
    posCalculator.h

FORMS    += mainwindow.ui
