#-------------------------------------------------
#
# Project created by QtCreator 2020-01-07T10:45:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = openglwidget
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    DoubleGrayCode.cpp \
    calibration.cpp \
    findnew.cpp \
    graycode.cpp \
    instance.cpp \
        main.cpp \
        mainwindow.cpp \
    MinBoundingBox.cpp \
    myqopenglwidget.cpp

HEADERS += \
    DoubleGrayCode.h \
    calibration.h \
    findnew.h \
    graycode.h \
    instance.h \
        mainwindow.h \
    MinBoundingBox.h \
    myqopenglwidget.h

FORMS += \
        mainwindow.ui

INCLUDEPATH+=D:\environment\qt-5.14.2\opencv\newbuild\install\include \
        D:\environment\qt-5.14.2\opencv\newbuild\install\include\opencv2

LIBS += D:\environment\qt-5.14.2\opencv\newbuild\install\x64\mingw\bin\libopencv_world453.dll \
             D:\environment\qt-5.14.2\opencv\newbuild\install\x64\mingw\lib\libopencv_world453.dll.a \
