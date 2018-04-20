#-------------------------------------------------
#
# Project created by QtCreator 2018-03-12T10:24:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG +=c++14

TARGET = Image
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    CImage.cpp \
    CPyramid.cpp \
    CDescriptor.cpp \
    CHistogram.cpp \
    CImageHandler.cpp

HEADERS += \
        mainwindow.h \
    CImage.h \
    CImageKernels.h \
    CMatrixV.h \
    CImageHandler.h \
    CPyramid.h \
    CDescriptor.h \
    CHistogram.h

FORMS += \
        mainwindow.ui
