#-------------------------------------------------
#
# Project created by QtCreator 2016-02-25T10:13:21
#
#-------------------------------------------------

QT       += core gui svg

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GG_calibv2
TEMPLATE = app

# OpenCV
INCLUDEPATH += /usr/local/include/opencv4

LIBS += -LC/usr/local/lib \
             -lopencv_core \
             -lopencv_imgproc \
            -lopencv_highgui\
            -lopencv_calib3d \
            -lopencv_imgcodecs \
            -lopencv_ml \
            -lopencv_objdetect \
            -lopencv_text \


SOURCES += main.cpp\
        #mainwindow.cpp \
        calib.cpp \
        align.cpp

HEADERS  += main.h \
         #mainwindow.h \
         calib.h \
         align.h














