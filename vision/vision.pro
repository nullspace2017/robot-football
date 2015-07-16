TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    test_video.cpp \
    vision.cpp \
    transform.cpp

HEADERS += \
    vision.h \
    transform.h

LIBS += /usr/local/lib/libopencv_*

CONFIG += c++11
