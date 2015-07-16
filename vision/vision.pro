TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    test_video.cpp \
    vision.cpp

HEADERS += \
    vision.h

LIBS += /usr/local/lib/libopencv_*

CONFIG += c++11
