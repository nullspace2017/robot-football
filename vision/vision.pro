TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += \
    test_video.cpp \
    vision.cpp \
    transform.cpp

HEADERS += \
    vision.h \
    transform.h \
    ground.hpp

LIBS += /usr/local/lib/libopencv_*
