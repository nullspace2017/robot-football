TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += \
    location.cpp \
    test_location.cpp \
    ../motor/motor.cpp \
    ../vision/transform.cpp \
    ../vision/vision.cpp

HEADERS += \
    location.h \
    capture.hpp

LIBS += /usr/local/lib/libopencv_* \
        /usr/lib/libpthread.so
