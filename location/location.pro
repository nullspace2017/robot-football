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

INCLUDEPATH += ../motor/ \
    ../vision/

LIBS += /usr/local/lib/libopencv_*
