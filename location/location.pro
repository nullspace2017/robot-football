TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += \
    location.cpp \
    test_location.cpp \
    ../motor/motor.cpp

HEADERS += \
    location.h

INCLUDEPATH += ../motor/ \
    ../vision/
