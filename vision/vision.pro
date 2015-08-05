TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += \
    vision.cpp \
    test_vision.cpp

HEADERS += \
    vision.h \
    ground.hpp \
    transform.hpp

LIBS += `pkg-config --libs opencv`
