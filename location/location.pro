TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += \
    location.cpp \
    test_location.cpp \
    ../motor/motor.cpp \
    ../vision/vision.cpp

HEADERS += \
    location.hpp \
    capture.hpp

LIBS += `pkg-config --libs opencv`
    -lpthread
