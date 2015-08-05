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
    location.h \
    capture.hpp \
    ../motor/motor.h \
    ../vision/vision.h \
    ../vision/ground.hpp \
    ../vision/transform.hpp \
    ../network/client.hpp \
    ../network/server.hpp

LIBS += `pkg-config --libs opencv` \
    -lpthread
