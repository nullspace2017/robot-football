CXX      = g++
CXXFLAGS = -Wall -O3 -std=c++11
HEADERS  = ../vision/vision.h ../vision/transform.h ../vision/ground.hpp ../motor/motor.h ../location/location.h ../location/capture.hpp 
SOURCES  = ../vision/vision.cpp ../vision/transform.cpp ../motor/motor.cpp ../location/location.cpp

all: circle_toy

circle_toy: circle_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ `pkg-config --libs opencv`

clean:
	$(RM) *_toy

