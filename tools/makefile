CXX := g++
CXXFLAGS := -O3 -Wall -std=c++11

all: histogram reverse_adjust calib network_viewer

histogram: histogram4.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ `pkg-config --libs opencv`

reverse_adjust: reverse_adjust.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ `pkg-config --libs opencv`

calib: calib.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ `pkg-config --libs opencv`

network_viewer: network_viewer.cpp ../vision/ground.hpp ../network/client.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ `pkg-config --libs opencv` -lpthread

clean:
	$(RM) histogram reverse_adjust calib network_viewer

