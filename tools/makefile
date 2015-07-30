CXX := g++
CXXFLAGS := -O3 -Wall -std=c++11

all: histogram

histogram: histogram4.cpp
	$(CXX) $(CXXFLAGS) histogram4.cpp -o histogram `pkg-config --libs opencv`

clean:
	$(RM) histogram

