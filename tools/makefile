CXX := g++
CXXFLAGS := -O3 -Wall -std=c++11

all: histogram reverse_adjust

histogram: histogram4.cpp
	$(CXX) $(CXXFLAGS) histogram4.cpp -o histogram `pkg-config --libs opencv`

reverse_adjust: reverse_adjust.cpp
	$(CXX) $(CXXFLAGS) reverse_adjust.cpp -o reverse_adjust `pkg-config --libs opencv`

clean:
	$(RM) histogram reverse_adjust

