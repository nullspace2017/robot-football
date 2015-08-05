CXX      = g++
CXXFLAGS = -Wall -O3 -std=c++11
HEADERS  = ../vision/vision.h ../vision/transform.hpp ../vision/ground.hpp ../motor/motor.h ../location/location.h ../location/capture.hpp ../network/server.hpp ../network/client.hpp
SOURCES  = ../vision/vision.cpp ../motor/motor.cpp ../location/location.cpp

<<<<<<< HEAD
all: circle_toy biarc_toy hit_toy

hit_toy: hit_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ `pkg-config --libs opencv` -lpthread

circle_toy: circle_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ `pkg-config --libs opencv` -lpthread
=======
all: circle_toy biarc_toy net_server_toy

circle_toy: circle.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ `pkg-config --libs opencv`
>>>>>>> bb2d47d81b32c6f344fc1f8c22f81a44393483ed

biarc_toy: biarc/arc_test.cpp biarc/double_arc.cpp biarc/double_arc.h $(HEADERS) $(SOURCES)
	$(CXX) biarc/arc_test.cpp biarc/double_arc.cpp $(SOURCES) $(CXXFLAGS) -I.. -I. -o $@ `pkg-config --libs opencv` -lpthread

net_server_toy: net_server.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ `pkg-config --libs opencv`

clean:
	$(RM) *_toy
