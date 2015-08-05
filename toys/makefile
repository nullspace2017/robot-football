CXX      = g++
CXXFLAGS = -Wall -O3 -std=c++11
HEADERS  = ../vision/vision.h ../vision/transform.hpp ../vision/ground.hpp ../motor/motor.h ../location/location.h ../location/capture.hpp ../network/server.hpp ../network/client.hpp
SOURCES  = ../vision/vision.cpp ../motor/motor.cpp ../location/location.cpp
LIBS = `pkg-config --libs opencv` -lpthread

all: circle_toy biarc_toy hit_toy net_server_toy net_client_toy goalkeeper_toy

hit_toy: hit_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)

circle_toy: circle.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)

biarc_toy: biarc/arc_test.cpp biarc/double_arc.cpp biarc/double_arc.h $(HEADERS) $(SOURCES)
	$(CXX) biarc/arc_test.cpp biarc/double_arc.cpp $(SOURCES) $(CXXFLAGS) -I.. -I. -o $@ $(LIBS)

net_server_toy: net_server.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)

net_client_toy: net_client.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)

goalkeeper_toy: goalkeeper_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)

square_toy: square_toy.cpp $(HEADERS) $(SOURCES)
	$(CXX) $< $(SOURCES) $(CXXFLAGS) -o $@ $(LIBS)


clean:
	$(RM) *_toy

