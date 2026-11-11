CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -Wall -Wextra -Wpedantic
INCLUDES = -Iinclude

.PHONY: all test clean
all: lanegt

lanegt: src/main.cpp include/lanegt/LaneGTCore.hpp include/lanegt/Graph.hpp include/lanegt/Triangle.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/main.cpp -o lanegt

smoke: tests/smoke.cpp include/lanegt/LaneGTCore.hpp include/lanegt/Graph.hpp include/lanegt/Triangle.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/smoke.cpp -o smoke

test: smoke
	./smoke

clean:
	rm -f lanegt smoke
