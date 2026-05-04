# make file for all the things
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude
SPOT_FLAGS := $(shell pkg-config --cflags --libs spot)
RAYLIB_FLAGS := $(shell pkg-config --cflags --libs raylib)

# Core source files (NO main here)
SRC := $(wildcard src/*.cpp)

# Entry points
MAIN_SRC := apps/main.cpp
SIM_SRC  := apps/sim.cpp
TEST_SRC := test.cpp

# Targets
MAIN_TARGET := main
SIM_TARGET  := sim
TEST_TARGET := test

all:
	@echo "Use on of the following; 'make test', 'make main', 'make sim'"

main: $(SRC) $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(MAIN_SRC) $(SPOT_FLAGS) $(RAYLIB_FLAGS) -o $(MAIN_TARGET).out

sim: $(SRC) $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(SIM_SRC) $(SPOT_FLAGS) $(RAYLIB_FLAGS) -o $(SIM_TARGET).out

test: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(SPOT_FLAGS) -o $(TEST_TARGET).out

clean:
	rm -f $(MAIN_TARGET).out $(SIM_TARGET).out $(TEST_TARGET).out