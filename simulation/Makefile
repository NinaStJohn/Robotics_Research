# make file for all the things
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude
SPOT_FLAGS := $(shell pkg-config --cflags --libs spot)

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
	@echo "Use 'make main' or 'make sim'"

main: $(SRC) $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(MAIN_SRC) $(SPOT_FLAGS) -o $(MAIN_TARGET)

sim: $(SRC) $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(SIM_SRC) $(SPOT_FLAGS) -o $(SIM_TARGET)

test: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(SPOT_FLAGS) -o $(TEST_TARGET)

clean:
	rm -f $(MAIN_TARGET) $(SIM_TARGET)