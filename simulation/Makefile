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
REPLANTEST_SRC := apps/replan_test.cpp
TESTS_SRC := tests/replan_tests.cpp

# Headless core: everything except the raylib visualizers
HEADLESS_SRC := $(filter-out src/grid_vis.cpp src/LTL_vis.cpp, $(SRC))

# Targets
MAIN_TARGET := main
SIM_TARGET  := sim
TEST_TARGET := test
REPLANTEST_TARGET := replantest
TESTS_TARGET := replan_tests

# 'tests' collides with the tests/ directory; mark phony so it always runs.
.PHONY: all main sim test replantest tests clean

all:
	@echo "Use on of the following; 'make test', 'make main', 'make sim', 'make replantest', 'make tests'"

replantest: $(HEADLESS_SRC) $(REPLANTEST_SRC)
	$(CXX) $(CXXFLAGS) $(HEADLESS_SRC) $(REPLANTEST_SRC) $(SPOT_FLAGS) -o $(REPLANTEST_TARGET).out

# Replanning correctness suite (invariants + incremental-vs-rebuild + fuzz)
tests: $(HEADLESS_SRC) $(TESTS_SRC)
	$(CXX) $(CXXFLAGS) $(HEADLESS_SRC) $(TESTS_SRC) $(SPOT_FLAGS) -o $(TESTS_TARGET).out

main: $(SRC) $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(MAIN_SRC) $(SPOT_FLAGS) $(RAYLIB_FLAGS) -o $(MAIN_TARGET).out

sim: $(SRC) $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(SIM_SRC) $(SPOT_FLAGS) $(RAYLIB_FLAGS) -o $(SIM_TARGET).out

test: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(SPOT_FLAGS) -o $(TEST_TARGET).out

clean:
	rm -f $(MAIN_TARGET).out $(SIM_TARGET).out $(TEST_TARGET).out $(REPLANTEST_TARGET).out $(TESTS_TARGET).out