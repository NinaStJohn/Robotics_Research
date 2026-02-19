CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude
SPOT_FLAGS := $(shell pkg-config --cflags --libs spot)

# Source files
SIM_SRC :=  $(wildcard src/*.cpp) # everything in src folder
# TEST_SRC := src/ts_world.cpp src/grid_world.cpp
TEST_SRC := test.cpp

# Output binaries
SIM_TARGET := sim
TEST_TARGET := test

# Default build (build nothing unless specified)
all:
	@echo "Use 'make sim' or 'make test'"

sim: $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) $(SPOT_FLAGS) -o $(SIM_TARGET)

test: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(SPOT_FLAGS) -o $(TEST_TARGET)

clean:
	rm -f $(SIM_TARGET) $(TEST_TARGET)
