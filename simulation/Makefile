CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2
SPOT_FLAGS := $(shell pkg-config --cflags --libs spot)

TARGET := test
SRC := ts_world.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(SPOT_FLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)
