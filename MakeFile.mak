# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Iinclude -std=c++17 -Wall

# Source files and object files
SRC = src/main.cpp src/aero.cpp src/forceApplied.cpp src/odeIterator.cpp src/rotationMatrix.cpp src/vectorMath.cpp src/vehicle.cpp
OBJ = $(SRC:.cpp=.o)

# Output executable
TARGET = build/main

# Default rule to build the program
all: $(TARGET)

# Rule to link the object files and create the executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJ)

# Rule to compile each source file into an object file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJ) $(TARGET)
