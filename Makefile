# Compiler
CXX = g++

# OpenCV pkg-config
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS = $(shell pkg-config --libs opencv4)

# Compiler flags
CXXFLAGS = -Wall -Wextra -std=c++23 $(OPENCV_CFLAGS)

# Directories
SRCDIR = src
OBJDIR = obj
BINDIR = bin

# Target executable
TARGET = $(BINDIR)/DroneCtrl

# Source files
SRCS = $(wildcard $(SRCDIR)/*.cpp) main.cpp

# Object files
OBJS = $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Include directories
INCLUDES = -I$(SRCDIR)

# Create directories if they don't exist
$(shell mkdir -p $(OBJDIR) $(BINDIR) $(addprefix $(OBJDIR)/, $(dir $(SRCS))))

# Default target
all: $(TARGET)

# Link the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(OPENCV_LIBS)

# Compile source files into object files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile main.cpp separately
$(OBJDIR)/main.o: main.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean up build files
clean:
	rm -rf $(OBJDIR) $(BINDIR)

# Phony targets
.PHONY: all clean