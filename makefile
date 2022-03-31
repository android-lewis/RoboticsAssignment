# makefile to compile lab asignments.
# Inaki Rano i.rano@ulster.ac.uk
#

# Set compiler and compiling options
CXX = g++
CXXFLAGS = `pkg-config --cflags playerc++`
LDFLAGS = `pkg-config --libs playerc++`

# Source files and program to compile
TARGET = robot-control
SRC = robot-control.cc
OBJ = $(SRC:.cc=.o)

# Compilation rules
%: %.o
	$(CXX) $< -o $@ $(LDFLAGS)

robot-control: robot-control.o
robot-control.o: robot-control.cc

clean:
	rm $(OBJ) $(TARGET)