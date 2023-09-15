# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -Wextra -std=c++11 -I./Eigen

# Library flags
LIBS = -lbcm2835 -lfftw3 -lm

# Source files
SRCS = transfer_interferometer_v3.cpp BCM2835_Driver.o ADS1256_Driver.o

# Output executable name
OUTPUT = transfer_interferometer_v3

all: $(OUTPUT)

$(OUTPUT): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(OUTPUT) $(SRCS) $(LIBS)

clean:
	rm -f $(OUTPUT)

.PHONY: all clean
