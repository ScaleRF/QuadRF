CXX ?= g++
TARGET := quadrf-ntsc-demod

CPPFLAGS += $(shell pkg-config --cflags SoapySDR 2>/dev/null)
CXXFLAGS += -O3 -march=native -mtune=native -ffast-math -flto -DNDEBUG -std=c++17 -pthread
LDLIBS += $(shell pkg-config --libs SoapySDR 2>/dev/null) -lSoapySDR -pthread

.PHONY: all clean

all: $(TARGET)

$(TARGET): ntsc_demod.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	$(RM) $(TARGET)
