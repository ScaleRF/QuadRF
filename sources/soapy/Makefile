CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -fPIC -ftree-vectorize -funsafe-math-optimizations -ffast-math -flto
ifneq (,$(findstring aarch64,$(shell uname -m)))
  CXXFLAGS += -mcpu=cortex-a76 -mtune=cortex-a76
endif

SOAPY_MOD_DIR ?= $(shell pkg-config --variable=libdir SoapySDR)/SoapySDR/modules0.8
SUDO ?= sudo

TARGET := libmipi.so
SRCS := Registration.cpp MipiDevice.cpp NEON.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -I. -I/usr/include/quadrf -shared $^ $(shell pkg-config --cflags --libs SoapySDR) -o $@

install: $(TARGET)
	@test -n "$(SOAPY_MOD_DIR)" || { echo "SoapySDR module directory not found" >&2; exit 1; }
	$(SUDO) install -d "$(SOAPY_MOD_DIR)"
	$(SUDO) install -m 0644 "$(TARGET)" "$(SOAPY_MOD_DIR)/$(TARGET)"
	@echo "Installed $(TARGET) to $(SOAPY_MOD_DIR)"

clean:
	rm -f $(TARGET)

.PHONY: all install clean
