CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -I. -I/usr/include/quadrf -I../drivers/csi
PREFIX ?= /usr
SUDO ?= sudo

TARGET := quadrf-jtag
SRCS := jtag.c max285x.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $^ -lm -o $@

install: $(TARGET)
	$(SUDO) install -d "$(PREFIX)/bin"
	$(SUDO) install -m 0755 "$(TARGET)" "$(PREFIX)/bin/$(TARGET)"
	@echo "Installed $(TARGET) to $(PREFIX)/bin"

clean:
	rm -f $(TARGET)

.PHONY: all install clean
