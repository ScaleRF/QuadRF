obj-m += fpga-csi.o

KREL ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KREL)/build

DTC ?= dtc
SUDO ?= sudo

INSTALL_MOD_DIR ?= updates/quadrf
DTS := fpga-csi.dts
DTBO := fpga-csi.dtbo

# Raspberry Pi OS/DietPi normally uses /boot/firmware. Retain compatibility
# with older installations that use /boot directly.
OVERLAY_DIR ?= $(shell \
	if [ -d /boot/firmware/overlays ]; then \
		printf '%s' /boot/firmware/overlays; \
	elif [ -d /boot/overlays ]; then \
		printf '%s' /boot/overlays; \
	fi)

all: module overlay

module:
	@test -f "$(KDIR)/Makefile" || { \
		echo "Kernel build directory not found: $(KDIR)" >&2; \
		echo "Install headers for kernel $(KREL)." >&2; \
		exit 1; \
	}
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" modules

overlay: $(DTBO)

$(DTBO): $(DTS)
	@command -v "$(DTC)" >/dev/null 2>&1 || { \
		echo "dtc not found; install device-tree-compiler." >&2; \
		exit 1; \
	}
	$(DTC) -@ -H epapr -I dts -O dtb -o "$@" "$<"
	$(DTC) -I dtb -O dts -o /dev/null "$@"

install: all
	@test -n "$(OVERLAY_DIR)" || { \
		echo "Raspberry Pi overlay directory not found." >&2; \
		exit 1; \
	}
	$(SUDO) $(MAKE) -C "$(KDIR)" \
		M="$(CURDIR)" \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)" \
		modules_install
	$(SUDO) depmod -a "$(KREL)"
	$(SUDO) install -m 0644 "$(DTBO)" \
		"$(OVERLAY_DIR)/$(DTBO)"
	@echo "Installed fpga-csi for kernel $(KREL). Reboot to load it."

clean:
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" clean
	$(RM) "$(DTBO)"

.PHONY: all module overlay install clean
