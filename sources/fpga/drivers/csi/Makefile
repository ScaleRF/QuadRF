obj-m += fpga-csi.o

KREL ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KREL)/build

DTC ?= dtc
SUDO ?= sudo

INSTALL_MOD_DIR ?= updates/dkms
DTS := fpga-csi.dts
DTBO := fpga-csi.dtbo

OVERLAY_DIR ?= /boot/firmware/overlays

default: module

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
	@test -d "$(OVERLAY_DIR)" || { \
		echo "Overlay directory not found: $(OVERLAY_DIR)" >&2; \
		exit 1; \
	}
	$(SUDO) $(MAKE) -C "$(KDIR)" \
		M="$(CURDIR)" \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)" \
		modules_install
	$(SUDO) depmod -a "$(KREL)"
	$(SUDO) install -m 0644 "$(DTBO)" \
		"$(OVERLAY_DIR)/$(DTBO)"
	@if [ -d /usr/lib/quadrf/overlays ]; then \
		$(SUDO) install -m 0644 "$(DTBO)" /usr/lib/quadrf/overlays/"$(DTBO)"; \
	fi
	@echo "Installed fpga-csi for kernel $(KREL). Reboot to load it."

clean:
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" clean
	$(RM) "$(DTBO)"

.PHONY: default all module overlay install clean
