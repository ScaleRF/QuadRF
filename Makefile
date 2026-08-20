# QuadRF build and install.
#
# Everything the appliance ships is built here and installed under DESTDIR;
# debian/rules only calls into these targets. Component install targets match
# the binary packages one for one.

VERSION      ?= $(shell dpkg-parsechangelog -SVersion 2>/dev/null | cut -d- -f1)
VERSION      := $(if $(VERSION),$(VERSION),1.0.0)

DESTDIR      ?=
prefix       ?= /usr
bindir        = $(prefix)/bin
sbindir       = $(prefix)/sbin
libdir        = $(prefix)/lib/quadrf
datadir       = $(prefix)/share/quadrf
icondir       = $(prefix)/share/icons/hicolor/scalable/apps
appdir        = $(prefix)/share/applications
unitdir       = /lib/systemd/system
mandir        = $(prefix)/share/man
includedir    = $(prefix)/include/quadrf
srcdir        = $(prefix)/src
sysconfdir   ?= /etc
statedir     ?= /var/lib/quadrf

# SoapySDR.pc has libdir but no module directory, so assemble the path the
# loader scans: <libdir>/SoapySDR/modules<major>.<minor>.
SOAPY_LIBDIR    := $(shell pkg-config --variable=libdir SoapySDR 2>/dev/null)
SOAPY_ABI       := $(shell pkg-config --modversion SoapySDR 2>/dev/null | cut -d. -f1,2)
SOAPY_MODULEDIR ?= $(SOAPY_LIBDIR)/SoapySDR/modules$(SOAPY_ABI)

BUILD        := build
SRC          := sources
CSI_HEADERS  := $(SRC)/fpga/drivers/csi

DEB_HOST_ARCH ?= $(shell dpkg --print-architecture 2>/dev/null)
ifeq ($(DEB_HOST_ARCH),arm64)
TUNE_FLAGS   ?= -mcpu=cortex-a76 -mtune=cortex-a76
endif

CFLAGS       ?= -O2 -Wall
CXXFLAGS     ?= -O2 -Wall
DEMO_FLAGS   := -O3 -flto $(TUNE_FLAGS)
INSTALL      := install

# Split along Architecture: all vs arm64 so debian/rules can build and stage the
# two halves separately.
INSTALL_INDEP := install-common install-boot install-fpga-dkms install-gui \
                 install-network install-desktop install-gnuradio install-ups
INSTALL_ARCH  := install-fpga install-soapy install-demos
INSTALL_TARGETS := $(INSTALL_INDEP) $(INSTALL_ARCH)

all: build-indep build-arch

build-indep: overlays

build-arch: $(BUILD)/quadrf-jtag soapy-modules demos

# --- radio front-end -------------------------------------------------------

$(BUILD)/quadrf-jtag: $(SRC)/fpga/jtag_src/jtag.c $(SRC)/fpga/jtag_src/max285x.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $^ -lm

# --- device-tree overlays --------------------------------------------------

OVERLAYS := $(BUILD)/overlays/fpga-csi.dtbo $(BUILD)/overlays/fpga-dsi.dtbo

overlays: $(OVERLAYS)

$(BUILD)/overlays/%.dtbo: $(SRC)/fpga/drivers/csi/%.dts
	@mkdir -p $(dir $@)
	dtc -@ -I dts -O dtb -o $@ $<

$(BUILD)/overlays/fpga-dsi.dtbo: $(SRC)/fpga/drivers/dsi/fpga-dsi.dts
	@mkdir -p $(dir $@)
	dtc -@ -I dts -O dtb -o $@ $<

# --- SoapySDR modules ------------------------------------------------------

soapy-modules: $(BUILD)/soapy-mipi/libmipi.so $(BUILD)/soapy-quadrf/quadrf.so

$(BUILD)/soapy-mipi/libmipi.so:
	cmake -S $(SRC)/soapy -B $(BUILD)/soapy-mipi -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD)/soapy-mipi

$(BUILD)/soapy-quadrf/quadrf.so:
	cmake -S $(SRC)/quadrfd/soapy -B $(BUILD)/soapy-quadrf -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD)/soapy-quadrf

# --- demo applications -----------------------------------------------------

DEMOS := $(BUILD)/demos/quadrf-rf-vision \
         $(BUILD)/demos/quadrf-psd \
         $(BUILD)/demos/quadrf-ntsc-demod

demos: $(DEMOS)

$(BUILD)/demos/quadrf-rf-vision: $(SRC)/demos/csi_sweep.c $(SRC)/demos/mongoose.c
	@mkdir -p $(dir $@)
	$(CC) $(DEMO_FLAGS) $(CPPFLAGS) -I$(CSI_HEADERS) -o $@ $^ \
		$(LDFLAGS) -lfftw3f -lSDL2 -lm -lpthread

$(BUILD)/demos/quadrf-psd: $(SRC)/demos/psd_switchable.c
	@mkdir -p $(dir $@)
	$(CC) $(DEMO_FLAGS) $(CPPFLAGS) -I$(CSI_HEADERS) -o $@ $^ \
		$(LDFLAGS) -lfftw3f -lSDL2 -lm

$(BUILD)/demos/quadrf-ntsc-demod: $(SRC)/demos/ntsc_demod.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DEMO_FLAGS) -ffast-math -std=c++17 $(CPPFLAGS) -o $@ $< \
		$(LDFLAGS) $(shell pkg-config --cflags --libs SoapySDR)

# --- install ---------------------------------------------------------------

install: install-indep install-arch

install-indep: $(INSTALL_INDEP)

install-arch: $(INSTALL_ARCH)

install-common:
	$(INSTALL) -D -m 755 $(SRC)/cli/quadrf $(DESTDIR)$(bindir)/quadrf
	$(INSTALL) -D -m 644 $(SRC)/common/functions.sh $(DESTDIR)$(datadir)/functions.sh
	$(INSTALL) -D -m 644 $(SRC)/common/quadrf.conf $(DESTDIR)$(sysconfdir)/quadrf/quadrf.conf
	$(INSTALL) -D -m 644 $(SRC)/man/quadrf.1 $(DESTDIR)$(mandir)/man1/quadrf.1
	$(INSTALL) -D -m 644 $(SRC)/man/quadrf.conf.5 $(DESTDIR)$(mandir)/man5/quadrf.conf.5
	$(INSTALL) -d -m 755 $(DESTDIR)$(libdir)/apply.d $(DESTDIR)$(statedir)

install-boot: overlays
	$(INSTALL) -D -m 644 $(BUILD)/overlays/fpga-csi.dtbo $(DESTDIR)$(libdir)/overlays/fpga-csi.dtbo
	$(INSTALL) -D -m 644 $(BUILD)/overlays/fpga-dsi.dtbo $(DESTDIR)$(libdir)/overlays/fpga-dsi.dtbo
	$(INSTALL) -D -m 755 $(SRC)/boot/10-boot $(DESTDIR)$(libdir)/apply.d/10-boot
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-reboot-clear.service \
		$(DESTDIR)$(unitdir)/quadrf-reboot-clear.service

install-fpga: $(BUILD)/quadrf-jtag
	$(INSTALL) -D -m 755 $(BUILD)/quadrf-jtag $(DESTDIR)$(bindir)/quadrf-jtag
	$(INSTALL) -D -m 755 $(SRC)/fpga/quadrf-load $(DESTDIR)$(sbindir)/quadrf-load
	$(INSTALL) -D -m 644 $(SRC)/fpga/quadrf.svf $(DESTDIR)$(datadir)/fpga/quadrf.svf
	$(INSTALL) -D -m 644 $(SRC)/fpga/board/rpi5_lfe5u45f.cfg \
		$(DESTDIR)$(datadir)/openocd/board/rpi5_lfe5u45f.cfg
	$(INSTALL) -D -m 644 $(SRC)/fpga/interface/rpi5_ecp5_gpio.cfg \
		$(DESTDIR)$(datadir)/openocd/interface/rpi5_ecp5_gpio.cfg
	$(INSTALL) -D -m 644 $(CSI_HEADERS)/fpga_csi.h $(DESTDIR)$(includedir)/fpga_csi.h
	$(INSTALL) -D -m 644 $(SRC)/systemd/load-quadrf.service $(DESTDIR)$(unitdir)/load-quadrf.service
	$(INSTALL) -D -m 644 $(SRC)/man/quadrf-jtag.1 $(DESTDIR)$(mandir)/man1/quadrf-jtag.1
	$(INSTALL) -D -m 644 $(SRC)/man/quadrf-load.8 $(DESTDIR)$(mandir)/man8/quadrf-load.8

install-fpga-dkms:
	$(INSTALL) -d -m 755 $(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/csi \
		$(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/dsi
	sed 's|@VERSION@|$(VERSION)|g' $(SRC)/fpga/drivers/dkms.conf.in \
		> $(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/dkms.conf
	chmod 644 $(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/dkms.conf
	$(INSTALL) -m 644 $(SRC)/fpga/drivers/csi/fpga-csi.c $(SRC)/fpga/drivers/csi/fpga_csi.h \
		$(SRC)/fpga/drivers/csi/Makefile $(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/csi/
	$(INSTALL) -m 644 $(SRC)/fpga/drivers/dsi/fpga-dsi.c $(SRC)/fpga/drivers/dsi/Makefile \
		$(DESTDIR)$(srcdir)/quadrf-fpga-$(VERSION)/dsi/

install-soapy: soapy-modules
	$(INSTALL) -D -m 644 $(BUILD)/soapy-mipi/libmipi.so $(DESTDIR)$(SOAPY_MODULEDIR)/libmipi.so
	$(INSTALL) -D -m 644 $(BUILD)/soapy-quadrf/quadrf.so $(DESTDIR)$(SOAPY_MODULEDIR)/quadrf.so
	$(INSTALL) -D -m 644 $(SRC)/soapy/sysctl/60-quadrf-soapy-server.conf \
		$(DESTDIR)$(prefix)/lib/sysctl.d/60-quadrf-soapy-server.conf
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-soapy-server.service \
		$(DESTDIR)$(unitdir)/quadrf-soapy-server.service

install-gui:
	$(INSTALL) -D -m 644 $(SRC)/flask/app.py $(DESTDIR)$(datadir)/gui/app.py
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/gui/templates $(SRC)/flask/templates/*
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/gui/static $(SRC)/flask/static/*
	$(INSTALL) -D -m 755 $(SRC)/flask/quadrf-app $(DESTDIR)$(sbindir)/quadrf-app
	$(INSTALL) -D -m 644 $(SRC)/flask/apps.json $(DESTDIR)$(datadir)/apps.json
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-gui.service $(DESTDIR)$(unitdir)/quadrf-gui.service
	$(INSTALL) -D -m 755 $(SRC)/flask/30-gui $(DESTDIR)$(libdir)/apply.d/30-gui
	$(INSTALL) -D -m 644 $(SRC)/man/quadrf-app.8 $(DESTDIR)$(mandir)/man8/quadrf-app.8

install-network:
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-hotspot $(DESTDIR)$(sbindir)/quadrf-hotspot
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-ethernet $(DESTDIR)$(sbindir)/quadrf-ethernet
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-usb $(DESTDIR)$(sbindir)/quadrf-usb
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-apply-wifi $(DESTDIR)$(sbindir)/quadrf-apply-wifi
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-apply-ap $(DESTDIR)$(sbindir)/quadrf-apply-ap
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-dns-update $(DESTDIR)$(sbindir)/quadrf-dns-update
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-mdns-alias $(DESTDIR)$(sbindir)/quadrf-mdns-alias
	$(INSTALL) -D -m 755 $(SRC)/network/bin/quadrf-tls-setup $(DESTDIR)$(libdir)/tls-setup
	$(INSTALL) -D -m 755 $(SRC)/network/if-up.d/quadrf-dns $(DESTDIR)$(sysconfdir)/network/if-up.d/quadrf-dns
	$(INSTALL) -D -m 644 $(SRC)/network/dnsmasq/quadrf.conf $(DESTDIR)$(sysconfdir)/dnsmasq.d/quadrf.conf
	$(INSTALL) -D -m 644 $(SRC)/network/modprobe.d/quadrf-g-ether.conf \
		$(DESTDIR)$(sysconfdir)/modprobe.d/quadrf-g-ether.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-locations.conf \
		$(DESTDIR)$(sysconfdir)/nginx/snippets/quadrf-locations.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-kasm-proxy.conf \
		$(DESTDIR)$(sysconfdir)/nginx/snippets/quadrf-kasm-proxy.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-vnc.conf \
		$(DESTDIR)$(sysconfdir)/nginx/snippets/quadrf-vnc.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf.in $(DESTDIR)$(datadir)/network/nginx-site.in
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-locations.conf \
		$(DESTDIR)$(datadir)/network/nginx-locations.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-kasm-proxy.conf \
		$(DESTDIR)$(datadir)/network/nginx-kasm-proxy.conf
	$(INSTALL) -D -m 644 $(SRC)/network/nginx/quadrf-vnc.conf \
		$(DESTDIR)$(datadir)/network/nginx-vnc.conf
	$(INSTALL) -D -m 644 $(SRC)/network/security/index.html.in \
		$(DESTDIR)$(datadir)/network/security.in
	$(INSTALL) -D -m 644 $(SRC)/network/hostapd/quadrf.conf.in $(DESTDIR)$(datadir)/network/hostapd.conf.in
	$(INSTALL) -D -m 644 $(SRC)/network/dhcp/dhcpd.conf.in $(DESTDIR)$(datadir)/network/dhcpd.conf.in
	$(INSTALL) -D -m 644 $(SRC)/network/sudoers.in $(DESTDIR)$(datadir)/network/sudoers.in
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/network/dnsmasq $(SRC)/network/dnsmasq/*.in
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/network/interfaces.d $(SRC)/network/interfaces.d/*.cfg
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-hotspot.service $(DESTDIR)$(unitdir)/quadrf-hotspot.service
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-ethernet.service $(DESTDIR)$(unitdir)/quadrf-ethernet.service
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-usb.service $(DESTDIR)$(unitdir)/quadrf-usb.service
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-mdns.service $(DESTDIR)$(unitdir)/quadrf-mdns.service
	$(INSTALL) -D -m 644 $(SRC)/network/systemd/hostapd.service.d/quadrf.conf \
		$(DESTDIR)$(unitdir)/hostapd.service.d/quadrf.conf
	$(INSTALL) -D -m 755 $(SRC)/network/40-network $(DESTDIR)$(libdir)/apply.d/40-network
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(mandir)/man8 $(SRC)/man/quadrf-hotspot.8 \
		$(SRC)/man/quadrf-ethernet.8 $(SRC)/man/quadrf-usb.8 \
		$(SRC)/man/quadrf-dns-update.8 \
		$(SRC)/man/quadrf-apply-wifi.8 $(SRC)/man/quadrf-apply-ap.8

install-demos: demos
	$(INSTALL) -D -m 755 $(BUILD)/demos/quadrf-rf-vision $(DESTDIR)$(bindir)/quadrf-rf-vision
	$(INSTALL) -D -m 755 $(BUILD)/demos/quadrf-psd $(DESTDIR)$(bindir)/quadrf-psd
	$(INSTALL) -D -m 755 $(BUILD)/demos/quadrf-ntsc-demod $(DESTDIR)$(bindir)/quadrf-ntsc-demod
	$(INSTALL) -D -m 755 $(SRC)/demos/ntsc.sh $(DESTDIR)$(bindir)/quadrf-ntsc
	$(INSTALL) -D -m 644 $(SRC)/demos/index.html $(DESTDIR)$(datadir)/ar/index.html
	$(INSTALL) -D -m 644 $(SRC)/demos/manifest.json $(DESTDIR)$(datadir)/ar/manifest.json
	$(INSTALL) -D -m 644 $(SRC)/demos/icon-192.png $(DESTDIR)$(datadir)/ar/icon-192.png
	$(INSTALL) -D -m 644 $(SRC)/demos/icon-512.png $(DESTDIR)$(datadir)/ar/icon-512.png
	$(INSTALL) -D -m 755 $(SRC)/demos/45-demos $(DESTDIR)$(libdir)/apply.d/45-demos
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-rf-vision.service $(DESTDIR)$(unitdir)/quadrf-rf-vision.service
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-ntsc.service $(DESTDIR)$(unitdir)/quadrf-ntsc.service
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-psd.service $(DESTDIR)$(unitdir)/quadrf-psd.service
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(mandir)/man1 $(SRC)/man/quadrf-rf-vision.1 \
		$(SRC)/man/quadrf-psd.1 $(SRC)/man/quadrf-ntsc.1
	cp -a $(SRC)/phasegaze-demo $(DESTDIR)$(datadir)/phasegaze
	rm -rf $(DESTDIR)$(datadir)/phasegaze/.gitignore

install-desktop:
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(appdir) $(SRC)/desktop/applications/*.desktop
	$(INSTALL) -D -m 755 $(SRC)/desktop/kasm-xstartup.sh $(DESTDIR)$(datadir)/desktop/xstartup
	$(INSTALL) -D -m 755 $(SRC)/desktop/kasm-wrangle.sh $(DESTDIR)$(libdir)/kasm-wrangle
	$(INSTALL) -D -m 644 $(SRC)/kasmvnc/kasmvnc_defaults.yaml $(DESTDIR)$(datadir)/desktop/kasmvnc.yaml
	$(INSTALL) -D -m 755 $(SRC)/desktop/openbox-autostart $(DESTDIR)$(datadir)/desktop/config/openbox/autostart
	$(INSTALL) -D -m 644 $(SRC)/desktop/tint2rc $(DESTDIR)$(datadir)/desktop/config/tint2/tint2rc
	$(INSTALL) -D -m 644 $(SRC)/desktop/pcmanfm-LXDE.conf \
		$(DESTDIR)$(datadir)/desktop/config/pcmanfm/LXDE/pcmanfm.conf
	$(INSTALL) -D -m 644 $(SRC)/desktop/libfm.conf $(DESTDIR)$(datadir)/desktop/config/libfm/libfm.conf
	$(INSTALL) -D -m 644 $(SRC)/desktop/qradiolink.cfg \
		$(DESTDIR)$(datadir)/desktop/config/qradiolink/qradiolink.cfg
	cp -a $(SRC)/kasmvnc/www $(DESTDIR)$(datadir)/desktop/www
	$(INSTALL) -D -m 755 $(SRC)/desktop/50-desktop $(DESTDIR)$(libdir)/apply.d/50-desktop
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-desktop.service $(DESTDIR)$(unitdir)/quadrf-desktop.service
	$(INSTALL) -D -m 644 $(SRC)/icons/spatial_vision.svg $(DESTDIR)$(icondir)/quadrf-spatial-vision.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/psd_switchable.svg $(DESTDIR)$(icondir)/quadrf-psd.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/video_decoder.svg $(DESTDIR)$(icondir)/quadrf-video-decoder.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/gnu_radio.svg $(DESTDIR)$(icondir)/quadrf-gnuradio.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/qradiolink.svg $(DESTDIR)$(icondir)/quadrf-qradiolink.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/terminal.svg $(DESTDIR)$(icondir)/quadrf-terminal.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/dietpi_software.svg $(DESTDIR)$(icondir)/quadrf-software.svg
	$(INSTALL) -D -m 644 $(SRC)/icons/agentic_radio.svg $(DESTDIR)$(icondir)/quadrf-agent.svg
	$(INSTALL) -D -m 644 $(SRC)/gptme/config/env.example $(DESTDIR)$(datadir)/agent/env.example

install-gnuradio:
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/grc $(SRC)/grc_projects/*

install-ups:
	$(INSTALL) -D -m 755 $(SRC)/ups_hat/ups_daemon.py $(DESTDIR)$(libdir)/ups-daemon
	$(INSTALL) -D -m 644 -t $(DESTDIR)$(datadir)/ups/images $(SRC)/ups_hat/images/*
	$(INSTALL) -D -m 644 $(SRC)/systemd/quadrf-ups.service $(DESTDIR)$(unitdir)/quadrf-ups.service
	$(INSTALL) -D -m 755 $(SRC)/ups_hat/60-ups $(DESTDIR)$(libdir)/apply.d/60-ups

clean:
	rm -rf $(BUILD)

.PHONY: all build-indep build-arch overlays soapy-modules demos \
        install install-indep install-arch clean $(INSTALL_TARGETS)
