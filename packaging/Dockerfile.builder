# Cached arm64 builder for packaging/build-in-container.sh.
# Toolchain and Build-Depends only — the live tree is mounted at /src on each run.
#
# Build context is a tiny staging dir (Dockerfile + debian/control), not the
# full repo, so packaging/out and other large trees never enter the image.
FROM debian:trixie

ENV DEBIAN_FRONTEND=noninteractive

# Packaging helpers plus the OpenOCD third-party build tools. Product
# Build-Depends from debian/control are installed in the next layer so a
# control change invalidates only that step.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        debhelper \
        devscripts \
        equivs \
        fakeroot \
        git \
        gnupg \
        reprepro \
        rsync \
        cmake \
        device-tree-compiler \
        pkg-config \
        autoconf \
        automake \
        libgpiod-dev \
        libjaylink-dev \
        libtool \
        libusb-1.0-0-dev \
        texinfo \
 && rm -rf /var/lib/apt/lists/*

COPY debian/control /tmp/debian/control
RUN apt-get update \
 && mk-build-deps -i -r -t "apt-get -y --no-install-recommends" /tmp/debian/control \
 && rm -rf /var/lib/apt/lists/* /tmp/debian /tmp/quadrf-build-deps_*

# qradiolink is not in Debian; thirdparty/qradiolink rebuilds upstream.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        gnuradio-dev \
        gr-osmosdr \
        libasound2-dev \
        libcodec2-dev \
        libconfig++-dev \
        libftdi-dev \
        libopus-dev \
        libprotobuf-dev \
        libpulse-dev \
        libsndfile1-dev \
        libspeexdsp-dev \
        libvolk-dev \
        protobuf-compiler \
        qt5-qmake \
        qtbase5-dev \
        qtmultimedia5-dev \
        liblimesuite-dev \
        libzmq3-dev \
        cppzmq-dev \
        libsoapysdr-dev \
        liblog4cpp5-dev \
        libuhd-dev \
 && rm -rf /var/lib/apt/lists/*
