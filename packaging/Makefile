# Build the QuadRF packages and fill the apt repository.
#
#   make quadrf        packages from this source tree
#   make thirdparty    quadrf-openocd, qradiolink, mirrored KasmVNC
#   make repo          publish everything in out/ into repo/
#   make serve         HTTP-serve the signed tree (maintainer testing)
#
# Build on the target architecture (a Pi 5, an arm64 builder or an arm64
# container). Cross builds are not set up.

TOP    := $(CURDIR)/..
OUT    := $(CURDIR)/out
SUITE  ?= trixie
PORT   ?= 8080

all: quadrf thirdparty

$(OUT):
	mkdir -p $(OUT)

quadrf: | $(OUT)
	@for pkg in $$(awk '/^Package: /{print $$2}' $(TOP)/debian/control); do \
		rm -f $(OUT)/$${pkg}_*.deb; \
	done
	cd $(TOP) && dpkg-buildpackage -b -uc -us
	mv $(TOP)/../quadrf*_$(shell dpkg-parsechangelog -l$(TOP)/debian/changelog -SVersion)_*.deb $(OUT)/
	mv $(TOP)/../quadrf_$(shell dpkg-parsechangelog -l$(TOP)/debian/changelog -SVersion)_*.buildinfo $(OUT)/ 2>/dev/null || true
	mv $(TOP)/../quadrf_$(shell dpkg-parsechangelog -l$(TOP)/debian/changelog -SVersion)_*.changes $(OUT)/ 2>/dev/null || true

thirdparty: openocd qradiolink kasmvnc

openocd: | $(OUT)
	OUT=$(OUT) ./thirdparty/openocd/build.sh

qradiolink: | $(OUT)
	OUT=$(OUT) ./thirdparty/qradiolink/build.sh

kasmvnc: | $(OUT)
	OUT=$(OUT) ./thirdparty/fetch-kasmvnc.sh

key:
	./repo/gen-key.sh
	./repo/export-key.sh

repo:
	SUITE=$(SUITE) OUT=$(OUT) ./repo/publish.sh

serve: repo
	QUADRF_REPO_PORT=$(PORT) ./repo/serve.sh

clean:
	rm -rf $(OUT) thirdparty/openocd/work thirdparty/qradiolink/work

.PHONY: all quadrf thirdparty openocd qradiolink kasmvnc key repo serve clean
