BUILD_DIR ?= build
INSTALL_PREFIX ?= $(CURDIR)/install
CMAKE ?= cmake
CTEST ?= ctest

.PHONY: all configure build test install clean

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR)

build: configure
	$(CMAKE) --build $(BUILD_DIR)

test: build
	$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure

install: build
	$(CMAKE) --install $(BUILD_DIR) --prefix $(INSTALL_PREFIX)

clean:
	$(CMAKE) --build $(BUILD_DIR) --target clean
