# --- Makefile for Z80 Emulator ---
CC ?= gcc
PKG_CONFIG ?= pkg-config
ARDUINO_CLI ?= arduino-cli
ARDUINO_FQBN ?= esp32:esp32:esp32

TARGET := z80
SRCS := z80.c

# Allow callers to provide custom SDL2 flags via the environment. Otherwise try
# pkg-config first, then fall back to sdl2-config if available.
ifndef SDL2_CFLAGS
SDL2_CFLAGS := $(shell if command -v $(PKG_CONFIG) >/dev/null 2>&1; then \
        $(PKG_CONFIG) --cflags sdl2 2>/dev/null; \
    fi)
endif

ifndef SDL2_LIBS
SDL2_LIBS := $(shell if command -v $(PKG_CONFIG) >/dev/null 2>&1; then \
        $(PKG_CONFIG) --libs sdl2 2>/dev/null; \
    fi)
endif

ifeq ($(strip $(SDL2_CFLAGS)$(SDL2_LIBS)),)
SDL2_CONFIG := $(shell command -v sdl2-config 2>/dev/null)
endif

ifeq ($(strip $(SDL2_CFLAGS)$(SDL2_LIBS)),)
ifneq ($(strip $(SDL2_CONFIG)),)
SDL2_CFLAGS := $(shell $(SDL2_CONFIG) --cflags)
SDL2_LIBS := $(shell $(SDL2_CONFIG) --libs)
else
$(error SDL2 development files not found. Install libsdl2-dev and ensure pkg-config or sdl2-config is in PATH.)
endif
endif

CFLAGS ?= -Wall -Wextra -O2
CFLAGS += -std=c11 $(SDL2_CFLAGS)

LDFLAGS += $(SDL2_LIBS) -lm

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)

run: $(TARGET)
	./$< 48.rom

test: $(TARGET)
	./$< --run-tests

check-env:
	./configure

clean:
	rm -f $(TARGET)

arduino-init:
	$(ARDUINO_CLI) config init --overwrite
	$(ARDUINO_CLI) config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

arduino-update-index: arduino-init
	$(ARDUINO_CLI) core update-index

arduino-install-core: arduino-update-index
	$(ARDUINO_CLI) core install esp32:esp32

arduino-install-libs: arduino-init
	$(ARDUINO_CLI) lib install "TFT_eSPI"

arduino-prepare: arduino-install-core arduino-install-libs

arduino-build:
	$(ARDUINO_CLI) compile --fqbn $(ARDUINO_FQBN) .

.PHONY: all run test clean check-env arduino-init arduino-update-index arduino-install-core arduino-install-libs arduino-prepare arduino-build
