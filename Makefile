# --- Makefile for the ESP32-directed Spectrum Emulator port ---
ARDUINO_CLI ?= arduino-cli
ARDUINO_FQBN ?= esp32:esp32:esp32

all:
	@echo "Host SDL desktop build retired; focus on ESP32 Freenove FNK0103 bring-up."

check-env:
	./configure

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

.PHONY: all check-env arduino-init arduino-update-index arduino-install-core arduino-install-libs arduino-prepare arduino-build
