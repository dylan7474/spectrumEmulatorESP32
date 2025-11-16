# Spectrum Emulator (ESP32 Port Readiness)

This repository is being prepared for a dedicated ESP32 build targeting the Freenove 4.0" ESP32-S3 board (FNK0103). 

## Current status
- **Platform**: SDL desktop build removed; work is centered on ESP32 bring-up for the FNK0103 board.
- **Input/Audio**: Pending reimplementation via ESP32 GPIO/touch and I2S peripherals.
- **Goal**: Standalone ESP32 firmware with LCD, I2S audio, on-board input, and storage support.

## ESP32 bring-up prerequisites
- `git` and `python3` for working with the codebase and helper scripts.
- `arduino-cli` with the Espressif ESP32 core and the TFT_eSPI library (see bootstrap below).

Run `./configure` (or `make check-env`) to validate that the Arduino toolchain pieces are present. The script no longer checks for SDL desktop dependencies because the host build path has been retired.

### Environment bootstrap for Arduino/ESP32 tooling
The following steps mirror the CI/bootstrap script used for ESP32 bring-up. Run them on Debian/Ubuntu systems to prepare `arduino-cli` with the ESP32 core and TFT driver library:

```bash
#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

# Proxy passthrough if required
export HTTP_PROXY="${HTTP_PROXY:-${http_proxy:-}}"
export HTTPS_PROXY="${HTTPS_PROXY:-${https_proxy:-$HTTP_PROXY}}"
export NO_PROXY="${NO_PROXY:-${no_proxy:-localhost,127.0.0.1,::1}}"
export http_proxy="$HTTP_PROXY"
export https_proxy="$HTTPS_PROXY"
export no_proxy="$NO_PROXY"

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
arduino-cli config init --overwrite
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli config set network.proxy "${HTTPS_PROXY:-$HTTP_PROXY}" || true
arduino-cli config set network.socket_timeout 60s || true
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "TFT_eSPI"
```

After the toolchain is installed, run `make arduino-prepare` to refresh indexes and libraries, or `make arduino-build` with the appropriate `ARDUINO_FQBN` when the ESP32 firmware scaffold is in place.

## ESP32 port roadmap
The following tasks outline the remaining work to deliver a usable ESP32 build. Each item should be kept in sync with implementation progress and any architectural changes in the emulator core.

1. **Implement ESP32 LCD video backend**
   - Replace the SDL video pipeline with the board’s LCD driver and frame blitting routine (ESP-IDF with Arduino GFX/LVGL is recommended).
   - Efficiently render the 256×192 frame plus borders using a double-buffered PSRAM framebuffer or a line-by-line flush strategy.
   - Add RGB565/RGB888 conversions as needed for the panel format.
2. **Add ESP32 I2S audio output**
   - Rebuild audio output using the ESP32 I2S peripheral (on-chip DAC or external codec).
   - Port the existing beeper/AY mixer to feed a continuous I2S stream at an achievable sample rate.
3. **Implement GPIO/touch input mapping**
   - Map the board’s buttons and/or touch panel to the Spectrum keyboard matrix.
   - Ensure input scanning integrates cleanly with the emulator’s timing loop.
4. **Port file I/O to ESP-IDF storage**
   - Adapt ROM/tape/snapshot handling to use on-board flash or SD storage for `.z80`, `.sna`, `.tap`, `.tzx`, and `.wav` assets.
   - Define a predictable directory layout for content and configuration.
5. **Set up ESP-IDF project and build system**
   - Create an ESP-IDF project structure (CMake, component definitions, sdkconfig defaults) targeting the Freenove FNK0103 board.
   - Add build instructions for flashing and monitoring via `idf.py`.
6. **Optimize for ESP32 performance**
   - Tune CPU usage, buffering strategy, and memory placement (e.g., PSRAM) to sustain 3.5 MHz Spectrum emulation with stable frame pacing.
   - Profile and adjust timing loops to respect gate-array contention expectations.
7. **Design embedded UI workflow**
   - Provide device-friendly overlays or menus for tape loading, reset, and display settings.
   - Ensure usability with the available physical controls and screen size.
8. **Test and validate on hardware**
   - Establish smoke and regression checks on the physical board, including display refresh, audio fidelity, input latency, and storage throughput.
   - Document validation steps and known limitations.

## Contribution guidelines
- Keep this README and accompanying ESP32 documentation updated whenever the roadmap, build steps, or user-facing workflows change.
- Align documentation with the actual behavior of the emulator (video timings, tape manager flow, snapshot handling, and exit/fullscreen affordances) as the ESP32 port evolves.
