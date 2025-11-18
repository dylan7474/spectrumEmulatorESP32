# Spectrum Emulator (ESP32 Port Readiness)

This repository is being prepared for a dedicated ESP32 build targeting the Freenove 4.0" ESP32-S3 board (FNK0103). 

## Current status
- **Platform**: Centered on ESP32 bring-up for the FNK0103 board.
- **Input/Audio**: Pending reimplementation via ESP32 GPIO/touch and I2S peripherals.
- **Video**: LCD path implemented with an Arduino GFX-powered blit routine that converts the emulator’s RGBA buffer to the panel’s RGB565 backbuffer (double-buffered when PSRAM is available).
- **Desktop fallback**: The legacy SDL renderer/audio backend has been removed so the source tree now exclusively targets the ESP32 toolchain.
- **Goal**: Standalone ESP32 firmware with LCD, I2S audio, on-board input, and storage support.

## ESP32 bring-up prerequisites
- `git` and `python3` for working with the codebase and helper scripts.
- `arduino-cli` with the Espressif ESP32 core and the TFT_eSPI library (see bootstrap below).

Run `./configure` (or `make check-env`) to validate that the Arduino toolchain pieces are present. The `environment` bootstrap
script in the repository root automates the same apt/`arduino-cli` steps that CI uses, including proxy propagation and retry
logic for core/library downloads.

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

## Code review highlights
- `init_lcd_backend()` in `z80.c` allocates PSRAM-backed framebuffers and immediately requires a board-supplied `create_board_gfx()` implementation. The current weak stub simply returns `NULL`, so the firmware will log "LCD driver unavailable" until the board layer constructs and returns the correct `Arduino_GFX` instance for the FNK0103 display bus.
- The LCD bring-up path sets `audio_available = 0` and never programs an I2S sink, so audio is effectively muted on ESP32 builds. The AY/buzzer mixer is still present in the core, but a hardware transport needs to be implemented before deployment.
- `keyboard_matrix` remains initialized to a fixed "all keys released" state, and there is no GPIO/touch scanning logic tied into the emulator loop. Without wiring real inputs into `keyboard_matrix`, the firmware will boot into the Spectrum BASIC prompt without any way for users to interact.
- Host-style file I/O (e.g., `fopen`, directory walking, WAV/tape read/write routines) is still baked into `z80.c`. Porting these helpers to ESP-IDF storage APIs (flash partitions or SD) is required before `.tap`, `.tzx`, `.z80`, and `.sna` handling will work on-device.
- The repository currently lacks an ESP-IDF project skeleton. `Makefile` targets still focus on Arduino CLI setup, but there is no `idf.py` workflow, partition map, or sdkconfig defaults for the FNK0103 board.

## Next steps to complete ESP32 deployment
1. **Board-specific LCD wiring**
   - Implement `create_board_gfx()` (ideally under a new `boards/fnk0103/` component) so that `init_lcd_backend()` receives a fully configured `Arduino_GFX` transport for the RGB panel or SPI bridge that ships with the Freenove kit. Confirm that PSRAM-backed double buffering succeeds on-device and fall back to a single buffer otherwise.
2. **Audio transport**
   - Add an I2S driver (DAC or external codec) and hook the existing AY/buzzer mixer output into a ring buffer so that `audio_available` can be re-enabled on ESP32 builds.
3. **Input scanning**
   - Poll the physical buttons or touch pads, translate them into Spectrum keyboard rows/columns, and populate `keyboard_matrix` each frame so BASIC, tape menus, and in-game controls respond.
4. **Storage adapters**
   - Wrap `fopen`/`fread`/`fwrite`/`stat` usage with ESP-IDF-compatible file APIs, define where ROMs and user media live on the device, and document the resulting directory layout.
5. **ESP-IDF project scaffolding**
   - Introduce an ESP-IDF project (CMakeLists, `sdkconfig.defaults`, partition table, flashing instructions) so that contributors can `idf.py build/flash/monitor` the firmware instead of relying on the placeholder Arduino CLI target.
6. **Runtime UX polish**
   - Revisit the tape manager overlays and Tab-invoked controls once the LCD/input pipeline exists to ensure the embedded UI is still readable and operable on the 4.0" panel.

## LCD video backend
 - The emulator now drives the FNK0103 LCD panel through Arduino GFX. Implement `create_board_gfx()` in your board layer to return an initialized `Arduino_GFX*` for the panel bus you are using (RGB panel helper or SPI/QSPI bridge).
- Frame data is assembled in a 352×288 RGBA buffer (Spectrum frame plus borders) and converted to RGB565 before it is flushed to the display.
- Two PSRAM framebuffers are allocated when possible for tear-free double buffering; if PSRAM is constrained, the code will automatically fall back to a single surface while retaining the same conversion path.
- The tape overlay and manager continue to render into the shared RGBA buffer before each flush so that desktop and ESP32 builds remain visually aligned.

## ESP32 port roadmap
The following tasks outline the remaining work to deliver a usable ESP32 build. Each item should be kept in sync with implementation progress and any architectural changes in the emulator core.

1. **Implement ESP32 LCD video backend (done in core; verify on hardware)**
   - SDL rendering has been replaced by an Arduino GFX-driven LCD path that converts the RGBA frame buffer to RGB565 before flushing.
   - Default path allocates two PSRAM framebuffers for tear-free swaps, falling back to a single surface when memory is tight.
   - Boards must supply `create_board_gfx()` to wire up the appropriate Arduino GFX panel instance; validate timing/tearing on real hardware.
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
