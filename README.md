# Spectrum Emulator (ESP32 Port Readiness)

This repository currently contains a ZX Spectrum emulator implementation (`z80.c`) that targets Intel Linux using SDL for video output. The next phase is to port the emulator to the Freenove 4.0" ESP32-S3 board (FNK0103), replacing the desktop SDL dependencies with ESP-IDF peripherals and board-specific drivers.

## Current status
- **Platform**: Intel Linux with SDL video path.
- **Input/Audio**: Desktop keyboard handling and audio mixer tied to SDL.
- **Goal**: Standalone ESP32 build with LCD, I2S audio, on-board input, and storage support.

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
