# ESP32 Port Task Breakdown

The desktop SDL path has been retired; all effort is focused on bringing the emulator up on the ESP32-S3 Freenove 4.0" (FNK0103) board. Use this checklist to track the port. Update statuses and notes as work progresses.

## Video: ESP32 LCD backend
- [ ] Replace SDL video setup with the board’s LCD driver (ESP-IDF + Arduino GFX or LVGL).
- [ ] Decide on framebuffer strategy: double-buffered PSRAM surface vs. per-line flush.
- [ ] Implement fast 256×192 frame + border blit, with RGB565/RGB888 conversion for the panel.
- [ ] Verify refresh timing and tearing behavior on hardware.

## Audio: I2S output
- [ ] Configure ESP32 I2S (DAC or external codec) and clocking at a sustainable sample rate.
- [ ] Adapt the beeper/AY mixer to stream continuously into the I2S buffer.
- [ ] Validate audio latency and balance versus desktop output.

## Input: GPIO/touch mapping
- [ ] Map board buttons/touch to Spectrum keyboard matrix rows/columns.
- [ ] Integrate scanning into the emulator timing loop without disrupting contention-sensitive code paths.
- [ ] Document the button-to-key mapping for users.

## Storage: ROMs, tapes, and snapshots
- [ ] Choose storage targets (flash partition vs. SD) for `.z80`, `.sna`, `.tap`, `.tzx`, and `.wav` assets.
- [ ] Implement file I/O wrappers for ESP-IDF (open/read/seek) that mirror the SDL build expectations.
- [ ] Define and document a directory structure for bundled and user-supplied media.

## Build system: ESP-IDF project
- [ ] Create an ESP-IDF project layout with CMakeLists, component registration, and sdkconfig defaults for FNK0103.
- [ ] Wire the emulator core (`z80.c`) into the build with necessary `#ifdef` guards for platform-specific code paths.
- [ ] Add flashing/monitor instructions (`idf.py build/flash/monitor`).

## Performance tuning
- [ ] Profile CPU load to sustain 3.5 MHz emulation with stable frame pacing.
- [ ] Place large buffers in PSRAM where appropriate and minimize heap fragmentation.
- [ ] Revisit gate-array contention and wait-state timing if the ESP32 scheduling model requires adjustments.

## Embedded UI
- [ ] Design on-device menus or overlays for tape loading, reset, and display options.
- [ ] Ensure UI flows are operable with available buttons/touch and fit the 4.0" display.

## Hardware validation
- [ ] Run smoke tests for display refresh, audio fidelity, input responsiveness, and storage throughput on physical hardware.
- [ ] Capture known limitations and regression steps for future firmware updates.
