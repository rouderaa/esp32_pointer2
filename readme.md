# TTGO T7 v1.3 (ESP32) — PlatformIO Project

PlatformIO configuration for the **LilyGO TTGO T7 v1.3** (Mini32 / ESP32-WROVER-B), targeting Arduino framework projects that use LiDAR distance sensing (TFMini Plus) and Bluetooth A2DP audio.

## Hardware

- **Board:** LilyGO TTGO T7 v1.3 (Mini32)
- **Module:** ESP32-WROVER-B
- **PSRAM:** 4 MB
- **Flash:** 4 MB (default partition scheme, OTA-capable)

## Environment

| Setting | Value |
|---|---|
| Environment name | `ttgo-t7-v13` |
| Platform | `espressif32@6.3.2` |
| Board | `esp32dev` |
| Framework | `arduino` |

## Serial / Upload Settings

| Setting | Value |
|---|---|
| Monitor speed | `115200` |
| Monitor port | `/dev/ttyUSB2` |
| Upload speed | `921600` |
| Upload port | `/dev/ttyUSB2` |

> Adjust `monitor_port` / `upload_port` to match your system (e.g. `/dev/ttyUSB0`, `COM5`, etc.).

## Build Flags

- `-DBOARD_HAS_PSRAM` — enables the 4 MB PSRAM on the WROVER-B module
- `-mfix-esp32-psram-cache-issue` — required compiler workaround for ESP32 PSRAM cache bug
- `-DARDUINO_ESP32_DEV` — identifies the board variant to the Arduino core
- `-DARDUINO_USB_CDC_ON_BOOT=0` — disables USB CDC on boot to avoid an undefined-reference error on `Serial1`

## Partition Scheme

Uses the `default.csv` partition table (4 MB flash) with OTA update support.

## Optional Settings

The following are commented out in `platformio.ini` and can be enabled if needed (e.g. for PSRAM-heavy tasks or custom flash configurations):

```ini
; board_build.f_cpu = 240000000L
; board_build.f_flash = 80000000L
; board_build.flash_mode = dio
```

## Library Dependencies

| Library | Purpose |
|---|---|
| [`budryerson/TFMPlus`](https://github.com/budryerson/TFMPlus) | Driver for the TFMini Plus LiDAR distance sensor |
| [`ESP32-A2DP`](https://github.com/pschatzmann/ESP32-A2DP) | Bluetooth A2DP audio source/sink for ESP32 |
| [`arduino-audio-tools`](https://github.com/pschatzmann/arduino-audio-tools) | Audio processing utilities for Arduino/ESP32 |

## Getting Started

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. Clone this repository and open it in PlatformIO.
3. Connect the TTGO T7 v1.3 board via USB and confirm the serial port matches `upload_port` / `monitor_port`.
4. Build and upload:
   ```bash
   pio run -e ttgo-t7-v13 -t upload
   ```
5. Open the serial monitor:
   ```bash
   pio device monitor -e ttgo-t7-v13
   ```

## Notes

- If you encounter an undefined reference to `Serial1`, confirm `ARDUINO_USB_CDC_ON_BOOT=0` is set (already included above).
- PSRAM must remain enabled (`BOARD_HAS_PSRAM`) for any PSRAM-heavy audio or buffering tasks.
