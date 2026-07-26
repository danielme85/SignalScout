> **Disclaimer:** This project and code was written by Claude (Claude Sonnet 4.5/4.6). All design, implementation, and documentation were AI-generated to create a functional wireless scanning and GPS logging device.

# SignalScout 📡

**A portable WiFi and Bluetooth scanner with GPS tracking for ESP32-C5**

<p align="center">
  <img src="./Untitled.jpeg" alt="SignalScout Logo">
</p>

SignalScout is a wardriving and wireless reconnaissance tool that scans 2.4GHz and 5GHz WiFi networks and Bluetooth Low Energy (BLE) devices, logging everything to an SD card with precise GPS coordinates and timestamps.

> 🙏 PCBs for this project sponsored by [PCBWay](https://www.pcbway.com).

## Features

- 📶 Dual-band WiFi scanning (2.4GHz & 5GHz)
- 🔵 Bluetooth Low Energy (BLE) device discovery
- 📍 GPS-tagged logging with location, altitude, and satellite data
- 📊 Real-time OLED display with stats and scanning indicators
- 💾 CSV logging to SD card for mapping and analysis
- 🧭 Built-in compass and speed display
- 🔒 Encryption type detection
- 🔋 Battery level monitoring with on-screen indicator
- 💡 RGB LED status indicator during boot sequence
- 📁 SD card file sharing over WiFi (browse and download log files from a browser)
- 🚔 Flock Safety / SoundThinking device detection with visual alert
- 🔁 Self-healing WiFi driver with proactive periodic resets for long-session stability

## Hardware Required

| Component | Example Model | Connection |
|-----------|--------------|------------|
| **Microcontroller** | ESP32-C5 Dev Board (16MB Flash, 8MB PSRAM) **or** Seeed XIAO ESP32-C5 (8MB Flash, 8MB PSRAM) | - |
| **GPS Module** | NEO-6M or NEO-7M | UART (RX:14, TX:13) |
| **OLED Display** | SSD1309 128x64 (or SSD1306) | SPI (MOSI:26, CLK:25, DC:9, CS:8, RST:10) |
| **SD Card Module** | Micro SD SPI adapter | SPI (MOSI:3, MISO:1, CLK:0, CS:2) |
| **RGB LED** | WS2812B | Data: GPIO27 |
| **Battery** | 3.7V LiPo 3000mAh | Via charging port + voltage divider to GPIO6 |
| **Push Button** | Momentary tactile switch | GPIO23 (active LOW, internal pullup) |

> **Note:** MOSI and CLK pins can be shared between SD card and OLED display since both use SPI, but each needs a unique CS (Chip Select) pin.

### Battery Voltage Divider

To monitor battery voltage, connect a voltage divider between the battery and GPIO6:
```
Battery (+) ──┬── 200kΩ ──┬── 100kΩ ──┬── GND
              │           │           │
              │         GPIO6         │
              │        (ADC)          │
```
This divides the voltage by 3 (200k resistor to 100k resistor), allowing the 3.3V ADC to safely measure up to ~10V.
For different resistor values, adjust `VOLTAGE_DIVIDER_RATIO` in the code.

## Display Layout

The OLED display provides real-time visual feedback, updating every 1 second:

```
┌──────────────────────────────────────────────┐
│ 📡▂▄▆█     [██░░]85%        NE              │  ← GPS satellites | Battery | Compass
├──────────────────────────────────────────────┤
│ W:12(47)*              (WiFi logo)           │  ← WiFi: last(total)*
│ B:8(23)                                      │  ← BLE: last(total)
│ FLOCK:2 ◄ inverted alert                     │  ← Flock device count (if any detected)
├──────────────────────────────────────────────┤
│ 14:23:57                       45MPH 72KPH   │  ← GPS time (UTC) | Speed (MPH/KPH)
└──────────────────────────────────────────────┘
```

### Display Elements

- **Top Left:** Satellite icon with signal strength bars (0-5 bars based on satellite count)
- **Top Center:** Battery indicator with icon and percentage (updates every 5 seconds)
- **Top Right:** Compass direction (N, NE, E, SE, S, SW, W, NW)
  - Requires movement >1 km/h to display compass bearing
- **Center Left:** Device counters showing format "W:X(XX)", "B:X(XX)"
  - First number = devices found in last scan
  - Number in parentheses = total unique devices seen since boot
  - **Asterisk (*)** appears when actively scanning (e.g., "W:5(23)*" during WiFi scan)
  - **`W:--` / `B:--`** shown when WiFi or BLE scanning is disabled
- **Center Right:** Animated WiFi logo
- **Flock Row:** Shows one of three states:
  - `FLOCK:N` inverted (black-on-white) — one or more Flock devices detected this session
  - `[FLOCK ONLY]` — Flock-only mode is active but no devices found yet
  - *(blank)* — normal mode, no Flock devices seen
  - A full-screen blinking "GET FLOCKED!" alert appears for 3 seconds on first detection of each new device.
- **Bottom Left:** Current UTC time from GPS in HH:MM:SS format (2px left margin, updates every second)
- **Bottom Right:** Current speed in MPH and KPH format (2px right margin, right-aligned)

## Pre-built Firmware

Every tagged release on GitHub includes ready-to-flash firmware built automatically by GitHub Actions for **two targets**. No Arduino IDE required to flash.

### Files in each release

| File | Target | Use |
|------|--------|-----|
| `SignalScout-esp32c5-16mb-merged.bin` | ESP32-C5 dev board (16MB flash) | **Recommended** — single file, flash at `0x0` |
| `SignalScout-xiao-esp32c5-8mb-merged.bin` | Seeed XIAO ESP32-C5 (8MB flash) | **Recommended for XIAO** — flash at `0x0` |
| `SignalScout.ino.bin` | (both) | App only — flash at `0x10000` |
| `SignalScout.ino.bootloader.bin` | (both) | Bootloader — flash at `0x0` |
| `SignalScout.ino.partitions.bin` | (both) | Partition table — flash at `0x8000` |

### Flashing with esptool.py

```bash
# Easiest — merged binary, single command (pick the right file for your board)
esptool.py --chip esp32c5 --port /dev/ttyUSB0 write_flash 0x0 SignalScout-esp32c5-16mb-merged.bin
# or for XIAO:
esptool.py --chip esp32c5 --port /dev/ttyUSB0 write_flash 0x0 SignalScout-xiao-esp32c5-8mb-merged.bin
```

### Releasing a new version

Push a tag and GitHub Actions builds and publishes automatically:

```bash
git tag v1.2.0
git push origin v1.2.0
```

The workflow installs arduino-cli, the ESP32 core, and all required libraries, then compiles and attaches the binaries to the release. `secrets.h` is not included in any release binary — you must supply your own WiFi credentials by flashing a build from source, or the file-sharing mode simply won't connect (all other features work without it).

## Quick Start Guide

### 1. Install Arduino IDE & Libraries

**Install ESP32 Board Support:**
- Open Arduino IDE
- Go to File → Preferences
- Add to "Additional Board Manager URLs":
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
- Go to Tools → Board → Boards Manager
- Search for "esp32" and install "esp32 by Espressif Systems"

**Install Required Libraries:**

Go to Tools → Manage Libraries and install:
- `TinyGPSPlus` by Mikal Hart
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit NeoPixel`
- `ESP32Time`
- `ESPAsyncWebServer`
- `AsyncTCP`

### 2. Hardware Assembly

1. **Connect GPS Module to ESP32:**
   - GPS TX → ESP32 RX (GPIO14)
   - GPS RX → ESP32 TX (GPIO13)
   - GPS VCC → 3.3V or 5V
   - GPS GND → GND

2. **Connect OLED Display (SPI):**
   - MOSI → GPIO26
   - CLK/SCK → GPIO25
   - DC → GPIO9
   - CS → GPIO8
   - RESET → GPIO10
   - VCC → 3.3V
   - GND → GND

3. **Connect SD Card Module (SPI):**
   - MOSI → GPIO3
   - MISO → GPIO1
   - CLK/SCK → GPIO0
   - CS → GPIO2
   - VCC → 5V (or 3.3V depending on module)
   - GND → GND

4. **Connect RGB LED (WS2812B):**
   - Data → GPIO27
   - VCC → 5V
   - GND → GND

5. **Connect Battery Monitor (voltage divider):**
   - Battery (+) → 200kΩ resistor → GPIO6 → 100kΩ resistor → GND
   - See voltage divider diagram above for proper ratio

6. **Connect Push Button:**
   - One terminal → GPIO23
   - Other terminal → GND
   - No external pullup resistor needed (internal pullup is enabled in firmware)

7. **Insert a formatted SD card** (FAT32 recommended)

### 3. Upload Code

1. Open `SignalScout.ino` in Arduino IDE
2. Select board: Tools → Board → ESP32 → ESP32-C5
3. Select the correct COM port: Tools → Port
4. Click Upload
5. Open Serial Monitor (115200 baud) to see scan output

### 4. Field Operation

1. **Power on the device** outdoors for GPS signal
2. **Watch LED indicators:**
   - 🔴 **Red** = System initializing
   - 🟠 **Orange** = Waiting for GPS signal
   - 🟢 **Green** = Ready, about to start scanning
   - ⚫ **Off** = Scanning active (LED off to save battery)
3. **Wait for GPS lock** - display shows "Waiting GPS" with satellite count and elapsed time (30-60 seconds)
4. **Scans run automatically** after GPS lock, every 10 seconds:
   - WiFi scans for ~3 seconds (asterisk appears: `W:12(47)*`)
   - BLE scans for ~3 seconds after WiFi completes (asterisk appears: `B:8(23)*`)
5. **Display updates every second** with GPS time, battery, and device counts
6. **Start moving** to see compass direction and speed (requires >1 km/h movement)
7. **Data is logged** to `/scan_YYYYMMDD_HHMMSS.txt` on the SD card after each scan completes

## Button Controls (GPIO23)

The push button on GPIO23 serves different functions depending on **when** you press it:

### At Boot (2-second window)

| Action | How |
|--------|-----|
| **Enter file sharing mode** | Hold the button while powering on and keep holding through the 2-second countdown shown on the display |
| **Normal scan mode** | Power on with button released (default) |

### During Scanning

| Action | How |
|--------|-----|
| **Pause + open settings menu** | Hold for 1 second |

### In Settings Menu

| Action | How |
|--------|-----|
| **Advance cursor** | Short tap (<1 second) |
| **Activate selected item** | Hold for 1 second |

**Settings menu items:**
- `WiFi: ON/OFF` — toggle WiFi scanning (saved to flash, survives reboot)
- `BLE: ON/OFF` — toggle BLE scanning (saved to flash, survives reboot)
- `Flock: ON/OFF` — toggle Flock-only mode; when ON, non-Flock devices are ignored entirely (not counted, not logged)
- `Resume Scan` — exit settings and resume scanning

The currently selected item is shown inverted on the display. Pausing flushes the SD card log queue before opening the menu so no data is lost.

## File Sharing Mode

File sharing mode gives you wireless access to all log files on the SD card without removing it from the device.

### How to Enter

Hold the mode button **while powering on**. The display shows a 2-second countdown — keep holding until it completes. Release once the countdown finishes and the device switches to file sharing mode.

### How It Works

1. The device connects to your WiFi network using credentials from `secrets.h`.
2. The OLED displays the assigned IP address.
3. Open `http://<IP>` in any browser on the same network — you'll see a file listing page with clickable download links for every file on the SD card.
4. **To return to scan mode:** power off the device, then boot normally (button not held).

### Setting Up `secrets.h`

Before uploading, create a `secrets.h` file in the same directory as `SignalScout.ino` and fill in your network credentials:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

const char* WIFI_SSID     = "YourNetworkSSID";
const char* WIFI_PASSWORD = "YourNetworkPassword";

#endif
```

> This file is listed in `.gitignore` so your credentials won't be committed to version control.

## Flock Safety Detection

SignalScout automatically identifies **Flock Safety** (now SoundThinking) license plate readers, Penguin external battery packs, and Raven acoustic gunshot sensors while scanning. These are fixed surveillance devices commonly found on utility poles and street furniture.

Signatures are sourced from [colonelpanichacks/flock-you](https://github.com/colonelpanichacks/flock-you), [MaxwellDPS/Flock-You-Android](https://github.com/MaxwellDPS/Flock-You-Android), and [justcallmekoko/ESP32Marauder](https://github.com/justcallmekoko/ESP32Marauder).

### What It Detects

**WiFi (via SSID prefix or hardware OUI):**
- SSID prefixes: `flock`, `fs-`, `fs_`, `falcon`, `sparrow`, `condor`, `penguin`, `pigvision`, `fs ext batt`
- **36 hardware OUIs** — 31 field-verified entries from colonelpanichacks research plus Quectel (`50:29:4D`, `86:25:19`), Telit (`00:14:2D`, `D8:C7:71`), and Flock Safety direct (`B4:1E:52`)

**BLE (via device name, service UUID, or manufacturer data):**
- Device name prefixes: `flock`, `falcon`, `raven`, `penguin` (any variant), `pigvision`, `fs ext batt`, `soundthinking`, `shotspotter`
- Raven service UUIDs (firmware 1.2.x+): `00003100` (GPS Location), `00003200` (Power Management), `00003300` (Network Status), `00003400` (Upload Statistics), `00003500` (Error/Diagnostics)
- Legacy Raven service UUIDs (firmware 1.1.x): `00001809`, `00001819`
- Xuntong manufacturer data (`C809` prefix) — identifies Flock Penguin external battery packs by ODM chip

### Flock-Only Mode

Enable **Flock-only mode** from the pause/settings menu (`Flock: ON`) to filter out all non-Flock devices. When active:
- WiFi and BLE scans still run normally
- Any device that doesn't match a Flock signature is silently ignored — not counted, not logged, not displayed
- The display shows `[FLOCK ONLY]` in the stats area until a Flock device is found
- The setting persists across reboots (saved to flash)

This mode is useful when you're specifically hunting for Flock cameras and don't want surrounding noise cluttering your log.

### Visual Alerts

- **Full-screen blinking alert:** When a new Flock device is detected for the first time, the display shows a blinking **"GET FLOCKED!"** message for 3 seconds.
- **Persistent counter:** After the alert, `FLOCK:N` is shown inverted (white background) in the device count area for the remainder of the session.

### Log Output

Flock devices are logged with a distinct type field:
- `FLOCK-WIFI` instead of `WIFI`
- `FLOCK-BLE` instead of `BLE`

All other fields (GPS coordinates, signal strength, fingerprint, etc.) are identical to standard entries.

## Log File Format

Each log file begins with a comprehensive header documenting all column formats and field descriptions. Each device is then logged on one line with complete GPS data in CSV format.

### Column Headers

**WiFi / Flock-WiFi Format:**
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption
```

**BLE / Flock-BLE Format:**
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,Name,Address,RSSI,ManufacturerData,ServiceUUID
```

### Field Descriptions

**Common Fields:**
- **Type**: `WIFI`, `BLE`, `FLOCK-WIFI`, or `FLOCK-BLE`
- **Fingerprint**: 8-character hex ID derived from device MAC address (stable, unique identifier)
- **Timestamp**: UTC time from GPS (`YYYY-MM-DD HH:MM:SS`) or RTC with `(RTC)` suffix
- **Latitude/Longitude**: GPS coordinates in decimal degrees
- **Altitude**: Elevation in meters above sea level
- **Satellites**: Number of GPS satellites visible
- **HDOP**: GPS accuracy (<2 = Good, 2-5 = Moderate, >5 = Poor)

**WiFi Fields:**
- **SSID**: Network name (`<hidden>` for hidden networks)
- **BSSID**: Access point MAC address
- **RSSI**: Signal strength in dBm (-30 = excellent, -90 = weak)
- **Channel**: WiFi channel (1-14 for 2.4GHz, 32+ for 5GHz)
- **Band**: `2.4GHz` or `5GHz`
- **Encryption**: `OPEN`, `WPA2-PSK`, `WPA3-PSK`, etc.

**BLE Fields:**
- **Name**: Device name (`Unknown` if not advertised)
- **Address**: Bluetooth MAC address
- **RSSI**: Signal strength in dBm
- **ManufacturerData**: Hex-encoded manufacturer data (if present)
- **ServiceUUID**: Advertised service UUID (if present)

### Example Entries

**WiFi (Home router):**
```
WIFI,3C7B6E95,2026-01-24 22:57:04,41.342822,-81.389317,327.20,8,1.34,MyHomeNetwork,60:B7:6E:6D:99:95,-45,6,2.4GHz,WPA2-PSK
```

**BLE (Smart watch):**
```
BLE,FA2FAF58,2026-01-24 22:57:16,41.342820,-81.389308,327.90,8,1.34,Smart Watch,58:D9:FA:AF:2F:FD,-65,4C001005,0000180A
```

**Flock Safety camera (WiFi):**
```
FLOCK-WIFI,A1B2C3D4,2026-01-24 22:57:20,41.342825,-81.389310,327.50,8,1.34,fs-cam-4821,50:29:4D:AA:BB:CC,-72,6,2.4GHz,WPA2-PSK
```

**Notes:**
- All strings sanitized to printable ASCII (no garbage characters)
- BLE manufacturer data shown as clean hex
- CSV format — easily imported to Excel, Python, GIS tools
- Same device always gets same fingerprint across scans

## Configuration

WiFi credentials for file sharing are stored separately in `secrets.h` (see [File Sharing Mode](#file-sharing-mode) above). All other key settings can be adjusted at the top of `SignalScout.ino`:

```cpp
// Output control
#define ENABLE_CONSOLE_OUTPUT false  // Serial console output (set true for debugging)
#define ENABLE_DISPLAY_OUTPUT true   // OLED display
#define ENABLE_LOG_OUTPUT true       // SD card logging

// Timing
#define SCAN_INTERVAL 10             // Scan cycle repeats every 10 seconds
#define BLE_SCAN_TIME 3              // BLE scan duration: 3 seconds
// WiFi scans for ~3 seconds
// Scans run sequentially: WiFi → BLE → repeat

// WiFi driver stability
#define WIFI_DRIVER_RESET_INTERVAL_MS (15UL * 60UL * 1000UL)  // Proactive mode reset every 15 min

// Battery voltage divider
#define VOLTAGE_DIVIDER_RATIO 3.333333  // For 200k/100k divider
```

### Runtime Settings (persisted in flash)

The pause/settings menu lets you toggle scanning modes at runtime without reflashing. These settings survive reboots:

- **WiFi scanning** — enable or disable WiFi scanning
- **BLE scanning** — enable or disable BLE scanning
- **Flock-only mode** — when enabled, ignore all non-Flock devices (nothing counted or logged)

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **"SD Card initialization failed"** | Check wiring, ensure SD card is FAT32 formatted, verify CS pin is correct |
| **"Waiting GPS" forever** | GPS needs clear sky view, move outdoors, wait up to 2 minutes for cold start |
| **Display shows nothing** | Verify OLED wiring, check if SSD1309/SSD1306 is set correctly in code |
| **No WiFi networks found** | Normal in remote areas, verify ESP32 WiFi is working |
| **WiFi stops working** | Fixed in firmware — root cause was a start→stop race in the proactive reset. Update to latest firmware. |
| **Compass shows "---"** | GPS course requires movement >1 km/h, start walking/driving |
| **LED stays red** | Initialization stuck, check serial monitor for errors |
| **LED stays orange** | GPS not getting signal, move to open sky area |
| **Battery shows 0% or wrong** | Check voltage divider wiring, adjust `VOLTAGE_DIVIDER_RATIO` in code |
| **File sharing WiFi fails** | Verify SSID and password in `secrets.h`, ensure device is within range of your network |
| **File sharing mode not entering** | Hold button during boot and keep holding through the full 2-second countdown on screen |
| **No files listed in browser** | SD card must be mounted and contain files in the root directory |
| **Settings not saving** | NVS write failure — check serial output; reflash if NVS is corrupted |

## Data Analysis

Import the CSV log file into:
- **Google Maps** using GPS coordinates
- **Wigle.net** for wardriving data submission
- **Excel/Python** for signal strength analysis
- **QGIS** for advanced mapping

## Legal Notice

This tool is intended for:
- ✅ Educational purposes
- ✅ Network security auditing with permission
- ✅ Radio frequency research
- ✅ Mapping your own networks

Always comply with local laws regarding wireless monitoring and data collection.

## License

Open source - feel free to modify and share!

---

**Built with ❤️ for wireless explorers and makers**

<p align="center">
  <img src="./signalscoutcat.gif" alt="SignalScout Cat">
</p>
