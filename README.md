# SignalScout 📡

**A portable WiFi and Bluetooth scanner with GPS tracking for ESP32-C5**

SignalScout is a wardriving and wireless reconnaissance tool that scans both 2.4GHz and 5GHz WiFi networks plus Bluetooth Low Energy (BLE) devices, logging everything to an SD card with precise GPS coordinates and timestamps.

## Features

- 📶 Dual-band WiFi scanning (2.4GHz & 5GHz)
- 🔵 Bluetooth Low Energy (BLE) device discovery
- 📍 GPS-tagged logging with location, altitude, and satellite data
- 📊 Real-time OLED display with stats and countdown
- 💾 CSV logging to SD card for mapping and analysis
- 🧭 Built-in compass and speed display
- 🔒 Encryption type detection

## Hardware Required

| Component | Example Model | Connection |
|-----------|--------------|------------|
| **Microcontroller** | ESP32-C5 Dev Board (16MB Flash, 8MB PSRAM) | - |
| **GPS Module** | NEO-6M or NEO-7M | UART (RX:16, TX:17) |
| **OLED Display** | SSD1309 128x64 (or SSD1306) | SPI (MOSI:23, CLK:18, DC:4, CS:15, RST:2) |
| **SD Card Module** | Micro SD SPI adapter | SPI (MOSI:23, MISO:19, CLK:18, CS:5) |
| **Power** | 18650 battery + holder, USB power bank | 5V input |

> **Note:** MOSI and CLK pins can be shared between SD card and OLED display since both use SPI, but each needs a unique CS (Chip Select) pin.

## Display Layout

```
┌──────────────────────────────────────────────┐
│ 📡5 ▂▄▆█  065° NE                           │  ← Satellite count + signal bars + compass
├──────────────────────────────────────────────┤
│ W:12(47)                    Scan in:         │  ← WiFi devices (last scan/total)
│ B:8(23)                           5s         │  ← BLE devices + countdown timer
├──────────────────────────────────────────────┤
│ 14:23:57                       45M 72K       │  ← GPS time (UTC) + speed (MPH/KPH)
└──────────────────────────────────────────────┘
```

### Display Elements

- **Top Left:** Satellite icon with signal strength bars (0-5 bars based on satellite count)
- **Top Right:** Heading in degrees and compass direction (N, NE, E, SE, S, SW, W, NW)
- **Center Left:** Device counters showing last scan count with total unique devices in parentheses
- **Center Right:** Countdown timer showing seconds until next scan
- **Bottom Left:** Current UTC time from GPS
- **Bottom Right:** Current speed in both MPH and KPH

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

### 2. Hardware Assembly

1. **Connect GPS Module to ESP32:**
   - GPS TX → ESP32 RX (Pin 16)
   - GPS RX → ESP32 TX (Pin 17)
   - GPS VCC → 3.3V or 5V
   - GPS GND → GND

2. **Connect OLED Display (SPI):**
   - MOSI → Pin 23
   - CLK/SCK → Pin 18
   - DC → Pin 4
   - CS → Pin 15
   - RESET → Pin 2
   - VCC → 3.3V
   - GND → GND

3. **Connect SD Card Module (SPI):**
   - MOSI → Pin 23 (shared with OLED)
   - MISO → Pin 19
   - CLK/SCK → Pin 18 (shared with OLED)
   - CS → Pin 5
   - VCC → 5V (or 3.3V depending on module)
   - GND → GND

4. **Insert a formatted SD card** (FAT32 recommended)

### 3. Upload Code

1. Open `WifiScanner.ino` in Arduino IDE
2. Select board: Tools → Board → ESP32 → ESP32-C5
3. Select the correct COM port: Tools → Port
4. Click Upload ⬆️
5. Open Serial Monitor (115200 baud) to see scan output

### 4. Field Operation

1. **Power on the device** outdoors for GPS signal
2. **Wait for GPS lock** - display shows "Waiting GPS" until satellites are acquired (30-60 seconds)
3. **Start moving** to see compass direction and speed (requires >1 km/h movement)
4. **Scans run automatically** every 10 seconds
5. **Data is logged** to `/scanner_log.txt` on the SD card

## Log File Format

Each device is logged on one line with complete GPS data:

**WiFi Entry:**
```
WIFI,A3F2C891,2026-01-21 15:30:45,37.774929,-122.419418,15.50,8,1.20,MyNetwork,AA:BB:CC:DD:EE:FF,-65,6,2.4GHz,WPA2-PSK
```

**BLE Entry:**
```
BLE,B7E4D123,2026-01-21 15:30:46,37.774930,-122.419420,15.52,8,1.20,MyDevice,12:34:56:78:9A:BC,-72,ManufData,ServiceUUID
```

**Format:** `TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...`

## Configuration

Key settings can be adjusted at the top of `WifiScanner.ino`:

```cpp
// Output control
#define ENABLE_CONSOLE_OUTPUT true   // Serial console output
#define ENABLE_DISPLAY_OUTPUT true   // OLED display
#define ENABLE_LOG_OUTPUT true       // SD card logging

// Timing
#define SCAN_INTERVAL 10000  // Scan every 10 seconds
#define BLE_SCAN_TIME 5      // BLE scan duration (seconds)
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **"SD Card initialization failed"** | Check wiring, ensure SD card is FAT32 formatted, verify CS pin is correct |
| **"Waiting GPS" forever** | GPS needs clear sky view, move outdoors, wait up to 2 minutes for cold start |
| **Display shows nothing** | Verify OLED wiring, check if SSD1309/SSD1306 is set correctly in code |
| **No WiFi networks found** | Normal in remote areas, verify ESP32 WiFi is working |
| **Compass shows "---"** | GPS course requires movement >1 km/h, start walking/driving |

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
