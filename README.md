> **Disclaimer:** This project and code was written by Claude (Claude Sonnet 4.5). All design, implementation, and documentation were AI-generated to create a functional wireless scanning and GPS logging device.

# SignalScout 📡

**A portable WiFi, Bluetooth, and Zigbee scanner with GPS tracking for ESP32-C5**

![Logo](./Untitled.jpeg)

SignalScout is a wardriving and wireless reconnaissance tool that scans 2.4GHz and 5GHz WiFi networks, Bluetooth Low Energy (BLE) devices, and Zigbee networks (IEEE 802.15.4), logging everything to an SD card with precise GPS coordinates and timestamps.

## Features

- 📶 Dual-band WiFi scanning (2.4GHz & 5GHz)
- 🔵 Bluetooth Low Energy (BLE) device discovery
- 🟢 Zigbee network scanning (IEEE 802.15.4, channels 11-26)
- 📍 GPS-tagged logging with location, altitude, and satellite data
- 📊 Real-time OLED display with stats and countdown
- 💾 CSV logging to SD card for mapping and analysis
- 🧭 Built-in compass and speed display
- 🔒 Encryption type detection
- 🔋 Battery level monitoring with on-screen indicator
- 💡 RGB LED status indicator during boot sequence
- 📁 SD card file sharing over WiFi (browse and download log files from a browser)
- 😴 Light sleep mode for battery conservation (button-controlled)

## Hardware Required

| Component | Example Model | Connection |
|-----------|--------------|------------|
| **Microcontroller** | ESP32-C5 Dev Board (16MB Flash, 8MB PSRAM) | - |
| **GPS Module** | NEO-6M or NEO-7M | UART (RX:14, TX:13) |
| **OLED Display** | SSD1309 128x64 (or SSD1306) | SPI (MOSI:26, CLK:25, DC:9, CS:8, RST:10) |
| **SD Card Module** | Micro SD SPI adapter | SPI (MOSI:3, MISO:1, CLK:0, CS:2) |
| **RGB LED** | WS2812B | Data: GPIO27 |
| **Battery** | 3.7V LiPo 3000mAh | Via charging port + voltage divider to GPIO6 |
| **Push Button** | Momentary tactile switch | GPIO7 (active LOW, internal pullup) |

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
│ 📡▂▄▆█     [██░░]85%        065° NE         │  ← GPS satellites | Battery | Compass
├──────────────────────────────────────────────┤
│ W:12(47)*              Scan in: 5s           │  ← WiFi: last(total)*
│ B:8(23)                                      │  ← BLE: last(total)
│ Z:2(5)                                       │  ← Zigbee: last(total)
├──────────────────────────────────────────────┤
│ 14:23:57                       45M 72K       │  ← GPS time (UTC) | Speed (MPH/KPH)
└──────────────────────────────────────────────┘
```

### Display Elements

- **Top Left:** Satellite icon with signal strength bars (0-5 bars based on satellite count)
- **Top Center:** Battery indicator with icon and percentage (updates every 5 seconds)
- **Top Right:** Degree heading (000-359°) and compass direction (N, NE, E, SE, S, SW, W, NW)
  - Requires movement >1 km/h to display compass bearing
- **Center Left:** Device counters showing format "W:X(XX)", "B:X(XX)", "Z:X(XX)"
  - First number = devices/networks found in last scan
  - Number in parentheses = total unique devices/networks seen since boot
  - **Asterisk (*)** appears when actively scanning (e.g., "W:5(23)*" during WiFi scan)
- **Center Right:** Countdown timer "Scan in: Xs" showing seconds until next scan cycle
- **Bottom Left:** Current UTC time from GPS in HH:MM:SS format (2px left margin, updates every second)
- **Bottom Right:** Current speed in MPH and KPH format "XXM YYK" (2px right margin, right-aligned)

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
   - One terminal → GPIO7
   - Other terminal → GND
   - No external pullup resistor needed (internal pullup is enabled in firmware)

7. **Insert a formatted SD card** (FAT32 recommended)

### 3. Upload Code

1. Open `SignalScout.ino` in Arduino IDE
2. Select board: Tools → Board → ESP32 → ESP32-C5
3. Select the correct COM port: Tools → Port
4. **Configure Zigbee settings:**
   - Tools → Zigbee Mode → **Zigbee ED (End Device)** (recommended for scanning)
   - Tools → Partition Scheme → **Zigbee 4MB with spiffs** (or appropriate for your flash size)
5. Click Upload
6. Open Serial Monitor (115200 baud) to see scan output

### 4. Field Operation

1. **Power on the device** outdoors for GPS signal
2. **Watch LED indicators:**
   - 🔴 **Red** = System initializing
   - 🟠 **Orange** = Waiting for GPS signal
   - 🟢 **Green** = Ready, about to start scanning
   - ⚫ **Off** = Scanning active (LED off to save battery)
   - 🔵 **Blue** = File sharing mode active
3. **Wait for GPS lock** - display shows "Waiting GPS" with satellite count and elapsed time (30-60 seconds)
4. **Scans run automatically** after GPS lock, every 10 seconds in staggered sequence:
   - WiFi scans for ~3 seconds (asterisk appears: `W:12(47)*`)
   - BLE scans for ~3 seconds starting at +3.5s (asterisk appears: `B:8(23)*`)
   - Zigbee scans for ~3 seconds starting at +7s (asterisk appears: `Z:2(5)*`)
5. **Display updates every second** with GPS time, battery, and device counts
6. **Start moving** to see compass direction and speed (requires >1 km/h movement)
7. **Data is logged** to `/scan_YYYYMMDD_HHMMSS.txt` on the SD card after each scan completes

### Button Controls (GPIO7)

The push button on GPIO7 is a multi-function control. Hold duration determines the action:

| Action | How |
|--------|-----|
| **Toggle file sharing** | Press and hold for 1 second, then **release** (before 3s) |
| **Enter light sleep** | Press and hold for 3 seconds (no release needed) |
| **Wake from sleep** | Hold button for 1 second while device is sleeping |

> Releasing the button before 1 second does nothing. If you hold past 3 seconds the device enters sleep regardless — it will exit file sharing mode first if that was active.

## File Sharing Mode

File sharing mode gives you wireless access to all log files on the SD card without removing it from the device.

### How It Works

1. **Enter:** Hold the button for 1 second and release. All scanning tasks pause.
2. The device connects to your WiFi network using credentials from `secrets.h` (10-second timeout).
3. The OLED displays the assigned IP address and the LED turns **blue**.
4. Open `http://<IP>` in any browser on the same network — you'll see a file listing page with clickable download links for every file on the SD card root.
5. **Exit:** Hold the button for 1 second and release again. The web server stops, WiFi disconnects, and scanning resumes automatically.

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

## Log File Format

Each log file begins with a comprehensive header documenting all column formats and field descriptions. Each device/network is then logged on one line with complete GPS data in CSV format.

### Column Headers

**WiFi Format:**
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption
```

**BLE Format:**
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,Name,Address,RSSI,ManufacturerData,ServiceUUID
```

**Zigbee Format:**
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,PAN_ID,ExtendedPAN_ID,Channel,PermitJoin,RouterCapacity,EndDeviceCapacity
```

### Field Descriptions

**Common Fields:**
- **Fingerprint**: 8-character hex ID derived from device MAC/PAN ID (stable, unique identifier)
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

**Zigbee Fields:**
- **PAN_ID**: 16-bit network ID (e.g., `0x1A2B`)
- **ExtendedPAN_ID**: 64-bit unique network ID (e.g., `00:11:22:33:44:55:66:77`)
- **Channel**: Zigbee channel 11-26 (all 2.4GHz)
- **PermitJoin**: Network accepting joins (`Yes`/`No`)
- **RouterCapacity**: Can accept routers (`Yes`/`No`)
- **EndDeviceCapacity**: Can accept end devices (`Yes`/`No`)

### Example Entries

**WiFi (Home router):**
```
WIFI,3C7B6E95,2026-01-24 22:57:04,41.342822,-81.389317,327.20,8,1.34,MyHomeNetwork,60:B7:6E:6D:99:95,-45,6,2.4GHz,WPA2-PSK
```

**BLE (Smart watch):**
```
BLE,FA2FAF58,2026-01-24 22:57:16,41.342820,-81.389308,327.90,8,1.34,Smart Watch,58:D9:FA:AF:2F:FD,-65,4C001005,0000180A
```

**Zigbee (Smart home hub):**
```
ZIGBEE,8A3F5C12,2026-01-24 22:57:28,41.342818,-81.389299,328.10,8,1.34,0x1A2B,00:11:22:33:44:55:66:77,15,Yes,Yes,No
```

**Notes:**
- All strings sanitized to printable ASCII (no garbage characters)
- BLE manufacturer data shown as clean hex
- CSV format - easily imported to Excel, Python, GIS tools
- Same device always gets same fingerprint across scans

## Configuration

WiFi credentials for file sharing are stored separately in `secrets.h` (see [File Sharing Mode](#file-sharing-mode) above). All other key settings can be adjusted at the top of `SignalScout.ino`:

```cpp
// Output control
#define ENABLE_CONSOLE_OUTPUT true   // Serial console output
#define ENABLE_DISPLAY_OUTPUT true   // OLED display
#define ENABLE_LOG_OUTPUT true       // SD card logging
#define ENABLE_ZIGBEE_SCAN true      // Zigbee network scanning

// Timing (staggered scanning for smooth display updates)
#define SCAN_INTERVAL 10           // Scan cycle repeats every 10 seconds
#define BLE_SCAN_TIME 3            // BLE scan duration: 3 seconds
#define ZIGBEE_SCAN_DURATION 5     // Zigbee scan duration: ~3 seconds
// WiFi scans for ~3 seconds
// Scans run sequentially: WiFi (0s) → BLE (3.5s) → Zigbee (7s) → repeat

// Battery voltage divider
#define VOLTAGE_DIVIDER_RATIO 3.333333  // For 200k/100k divider
```

### Zigbee Mode Selection

In Arduino IDE, configure Zigbee via Tools menu:
- **Zigbee ED (End Device)**: Recommended for scanning - passive, low power
- **Zigbee ZCZR (Coordinator/Router)**: Alternative - can also route traffic

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **"SD Card initialization failed"** | Check wiring, ensure SD card is FAT32 formatted, verify CS pin is correct |
| **"Waiting GPS" forever** | GPS needs clear sky view, move outdoors, wait up to 2 minutes for cold start |
| **Display shows nothing** | Verify OLED wiring, check if SSD1309/SSD1306 is set correctly in code |
| **No WiFi networks found** | Normal in remote areas, verify ESP32 WiFi is working |
| **Compass shows "---"** | GPS course requires movement >1 km/h, start walking/driving |
| **LED stays red** | Initialization stuck, check serial monitor for errors |
| **LED stays orange** | GPS not getting signal, move to open sky area |
| **Battery shows 0% or wrong** | Check voltage divider wiring, adjust `VOLTAGE_DIVIDER_RATIO` in code |
| **Zigbee init failed** | Check Arduino IDE: Zigbee Mode and Partition Scheme must be configured |
| **No Zigbee networks found** | Normal if no Zigbee/Thread devices nearby; they're less common than WiFi |
| **Z:0(0) always** | Ensure `ENABLE_ZIGBEE_SCAN true` and correct IDE settings |
| **File sharing WiFi fails** | Verify SSID and password in `secrets.h`, ensure device is within range of your network |
| **File sharing not starting** | Check button wiring (GPIO7 to GND), hold for a full second before releasing |
| **No files listed in browser** | SD card must be mounted and contain files in the root directory |
| **Device won't wake from sleep** | Hold the button for a full second; ensure GPIO7 wiring is secure |

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


![ScoutCat](signalscoutcat.gif)
