# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WiFi and Bluetooth scanner for ESP32-C5 that:
- Scans WiFi networks on both 2.4GHz and 5GHz bands
- Scans Bluetooth Low Energy (BLE) devices
- GPS-timestamped logging with location data
- Real-time OLED display showing GPS status, satellite count, compass, time, and speed
- Logs all scan results to SD card via SPI
- Outputs scan data to serial monitor for debugging

Hardware requirements:
- ESP32-C5 board (16MB FLASH, 8MB PSRAM)
- SD card module connected via SPI
- NEO-6M GPS module connected via UART (TX/RX)
- SSD1309 OLED display 128x64 connected via SPI

## Development Environment

This is an Arduino sketch (.ino file) that should be developed using:
- Arduino IDE, or
- Arduino CLI, or
- PlatformIO

## File Structure

- `WifiScanner.ino` - Main Arduino sketch with WiFi/BLE scanning and SD card logging

## Architecture

The sketch is structured around five main components:

1. **WiFi Scanner** (`scanWiFi()` function):
   - Scans all available WiFi networks
   - Detects both 2.4GHz (channels 1-14) and 5GHz (channels >14) networks
   - Captures SSID, BSSID, RSSI, channel, frequency band, and encryption type
   - Tracks unique devices across scans using BSSID
   - Generates unique fingerprint for each device
   - Logs each device on one line with complete GPS data

2. **Bluetooth Scanner** (`scanBluetooth()` function):
   - Performs BLE device discovery
   - Captures device name, MAC address, RSSI, and metadata (manufacturer data, service UUIDs)
   - Tracks unique devices across scans using address
   - Generates unique fingerprint for each device
   - Logs each device on one line with complete GPS data

3. **GPS Handler** (`displayGPSInfo()` function):
   - Continuously reads GPS data from NEO-6M module via UART
   - Parses NMEA/UBLOX sentences using TinyGPSPlus library
   - Provides accurate UTC timestamps for all log entries
   - Captures location (lat/lon), altitude/elevation, satellite count, and HDOP
   - Updates `gpsTimeValid` flag when GPS achieves time lock

4. **OLED Display** (`updateDisplay()` function):
   - Updates every 500ms with real-time GPS and system status
   - **Top Left**: Satellite signal strength indicator with icon and bars (0-5 bars based on satellite count)
   - **Top Right**: Degree heading (000-359°) and compass direction (N, NE, E, SE, S, SW, W, NW)
   - **Center Left**: WiFi count "W:X(XX)" and BLE count "B:X(XX)" - shows last scan count with total unique count in parentheses
   - **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next scan
   - **Bottom Left**: GPS time in HH:MM:SS UTC format (2px left margin)
   - **Bottom Right**: Speed in both MPH and KPH (2px right margin, right-aligned)
   - All display elements use consistent 2px margins on left and right edges

5. **Device Tracking**:
   - Maintains count of unique WiFi devices seen (by BSSID)
   - Maintains count of unique BLE devices seen (by address)
   - Generates 8-character hexadecimal fingerprint for each device
   - Fingerprints used for device identification and tracking

6. **SD Card Logger** (`logDeviceToFile()` and `logToFile()` functions):
   - Each device logged on one line with complete GPS data
   - Format: `TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...`
   - WiFi format: `WIFI,FP,Time,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Ch,Band,Enc`
   - BLE format: `BLE,FP,Time,Lat,Lon,Alt,Sats,HDOP,Name,Addr,RSSI,ManufData,ServiceUUID`
   - GPS data included with every device entry for location tracking

## Configuration

Key settings defined at the top of the sketch:

- **Output Flags** (WifiScanner.ino:49-51):
  - `ENABLE_CONSOLE_OUTPUT`: Enable/disable serial console output (default: true)
  - `ENABLE_DISPLAY_OUTPUT`: Enable/disable OLED display (default: true)
  - `ENABLE_LOG_OUTPUT`: Enable/disable SD card logging (default: true)

- **SD Card SPI Pins**: CS=5, MOSI=23, MISO=19, SCK=18
  - Adjust these to match your SD card module wiring

- **GPS UART Pins**: RX=16, TX=17, Baud=9600
  - RX pin connects to GPS module TX
  - TX pin connects to GPS module RX
  - Adjust pins to match your wiring

- **OLED Display SPI Pins**: MOSI=23, CLK=18, DC=4, CS=15, RESET=2
  - MOSI and CLK can be shared with SD card module
  - CS must be different from SD card CS pin
  - Adjust pins to match your wiring

- **Display Update Interval**: 500ms

- **Scan Interval**: 10 seconds
  - WiFi and BLE scans run together at the same interval
  - Countdown timer on display shows seconds until next scan

- **BLE Scan Duration**: 5 seconds per scan

## Arduino-Specific Constraints

- The main file must have the same name as the parent directory (WifiScanner)
- Arduino sketches require `setup()` function (runs once on boot) and `loop()` function (runs repeatedly)
- When adding WiFi functionality, use the appropriate WiFi library for the target board (e.g., `WiFi.h` for ESP32/ESP8266, `WiFiNINA.h` for Arduino boards with WiFi capabilities)
- Serial communication for debugging uses `Serial.begin()` in setup and `Serial.print()`/`Serial.println()` in loop

## Building and Uploading

### Using Arduino IDE
1. Install ESP32 board support via Board Manager
2. Select board: Tools > Board > ESP32 > ESP32-C5
3. Select the correct serial port
4. Click Upload

### Using Arduino CLI
```bash
# Install ESP32 core (if not already installed)
arduino-cli core install esp32:esp32

# Compile the sketch
arduino-cli compile --fqbn esp32:esp32:esp32c5 WifiScanner

# Upload to board
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32c5 WifiScanner

# Monitor serial output
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Note: Replace `/dev/ttyUSB0` with your actual serial port (check with `arduino-cli board list`).

## Required Libraries

Most libraries are included in the ESP32 Arduino core:
- `WiFi.h` - WiFi scanning functionality
- `BLEDevice.h`, `BLEUtils.h`, `BLEScan.h`, `BLEAdvertisedDevice.h` - Bluetooth LE scanning
- `SD.h`, `SPI.h` - SD card access via SPI
- `HardwareSerial.h` - UART communication for GPS

External libraries required (install via Library Manager):
- `TinyGPSPlus` - GPS NMEA/UBLOX sentence parsing
- `Adafruit GFX Library` - Graphics library for OLED display
- `Adafruit SSD1306` - OLED display driver (compatible with SSD1309)

To install libraries:
```bash
# Using Arduino CLI
arduino-cli lib install "TinyGPSPlus"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SSD1306"

# Or in Arduino IDE: Tools > Manage Libraries > Search for each library name
```

## Testing

Monitor serial output at 115200 baud to see scan results in real-time.

**OLED Display:**
The display provides real-time visual feedback:
- **Top Left**: Satellite icon with signal bars showing GPS satellite count (1-5 bars)
- **Top Right**: Degree heading (000-359°) followed by compass direction (N, NE, E, SE, S, SW, W, NW) based on GPS course (requires movement >1 km/h)
- **Center Left**: Device counts showing "W:X(XX)" for WiFi and "B:X(XX)" for BLE
  - First number = devices found in last scan
  - Number in parentheses = total unique devices seen since startup
- **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next WiFi/BLE scan
- **Bottom Left**: Current UTC time from GPS (HH:MM:SS format)
- **Bottom Right**: Current speed from GPS (format: "XXM YYK" for MPH and KPH, right-aligned)

**GPS Acquisition:**
- On first boot, display shows "Waiting GPS" while acquiring satellite lock
- May take 30-60 seconds outdoors for initial fix
- Serial monitor will display "GPS time lock acquired!" when ready
- GPS information is displayed showing location, altitude, satellites, and HDOP
- Display will show satellite count in top-left corner with signal bars

**Log File Format:**

The SD card log file (`/scanner_log.txt`) contains one line per device with complete GPS data.

**WiFi Entry Format:**
```
WIFI,XXXXXXXX,YYYY-MM-DD HH:MM:SS,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption
```

**BLE Entry Format:**
```
BLE,XXXXXXXX,YYYY-MM-DD HH:MM:SS,Lat,Lon,Alt,Sats,HDOP,Name,Address,RSSI,ManufData,ServiceUUID
```

Where:
- `XXXXXXXX` = 8-character hexadecimal device fingerprint
- GPS coordinates, altitude, satellite count, and HDOP included with every device
- When GPS fix unavailable, timestamp shows milliseconds and GPS fields show "N/A"

**Example WiFi Entry:**
```
WIFI,A3F2C891,2026-01-21 15:30:45,37.774929,-122.419418,15.50,8,1.20,MyNetwork,AA:BB:CC:DD:EE:FF,-65,6,2.4GHz,WPA2-PSK
```

**Example BLE Entry:**
```
BLE,B7E4D123,2026-01-21 15:30:46,37.774930,-122.419420,15.52,8,1.20,MyDevice,12:34:56:78:9A:BC,-72,ManufData,ServiceUUID
```
