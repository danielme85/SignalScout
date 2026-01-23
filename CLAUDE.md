# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**SignalScout** - WiFi, Bluetooth, and Zigbee scanner for ESP32-C5 that:
- Scans WiFi networks on both 2.4GHz and 5GHz bands
- Scans Bluetooth Low Energy (BLE) devices
- Scans Zigbee networks (IEEE 802.15.4) on channels 11-26
- GPS-timestamped logging with location data
- Real-time OLED display showing GPS status, satellite count, compass, time, and speed
- Logs all scan results to SD card via SPI
- Outputs scan data to serial monitor for debugging

Hardware requirements:
- ESP32-C5 board (16MB FLASH, 8MB PSRAM)
- SD card module connected via SPI
- NEO-6M GPS module connected via UART (TX/RX)
- SSD1309 OLED display 128x64 connected via SPI
- WS2812B RGB LED on GPIO27 (status indicator)
- 3.7V LiPo battery (e.g., 3000mAh) with voltage divider on GPIO6 for monitoring

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

3. **Zigbee Scanner** (`scanZigbee()` function):
   - Scans IEEE 802.15.4 networks on Zigbee channels 11-26 (2.4GHz band)
   - Uses ESP32-C5's built-in IEEE 802.15.4 radio
   - Operates in End Device (ED) mode for passive, low-power scanning
   - Captures PAN ID, Extended PAN ID, channel, and network capabilities
   - Detects permit joining status, router capacity, and end device capacity
   - Tracks unique networks across scans using Extended PAN ID
   - Generates unique fingerprint for each network
   - Logs each network on one line with complete GPS data
   - Requires Arduino IDE Zigbee mode and partition scheme configuration

4. **GPS Handler** (`displayGPSInfo()` function):
   - Continuously reads GPS data from NEO-6M module via UART
   - Parses NMEA/UBLOX sentences using TinyGPSPlus library
   - Provides accurate UTC timestamps for all log entries
   - Captures location (lat/lon), altitude/elevation, satellite count, and HDOP
   - Updates `gpsTimeValid` flag when GPS achieves time lock
   - Syncs ESP32 RTC from GPS time (updates every 60 seconds for accuracy)
   - RTC time persists across reboots, allowing immediate time availability on boot

5. **OLED Display** (`updateDisplay()` function):
   - Updates every 500ms with real-time GPS and system status
   - **Top Left**: Satellite signal strength indicator with icon and bars (0-5 bars based on satellite count)
   - **Top Center**: Battery indicator with icon showing charge level and percentage
   - **Top Right**: Degree heading (000-359°) and compass direction (N, NE, E, SE, S, SW, W, NW)
   - **Center Left**: Device counts - WiFi "W:X(XX)", BLE "B:X(XX)", Zigbee "Z:X(XX)"
   - **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next scan
   - **Bottom Left**: GPS time in HH:MM:SS UTC format (2px left margin)
   - **Bottom Right**: Speed in both MPH and KPH (2px right margin, right-aligned)
   - All display elements use consistent 2px margins on left and right edges

6. **Device Tracking**:
   - Maintains count of unique WiFi devices seen (by BSSID)
   - Maintains count of unique BLE devices seen (by address)
   - Maintains count of unique Zigbee networks seen (by Extended PAN ID)
   - Generates 8-character hexadecimal fingerprint for each device/network
   - Fingerprints used for device identification and tracking

7. **SD Card Logger** (`logDeviceToFile()` and `logToFile()` functions):
   - Creates unique log file on each boot with timestamp in filename
   - Filename format: `/scan_YYYYMMDD_HHMMSS.txt` (using RTC time) or `/scan_boot_XXXXX.txt` (using millis if no RTC)
   - Each device logged on one line with complete GPS data
   - Format: `TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...`
   - WiFi format: `WIFI,FP,Time,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Ch,Band,Enc`
   - BLE format: `BLE,FP,Time,Lat,Lon,Alt,Sats,HDOP,Name,Addr,RSSI,ManufData,ServiceUUID`
   - Zigbee format: `ZIGBEE,FP,Time,Lat,Lon,Alt,Sats,HDOP,PAN_ID,Ext_PAN_ID,Ch,PermitJoin,RouterCap,EDCap`
   - GPS data included with every device entry for location tracking
   - Falls back to RTC time when GPS not available (marked with "(RTC)" suffix)

8. **Status LED** (WS2812B RGB LED on GPIO27):
   - Provides visual feedback during boot/setup process
   - **Red**: System initializing (hardware init in progress)
   - **Orange**: Waiting for GPS signal (blocking until GPS fix acquired)
   - **Green**: Setup complete, ready to start scanning
   - **Off**: Main loop running (LED turned off to conserve battery)
   - Uses Adafruit NeoPixel library for control

9. **Battery Monitor** (`readBatteryPercent()` function):
   - Reads battery voltage via ADC on GPIO6
   - Assumes 100k/100k voltage divider (adjust `VOLTAGE_DIVIDER_RATIO` if different)
   - Calculates percentage based on 3.7V LiPo voltage range (3.0V empty, 4.2V full)
   - Updates every 5 seconds during main loop
   - Displays battery icon with percentage on OLED
   - Battery level shown during GPS wait screen

## Configuration

Key settings defined at the top of the sketch:

- **Output Flags**:
  - `ENABLE_CONSOLE_OUTPUT`: Enable/disable serial console output (default: true). Set to `false` in production to save processing time.
  - `ENABLE_DISPLAY_OUTPUT`: Enable/disable OLED display (default: true)
  - `ENABLE_LOG_OUTPUT`: Enable/disable SD card logging (default: true)
  - `ENABLE_ZIGBEE_SCAN`: Enable/disable Zigbee network scanning (default: true)

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

- **Status LED Pin**: GPIO27
  - WS2812B RGB LED for boot status indication
  - Automatically turns off when main loop starts

- **Battery ADC Pin**: GPIO6
  - Connect via voltage divider (100k/100k recommended for 3.7V LiPo)
  - Adjust `VOLTAGE_DIVIDER_RATIO` constant if using different resistor values
  - Battery thresholds: 3.0V (empty) to 4.2V (full)

- **Display Update Interval**: 500ms

- **Scan Interval**: 10 seconds
  - WiFi, BLE, and Zigbee scans run together at the same interval
  - Countdown timer on display shows seconds until next scan

- **BLE Scan Duration**: 5 seconds per scan

- **Zigbee Scan Duration**: 5 (scan duration setting 1-14, higher = longer)

- **Zigbee Mode** (Arduino IDE setting required):
  - **ED (End Device)**: Recommended for scanning - passive, low-power operation
  - **ZCZR (Coordinator/Router)**: Alternative mode, uses more power but can also route
  - Set via: Tools -> Zigbee Mode -> Zigbee ED (End Device)
  - Partition: Tools -> Partition Scheme -> Zigbee with spiffs (size for your flash)

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
- `Zigbee.h` - Zigbee/IEEE 802.15.4 network scanning (requires ESP32 Arduino Core v3.0+)
- `SD.h`, `SPI.h` - SD card access via SPI
- `HardwareSerial.h` - UART communication for GPS

External libraries required (install via Library Manager):
- `TinyGPSPlus` - GPS NMEA/UBLOX sentence parsing
- `Adafruit GFX Library` - Graphics library for OLED display
- `Adafruit SSD1306` - OLED display driver (compatible with SSD1309)
- `Adafruit NeoPixel` - WS2812B RGB LED control
- `ESP32Time` - RTC time management for ESP32 (persists time across reboots)

To install libraries:
```bash
# Using Arduino CLI
arduino-cli lib install "TinyGPSPlus"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit NeoPixel"
arduino-cli lib install "ESP32Time"

# Or in Arduino IDE: Tools > Manage Libraries > Search for each library name
```

## Testing

Monitor serial output at 115200 baud to see scan results in real-time.

**Status LED Indicators (WS2812B on GPIO27):**
- **Red**: System is initializing (hardware setup in progress)
- **Orange**: Waiting for GPS signal (blocking until fix acquired)
- **Green**: Setup complete, about to start scanning
- **Off**: Main loop running (LED turned off to conserve battery)

**OLED Display:**
The display provides real-time visual feedback:
- **Top Left**: Satellite icon with signal bars showing GPS satellite count (1-5 bars)
- **Top Center**: Battery indicator with icon and percentage
- **Top Right**: Degree heading (000-359°) followed by compass direction (N, NE, E, SE, S, SW, W, NW) based on GPS course (requires movement >1 km/h)
- **Center Left**: Device counts showing "W:X(XX)" for WiFi, "B:X(XX)" for BLE, and "Z:X(XX)" for Zigbee
  - First number = devices/networks found in last scan
  - Number in parentheses = total unique devices/networks seen since startup
- **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next scan
- **Bottom Left**: Current UTC time from GPS (HH:MM:SS format)
- **Bottom Right**: Current speed from GPS (format: "XXM YYK" for MPH and KPH, right-aligned)

**GPS Acquisition (Required Before Scanning):**
- Device waits for valid GPS signal before starting scan loop
- LED shows orange during GPS wait
- Display shows "Waiting GPS" with satellite count and elapsed time
- Battery level is also displayed during GPS wait
- May take 30-60 seconds outdoors for initial fix
- Serial monitor shows progress with satellite count updates
- Once GPS fix acquired, LED turns green briefly, then off
- Scanning begins automatically after GPS lock

**Log File Format:**

Each boot creates a unique log file with timestamp in the filename:
- With RTC time: `/scan_YYYYMMDD_HHMMSS.txt` (e.g., `/scan_20260121_153045.txt`)
- Without RTC time: `/scan_boot_XXXXX.txt` (e.g., `/scan_boot_1234.txt`)

Each file contains one line per device with complete GPS data.

**WiFi Entry Format:**
```
WIFI,XXXXXXXX,YYYY-MM-DD HH:MM:SS,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption
```

**BLE Entry Format:**
```
BLE,XXXXXXXX,YYYY-MM-DD HH:MM:SS,Lat,Lon,Alt,Sats,HDOP,Name,Address,RSSI,ManufData,ServiceUUID
```

**Zigbee Entry Format:**
```
ZIGBEE,XXXXXXXX,YYYY-MM-DD HH:MM:SS,Lat,Lon,Alt,Sats,HDOP,PAN_ID,Ext_PAN_ID,Channel,PermitJoin,RouterCap,EDCap
```

Where:
- `XXXXXXXX` = 8-character hexadecimal device fingerprint
- GPS coordinates, altitude, satellite count, and HDOP included with every device
- When GPS fix unavailable but RTC time exists, timestamp shows RTC time with "(RTC)" suffix
- When neither GPS nor RTC available, timestamp shows milliseconds and GPS fields show "N/A"

**Example WiFi Entry:**
```
WIFI,A3F2C891,2026-01-21 15:30:45,37.774929,-122.419418,15.50,8,1.20,MyNetwork,AA:BB:CC:DD:EE:FF,-65,6,2.4GHz,WPA2-PSK
```

**Example BLE Entry:**
```
BLE,B7E4D123,2026-01-21 15:30:46,37.774930,-122.419420,15.52,8,1.20,MyDevice,12:34:56:78:9A:BC,-72,ManufData,ServiceUUID
```

**Example Zigbee Entry:**
```
ZIGBEE,C8D5E234,2026-01-21 15:30:47,37.774931,-122.419422,15.54,8,1.20,0x1A2B,00:11:22:33:44:55:66:77,15,Yes,Yes,Yes
```

Where Zigbee fields are:
- `PAN_ID` = 16-bit network identifier (e.g., 0x1A2B)
- `Ext_PAN_ID` = 64-bit extended PAN ID (8 bytes, colon-separated)
- `Channel` = Zigbee channel number (11-26)
- `PermitJoin` = Whether network is accepting new devices (Yes/No)
- `RouterCap` = Router capacity available (Yes/No)
- `EDCap` = End device capacity available (Yes/No)
