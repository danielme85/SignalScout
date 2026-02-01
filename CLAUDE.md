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
- BOOT button on GPIO28 (for light sleep control)

## Development Environment

This is an Arduino sketch (.ino file) that should be developed using:
- Arduino IDE, or
- Arduino CLI, or
- PlatformIO

## File Structure

- `SignalScout.ino` - Main Arduino sketch with WiFi/BLE/Zigbee scanning and SD card logging

## Architecture

**Multithreaded FreeRTOS Design:**
The sketch uses FreeRTOS tasks for concurrent execution on the ESP32-C5's single-core RISC-V CPU (240MHz). Tasks are scheduled cooperatively, allowing simultaneous scanning and display updates without blocking. This significantly improves responsiveness and scan frequency.

**FreeRTOS Tasks:**
- **WiFi Scanner Task**: Scans WiFi networks every 10 seconds (~3 second duration, Priority 1)
- **BLE Scanner Task**: Scans Bluetooth devices every 10 seconds (~3 second duration, Priority 1)
- **Zigbee Scanner Task**: Scans Zigbee networks every 10 seconds (~3 second duration, Priority 1)
- **SD Logger Task**: Processes log queue and writes to SD card (Priority 3 - highest priority for data integrity)
- **Main Loop**: GPS reading (continuous), battery monitoring, display updates (every 1 second), and light sleep control

**Single-Core Architecture:**
- ESP32-C5 uses a 32-bit RISC-V single-core CPU operating at 240MHz
- FreeRTOS scheduler handles task switching and time-slicing on the single core
- Tasks are created with `xTaskCreate()` and scheduled based on priority
- Higher priority tasks preempt lower priority tasks when ready to run

**Thread Safety & Task Staggering:**
- **Mutexes**: Protect shared resources (device maps, SD card access, display)
- **Queue System**: Non-blocking log entry queue (50 entries) prevents SD card write conflicts
- **Staggered Scanning**: Tasks run sequentially every 10 seconds to prevent resource contention:
  - WiFi: Starts at t=0s (no delay), runs for ~3 seconds
  - BLE: Starts at t=3.5s (3500ms delay), runs for ~3 seconds
  - Zigbee: Starts at t=7s (7000ms delay), runs for ~3 seconds
  - This ensures smooth GPS updates and display rendering throughout the scan cycle

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
   - Updates every 1 second with real-time GPS and system status
   - **Top Left**: Satellite signal strength indicator with icon and bars (0-5 bars based on satellite count)
   - **Top Center**: Battery indicator with icon showing charge level and percentage
   - **Top Right**: Degree heading (000-359°) and compass direction (N, NE, E, SE, S, SW, W, NW)
   - **Center Left**: Device counts - WiFi "W:X(XX)", BLE "B:X(XX)", Zigbee "Z:X(XX)"
     - Asterisk (*) appears after count when actively scanning (e.g., "W:5(23)*")
     - Shows last scan count and total unique devices seen
   - **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next scan
   - **Bottom Left**: GPS time in HH:MM:SS UTC format (2px left margin, updates every second)
   - **Bottom Right**: Speed in both MPH and KPH (2px right margin, right-aligned)
   - All display elements use consistent 2px margins on left and right edges

6. **Device Tracking**:
   - Maintains count of unique WiFi devices seen (by BSSID)
   - Maintains count of unique BLE devices seen (by address)
   - Maintains count of unique Zigbee networks seen (by Extended PAN ID)
   - Generates 8-character hexadecimal fingerprint for each device/network
   - Fingerprints used for device identification and tracking

7. **SD Card Logger** (`sdLogTask()` FreeRTOS task):
   - Creates unique log file AFTER GPS lock or RTC time is available
   - Filename format: `/scan_YYYYMMDD_HHMMSS.txt` (using RTC time) or `/scan_boot_XXXXX.txt` (using millis if no RTC)
   - File is created and opened in write mode, then closed - subsequent writes use FILE_APPEND mode
   - Uses non-blocking queue system to prevent SD card write conflicts between scanning tasks
   - Queue size: 50 entries, protected by mutex for thread-safe access
   - Each device logged on one line with complete GPS data
   - Format: `TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...`
   - WiFi format: `WIFI,FP,Time,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Ch,Band,Enc`
   - BLE format: `BLE,FP,Time,Lat,Lon,Alt,Sats,HDOP,Name,Addr,RSSI,ManufData,ServiceUUID`
   - Zigbee format: `ZIGBEE,FP,Time,Lat,Lon,Alt,Sats,HDOP,PAN_ID,Ext_PAN_ID,Ch,PermitJoin,RouterCap,EDCap`
   - GPS data included with every device entry for location tracking
   - Falls back to RTC time when GPS not available (marked with "(RTC)" suffix)
   - Dedicated SD logger task runs on Core 0 with highest priority (3) for reliable logging

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

10. **Light Sleep Control** (`enterLightSleep()` function):
   - BOOT button on GPIO28 controls light sleep mode
   - Hold BOOT button for 3 seconds to enter light sleep (prevents accidental activation)
   - Display shows "Going to sleep..." message before entering light sleep
   - Device logs sleep event to SD card before sleeping
   - Wake up by holding BOOT button for 1 second (GPIO wakeup on any GPIO pin)
   - After waking, execution resumes from sleep point (no full restart required)
   - RTC time persists across sleep cycles, providing immediate time availability
   - Light sleep reduces power consumption significantly for battery-powered operation

## Configuration

Key settings defined at the top of the sketch:

- **Scanner Flags**:
  - `ENABLE_WIFI_SCAN`: Enable/disable WiFi network scanning (default: true)
  - `ENABLE_BLE_SCAN`: Enable/disable Bluetooth Low Energy scanning (default: true)
  - `ENABLE_ZIGBEE_SCAN`: Enable/disable Zigbee network scanning (default: true)

- **Output Flags**:
  - `ENABLE_CONSOLE_OUTPUT`: Enable/disable serial console output (default: true). Set to `false` in production to save processing time.
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

- **Status LED Pin**: GPIO27
  - WS2812B RGB LED for boot status indication
  - Automatically turns off when main loop starts

- **Battery ADC Pin**: GPIO6
  - Connect via voltage divider (200k/100k for 3.7V LiPo)
  - `VOLTAGE_DIVIDER_RATIO` set to 3.333333 for 200k/100k divider
  - Adjust constant if using different resistor values
  - Battery thresholds: 3.0V (empty) to 4.2V (full)

- **BOOT Button Pin**: GPIO28
  - Used for light sleep control (active LOW, internal pullup enabled)
  - Hold for 3 seconds to enter light sleep mode
  - Hold for 1 second to wake from light sleep
  - Prevents accidental sleep activation while allowing easy wake

- **Display Update Interval**: 1000ms (1 second)
  - Display updates handled in main loop with timing check
  - GPS data read continuously in main loop (non-blocking)

- **Scan Interval**: 10 seconds (full scan cycle)
  - WiFi, BLE, and Zigbee scans run sequentially on separate FreeRTOS tasks
  - Staggered timing prevents resource contention and ensures smooth display updates:
    - WiFi: Starts at t=0s, ~3 second duration
    - BLE: Starts at t=3.5s, ~3 second duration
    - Zigbee: Starts at t=7s, ~3 second duration
  - Non-blocking queue system for SD card logging after each scan completes
  - Active scanning flags (`wifiScanning`, `bleScanning`, `zigbeeScanning`) trigger asterisk display

- **BLE Scan Duration**: 3 seconds per scan

- **Zigbee Scan Duration**: 5 (scan duration setting 1-14, approximately 3 seconds)

- **WiFi Scan Duration**: ~3 seconds (per-channel timing: 120-150ms active, 400ms passive)

- **Zigbee Mode** (Arduino IDE setting required):
  - **ED (End Device)**: Recommended for scanning - passive, low-power operation
  - **ZCZR (Coordinator/Router)**: Alternative mode, uses more power but can also route
  - Set via: Tools -> Zigbee Mode -> Zigbee ED (End Device)
  - Partition: Tools -> Partition Scheme -> Zigbee with spiffs (size for your flash)

## Arduino-Specific Constraints

- The main file must have the same name as the parent directory (SignalScout)
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
arduino-cli compile --fqbn esp32:esp32:esp32c5 SignalScout

# Upload to board
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32c5 SignalScout

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
The display provides real-time visual feedback (updates every 1 second):
- **Top Left**: Satellite icon with signal bars showing GPS satellite count (1-5 bars)
- **Top Center**: Battery indicator with icon and percentage
- **Top Right**: Degree heading (000-359°) followed by compass direction (N, NE, E, SE, S, SW, W, NW) based on GPS course (requires movement >1 km/h)
- **Center Left**: Device counts showing "W:X(XX)" for WiFi, "B:X(XX)" for BLE, and "Z:X(XX)" for Zigbee
  - First number = devices/networks found in last scan
  - Number in parentheses = total unique devices/networks seen since startup
  - **Asterisk (*)** appears when actively scanning (e.g., "W:5(23)*" during WiFi scan)
- **Center Right**: Countdown timer "Scan in: Xs" showing seconds until next scan
- **Bottom Left**: Current UTC time from GPS (HH:MM:SS format, updates every second)
- **Bottom Right**: Current speed from GPS (format: "XXM YYK" for MPH and KPH, right-aligned)

**GPS Acquisition (Required Before Scanning):**
- Device waits for valid GPS signal before starting scan loop
- LED shows orange during GPS wait
- Display shows "Waiting GPS" with satellite count and elapsed time
- Battery level is also displayed during GPS wait
- May take 30-60 seconds outdoors for initial fix
- Serial monitor shows progress with satellite count updates
- Once GPS fix acquired, LED turns green briefly, then off
- Scanning begins automatically after GPS lock in 10-second cycles:
  - WiFi scan starts immediately (~3 seconds)
  - BLE scan starts 3.5 seconds later (~3 seconds)
  - Zigbee scan starts 7 seconds later (~3 seconds)
  - Cycle repeats every 10 seconds
- Asterisk indicator shows which scanner is currently active

**Light Sleep Mode (Battery Conservation):**
- Hold BOOT button (GPIO28) for 3 seconds to enter light sleep
- Display shows "Going to sleep..." message before entering sleep mode
- Device logs "Entering light sleep mode" to SD card before sleeping
- In light sleep, power consumption is significantly reduced while maintaining system state
- To wake up, hold BOOT button for 1 second
- After waking, execution continues from sleep point (faster wake than deep sleep)
- RTC time persists across sleep cycles, ensuring accurate timestamps resume immediately
- GPS lock is maintained during light sleep (faster resumption of scanning)
- Useful for battery-powered operation when not actively scanning

**Log File Format:**

Each boot creates a unique log file with timestamp in the filename:
- With RTC time: `/scan_YYYYMMDD_HHMMSS.txt` (e.g., `/scan_20260121_153045.txt`)
- Without RTC time: `/scan_boot_XXXXX.txt` (e.g., `/scan_boot_1234.txt`)

Each file begins with a comprehensive header explaining column formats and field descriptions, followed by one line per device with complete GPS data.

**Column Headers:**

WiFi Format:
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption
```

BLE Format:
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,Name,Address,RSSI,ManufacturerData,ServiceUUID
```

Zigbee Format:
```
Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,PAN_ID,ExtendedPAN_ID,Channel,PermitJoin,RouterCapacity,EndDeviceCapacity
```

**Common Field Descriptions:**

- **Type**: Entry type (WIFI, BLE, or ZIGBEE)
- **Fingerprint**: 8-character hexadecimal unique identifier derived from device MAC address or network Extended PAN ID
  - WiFi: Based on BSSID (access point MAC address)
  - BLE: Based on Bluetooth MAC address
  - Zigbee: Based on Extended PAN ID
  - Same device/network always gets same fingerprint (stable identifier)
- **Timestamp**: UTC time from GPS in format `YYYY-MM-DD HH:MM:SS`
  - Appends `(RTC)` suffix when using RTC fallback time
  - Shows milliseconds if neither GPS nor RTC available
- **Latitude**: GPS latitude in decimal degrees (positive = North, negative = South)
- **Longitude**: GPS longitude in decimal degrees (positive = East, negative = West)
- **Altitude**: Elevation in meters above sea level from GPS
- **Satellites**: Number of GPS satellites currently visible
- **HDOP**: Horizontal Dilution of Precision - GPS accuracy indicator
  - Lower values = better accuracy
  - <2.0 = Good, 2-5 = Moderate, >5 = Poor

**WiFi-Specific Fields:**

- **SSID**: Network name (shows `<hidden>` for hidden networks)
- **BSSID**: Access point MAC address in format `XX:XX:XX:XX:XX:XX`
- **RSSI**: Received Signal Strength Indicator in dBm
  - Higher (less negative) = stronger signal
  - Typical range: -30 dBm (excellent) to -90 dBm (weak)
- **Channel**: WiFi channel number
  - Channels 1-14: 2.4GHz band
  - Channels 32+: 5GHz band
- **Band**: Frequency band (`2.4GHz` or `5GHz`)
- **Encryption**: Security type
  - Examples: `OPEN`, `WEP`, `WPA-PSK`, `WPA2-PSK`, `WPA3-PSK`, `WPA2/WPA3-PSK`, `WPA2-ENT`

**BLE-Specific Fields:**

- **Name**: Bluetooth device name as advertised (shows `Unknown` if not provided)
- **Address**: Bluetooth MAC address in format `xx:xx:xx:xx:xx:xx`
- **RSSI**: Received Signal Strength Indicator in dBm (higher = stronger)
- **ManufacturerData**: Hexadecimal representation of manufacturer-specific advertising data
  - Format: Raw bytes as hex string (e.g., `4C001005` for Apple)
  - Empty if not advertised
- **ServiceUUID**: Primary service UUID advertised by device
  - Format: Standard UUID (e.g., `0000180A` for Device Information Service)
  - Empty if not advertised

**Zigbee-Specific Fields:**

- **PAN_ID**: 16-bit Personal Area Network identifier in hexadecimal (e.g., `0x1A2B`)
  - Range: 0x0000 to 0xFFFF
  - Used for basic network identification
- **ExtendedPAN_ID**: 64-bit globally unique network identifier
  - Format: 8 bytes as colon-separated hex (e.g., `00:11:22:33:44:55:66:77`)
  - Globally unique, never changes for a network
- **Channel**: Zigbee radio channel (11-26)
  - All Zigbee channels are in 2.4GHz band
  - Channel spacing: 5 MHz
- **PermitJoin**: Whether network currently accepts new devices joining (`Yes` or `No`)
- **RouterCapacity**: Whether network can accept additional router devices (`Yes` or `No`)
- **EndDeviceCapacity**: Whether network can accept additional end devices (`Yes` or `No`)

**Example Entries:**

WiFi (Home router on 2.4GHz):
```
WIFI,3C7B6E95,2026-01-24 22:57:04,41.342822,-81.389317,327.20,8,1.34,MyHomeNetwork,60:B7:6E:6D:99:95,-45,6,2.4GHz,WPA2-PSK
```

BLE (Smart watch):
```
BLE,FA2FAF58,2026-01-24 22:57:16,41.342820,-81.389308,327.90,8,1.34,Smart Watch,58:D9:FA:AF:2F:FD,-65,4C001005,0000180A
```

Zigbee (Smart home hub):
```
ZIGBEE,8A3F5C12,2026-01-24 22:57:28,41.342818,-81.389299,328.10,8,1.34,0x1A2B,00:11:22:33:44:55:66:77,15,Yes,Yes,No
```

**Data Quality Notes:**

- All strings (SSID, BLE names) are sanitized to printable ASCII characters only
- Non-printable characters, control codes, and extended UTF-8 are removed
- BLE manufacturer data is converted to clean hexadecimal representation
- All log entries are guaranteed null-terminated with no garbage characters
- CSV format allows easy import into spreadsheets, databases, or GIS tools
