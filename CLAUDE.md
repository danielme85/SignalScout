# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**SignalScout** - WiFi and Bluetooth scanner for ESP32-C5 with two operating modes:

**Scan Mode (Default on boot):**
- Waits for GPS signal lock before starting scans
- Scans WiFi networks on both 2.4GHz and 5GHz bands
- Scans Bluetooth Low Energy (BLE) devices
- GPS-timestamped logging with location data
- Real-time OLED display showing GPS status, satellite count, compass, time, and speed
- Logs all scan results to SD card via SPI
- Outputs scan data to serial monitor for debugging

**File Sharing Mode (Activated via button):**
- Connects to WiFi network and hosts web server on port 80
- Browse and download scan files from SD card via web browser
- No GPS required - starts immediately when entered
- Low power consumption, ideal for data retrieval

Hardware requirements:
- ESP32-C5 board (16MB FLASH, 8MB PSRAM)
- SD card module connected via SPI
- NEO-6M GPS module connected via UART (TX/RX)
- SSD1309 OLED display 128x64 connected via SPI
- WS2812B RGB LED on GPIO27 (status indicator)
- 3.7V LiPo battery (e.g., 3000mAh) with voltage divider on GPIO6 for monitoring
- Mode control button on GPIO23 (toggle between modes, deep sleep control)

## Development Environment

This is an Arduino sketch (.ino file) that should be developed using:
- Arduino IDE, or
- Arduino CLI, or
- PlatformIO

## File Structure

- `SignalScout.ino` - Main Arduino sketch with WiFi/BLE scanning and SD card logging

## Architecture

**Multithreaded FreeRTOS Design:**
The sketch uses FreeRTOS tasks for concurrent execution on the ESP32-C5's single-core RISC-V CPU (240MHz). Tasks are scheduled cooperatively, allowing simultaneous scanning and display updates without blocking. This significantly improves responsiveness and scan frequency.

**FreeRTOS Tasks:**
- **Unified Scanner Task**: Scans WiFi then BLE sequentially every 10 seconds (~3 second duration each, Priority 1)
- **SD Logger Task**: Processes log queue and writes to SD card (Priority 3 - highest priority for data integrity)
- **Main Loop**: GPS reading (continuous), battery monitoring, display updates (every 1 second), and button handling

**Single-Core Architecture:**
- ESP32-C5 uses a 32-bit RISC-V single-core CPU operating at 240MHz
- FreeRTOS scheduler handles task switching and time-slicing on the single core
- Tasks are created with `xTaskCreate()` and scheduled based on priority
- Higher priority tasks preempt lower priority tasks when ready to run

**Thread Safety & Radio Sequencing:**
- **Mutexes**: Protect shared resources (device maps, SD card access, display)
- **Queue System**: Non-blocking log entry queue (50 entries) prevents SD card write conflicts
- **Sequential Scanning**: WiFi and BLE scans run back-to-back within a single unified task to prevent radio conflicts on ESP32-C5's shared 2.4GHz radio:
  - WiFi: Runs first (~3 seconds)
  - BLE: Runs after WiFi completes (~3 seconds)
  - Cycle repeats every 10 seconds

The sketch is structured around four main components:

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
   - Syncs ESP32 RTC from GPS time (updates every 60 seconds for accuracy)
   - RTC time persists across reboots, allowing immediate time availability on boot

4. **OLED Display** (`updateDisplay()` function):
   - Updates every 1 second with real-time GPS and system status
   - **Top Left**: Satellite signal strength indicator with icon and bars (0-5 bars based on satellite count)
   - **Top Center**: Battery indicator with icon showing charge level and percentage
   - **Top Right**: Compass direction (N, NE, E, SE, S, SW, W, NW)
   - **Center Left**: Device counts - WiFi "W:X(XX)", BLE "B:X(XX)"
     - Asterisk (*) appears after count when actively scanning (e.g., "W:5(23)*")
     - Shows last scan count and total unique devices seen
   - **Center Right**: Animated WiFi logo
   - **Bottom Left**: GPS time in HH:MM:SS UTC format (2px left margin, updates every second)
   - **Bottom Right**: Speed in both MPH and KPH (2px right margin, right-aligned)
   - All display elements use consistent 2px margins on left and right edges

5. **Device Tracking**:
   - Maintains count of unique WiFi devices seen (by BSSID)
   - Maintains count of unique BLE devices seen (by address)
   - Generates 8-character hexadecimal fingerprint for each device
   - Fingerprints used for device identification and tracking

6. **SD Card Logger** (`sdLogTask()` FreeRTOS task):
   - Creates unique log file AFTER GPS lock or RTC time is available
   - Filename format: `/scan_YYYYMMDD_HHMMSS.txt` (using RTC time) or `/scan_boot_XXXXX.txt` (using millis if no RTC)
   - File is created and opened in write mode, then closed - subsequent writes use FILE_APPEND mode
   - Uses non-blocking queue system to prevent SD card write conflicts between scanning tasks
   - Queue size: 50 entries, protected by mutex for thread-safe access
   - Each device logged on one line with complete GPS data
   - Format: `TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...`
   - WiFi format: `WIFI,FP,Time,Lat,Lon,Alt,Sats,HDOP,SSID,BSSID,RSSI,Ch,Band,Enc`
   - BLE format: `BLE,FP,Time,Lat,Lon,Alt,Sats,HDOP,Name,Addr,RSSI,ManufData,ServiceUUID`
   - GPS data included with every device entry for location tracking
   - Falls back to RTC time when GPS not available (marked with "(RTC)" suffix)
   - Dedicated SD logger task runs with highest priority (3) for reliable logging

7. **Status LED** (WS2812B RGB LED on GPIO27):
   - Provides visual feedback during boot/setup process
   - **Red**: System initializing (hardware init in progress)
   - **Orange**: Waiting for GPS signal (blocking until GPS fix acquired)
   - **Green**: Setup complete, ready to start scanning
   - **Off**: Main loop running (LED turned off to conserve battery)
   - Uses Adafruit NeoPixel library for control

8. **Battery Monitor** (`readBatteryPercent()` function):
   - Reads battery voltage via ADC on GPIO6
   - Assumes 200k/100k voltage divider (adjust `VOLTAGE_DIVIDER_RATIO` if different)
   - Calculates percentage based on 3.7V LiPo voltage range (3.0V empty, 4.2V full)
   - Updates every 5 seconds during main loop
   - Displays battery icon with percentage on OLED
   - Battery level shown during GPS wait screen

9. **Light Sleep Control** (`enterLightSleep()` function):
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

- **Output Flags**:
  - `ENABLE_CONSOLE_OUTPUT`: Enable/disable serial console output (default: false). Set to `false` in production to save processing time.
  - `ENABLE_DISPLAY_OUTPUT`: Enable/disable OLED display (default: true)
  - `ENABLE_LOG_OUTPUT`: Enable/disable SD card logging (default: true)

- **SD Card SPI Pins**: CS=1, MOSI=0, MISO=3, SCK=2
  - Adjust these to match your SD card module wiring

- **GPS UART Pins**: RX=14, TX=13, Baud=9600
  - RX pin connects to GPS module TX
  - TX pin connects to GPS module RX
  - Adjust pins to match your wiring

- **OLED Display SPI Pins**: MOSI=26, CLK=25, DC=9, CS=8, RESET=10
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

- **Mode Control Button Pin**: GPIO23 (SHARE_BUTTON_PIN)
  - Used for mode switching and deep sleep control (active LOW, internal pullup enabled)
  - Hold for 1 second: Toggle between file sharing mode and scan mode
  - Hold for 3 seconds: Enter deep sleep mode
  - Device boots into scan mode by default (waits for GPS signal on first boot)
  - GPS signal is acquired on boot before scanning begins

- **Display Update Interval**: 1000ms (1 second)
  - Display updates handled in main loop with timing check
  - GPS data read continuously in main loop (non-blocking)

- **Scan Interval**: 10 seconds (full scan cycle)
  - WiFi and BLE scans run sequentially within the unified scan task
  - WiFi scans first (~3 seconds), then BLE (~3 seconds)
  - Non-blocking queue system for SD card logging after each scan completes
  - Active scanning flags (`wifiScanning`, `bleScanning`) trigger asterisk display

- **BLE Scan Duration**: 3 seconds per scan

- **WiFi Scan Duration**: ~3 seconds (per-channel timing: 120-150ms active, 400ms passive)

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
- `SD.h`, `SPI.h` - SD card access via SPI
- `HardwareSerial.h` - UART communication for GPS

External libraries required (install via Library Manager):
- `TinyGPSPlus` - GPS NMEA/UBLOX sentence parsing
- `Adafruit GFX Library` - Graphics library for OLED display
- `Adafruit SSD1306` - OLED display driver (compatible with SSD1309)
- `Adafruit NeoPixel` - WS2812B RGB LED control
- `ESP32Time` - RTC time management for ESP32 (persists time across reboots)
- `ESPAsyncWebServer` - Async web server for file sharing mode
- `AsyncTCP` - TCP library required by ESPAsyncWebServer

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
- **Top Right**: Compass direction (N, NE, E, SE, S, SW, W, NW) based on GPS course (requires movement >1 km/h)
- **Center Left**: Device counts showing "W:X(XX)" for WiFi, "B:X(XX)" for BLE
  - First number = devices found in last scan
  - Number in parentheses = total unique devices seen since startup
  - **Asterisk (*)** appears when actively scanning (e.g., "W:5(23)*" during WiFi scan)
- **Center Right**: Animated WiFi logo
- **Bottom Left**: Current UTC time from GPS (HH:MM:SS format, updates every second)
- **Bottom Right**: Current speed from GPS (format: "XXM YYK" for MPH and KPH, right-aligned)

**GPS Acquisition (on boot):**
- Device boots into scan mode by default and waits for GPS signal
- LED shows orange during GPS wait
- Display shows "Waiting GPS" with satellite count and elapsed time
- Battery level is also displayed during GPS wait
- May take 30-60 seconds outdoors for initial fix
- Serial monitor shows progress with satellite count updates
- Once GPS fix acquired, LED turns green briefly, then off
- Scanning begins automatically after GPS lock in 10-second cycles:
  - WiFi scan runs first (~3 seconds)
  - BLE scan runs after WiFi completes (~3 seconds)
  - Cycle repeats every 10 seconds
- Asterisk indicator shows which scanner is currently active
- If GPS was already acquired (e.g. re-entering scan mode), the GPS wait is skipped

**Operating Modes:**

The device has two primary operating modes:

1. **Scan Mode (Default):**
   - Device boots into scan mode by default
   - Waits for GPS signal lock before starting (only on first entry to scan mode)
   - Starts WiFi and BLE scanning tasks
   - Logs all scan results to SD card with GPS timestamps
   - Display shows real-time GPS data, device counts, and scanning status
   - To switch: Hold mode button for 1 second to enter file sharing mode

2. **File Sharing Mode:**
   - Activated by holding the mode button (GPIO23) for 1 second while in scan mode
   - Connects to WiFi network specified in `secrets.h` (WIFI_SSID and WIFI_PASSWORD)
   - Hosts a web server on port 80 for browsing and downloading scan files
   - No GPS signal required
   - Scanning tasks are suspended to avoid SD card and radio conflicts
   - Display shows IP address and mode instructions
   - Access files by opening `http://[IP_ADDRESS]` in a web browser
   - To exit: Hold mode button for 1 second to return to scan mode

**Deep Sleep Mode (Battery Conservation):**
- Hold mode button (GPIO23) for 3 seconds to enter deep sleep
- Display shows "Going to sleep..." message before entering sleep mode
- Device logs "Entering deep sleep mode" to SD card before sleeping
- In deep sleep, power consumption is minimized (device essentially off)
- To wake up, press the mode button (device will reboot)
- After waking, device reboots and enters scan mode by default
- RTC time persists across sleep cycles, ensuring accurate timestamps resume immediately
- Useful for battery-powered operation when not in use

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

**Common Field Descriptions:**

- **Type**: Entry type (WIFI or BLE)
- **Fingerprint**: 8-character hexadecimal unique identifier derived from device MAC address
  - WiFi: Based on BSSID (access point MAC address)
  - BLE: Based on Bluetooth MAC address
  - Same device always gets same fingerprint (stable identifier)
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

**Example Entries:**

WiFi (Home router on 2.4GHz):
```
WIFI,3C7B6E95,2026-01-24 22:57:04,41.342822,-81.389317,327.20,8,1.34,MyHomeNetwork,60:B7:6E:6D:99:95,-45,6,2.4GHz,WPA2-PSK
```

BLE (Smart watch):
```
BLE,FA2FAF58,2026-01-24 22:57:16,41.342820,-81.389308,327.90,8,1.34,Smart Watch,58:D9:FA:AF:2F:FD,-65,4C001005,0000180A
```

**Data Quality Notes:**

- All strings (SSID, BLE names) are sanitized to printable ASCII characters only
- Non-printable characters, control codes, and extended UTF-8 are removed
- BLE manufacturer data is converted to clean hexadecimal representation
- All log entries are guaranteed null-terminated with no garbage characters
- CSV format allows easy import into spreadsheets, databases, or GIS tools
