/*
 * SignalScout - WiFi and Bluetooth Scanner for ESP32-C5
 * Scans 2.4GHz and 5GHz WiFi networks and Bluetooth devices
 * Logs all activity to SD card over SPI with GPS timestamps and location
 * Displays status on SSD1309 OLED display
 *
 * Hardware: ESP32-C5 (16MB FLASH, 8MB PSRAM)
 * SD Card: Connected via SPI
 * GPS: NEO-6M connected via UART (TX/RX)
 * OLED: SSD1309 128x64 connected via SPI
 */

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// SD Card SPI Pins (adjust these based on your wiring)
#define SD_CS    5   // Chip Select
#define SD_MOSI  23  // MOSI
#define SD_MISO  19  // MISO
#define SD_SCK   18  // SCK

// GPS UART Pins (adjust these based on your wiring)
#define GPS_RX   16  // ESP32 RX pin (connects to GPS TX)
#define GPS_TX   17  // ESP32 TX pin (connects to GPS RX)
#define GPS_BAUD 9600

// OLED Display SPI Pins (adjust these based on your wiring)
#define OLED_MOSI  23  // Can share with SD card MOSI
#define OLED_CLK   18  // Can share with SD card SCK
#define OLED_DC    4   // Data/Command pin
#define OLED_CS    15  // Chip Select (different from SD card)
#define OLED_RESET 2   // Reset pin
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Scan settings
#define SCAN_INTERVAL 10000  // WiFi and BLE scan every 10 seconds
#define BLE_SCAN_TIME 5      // BLE scan duration in seconds
#define LOG_FILE "/scanner_log.txt"

// Output enable/disable flags
#define ENABLE_CONSOLE_OUTPUT true   // Enable/disable serial console output
#define ENABLE_DISPLAY_OUTPUT true   // Enable/disable OLED display output
#define ENABLE_LOG_OUTPUT true       // Enable/disable SD card logging

// Timing variables
unsigned long lastScan = 0;
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_UPDATE_INTERVAL 500  // Update display every 500ms

// Device tracking
#include <map>
#include <string>
std::map<std::string, bool> seenWiFiDevices;  // Track unique WiFi devices by BSSID
std::map<std::string, bool> seenBLEDevices;   // Track unique BLE devices by address
int uniqueWiFiCount = 0;
int uniqueBLECount = 0;
int lastWiFiScanCount = 0;  // Devices found in last scan
int lastBLEScanCount = 0;   // Devices found in last scan

// GPS variables
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1
bool gpsTimeValid = false;

// OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

BLEScan* pBLEScan;

// BLE Scan callback class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // This will be called for each BLE device found during scan
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (ENABLE_CONSOLE_OUTPUT) {
    Serial.println("SignalScout - WiFi & Bluetooth Scanner with GPS");
    Serial.println("================================================");
  }

  // Initialize OLED Display
  if (ENABLE_DISPLAY_OUTPUT) {
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
      if (ENABLE_CONSOLE_OUTPUT) Serial.println("ERROR: SSD1306 allocation failed!");
      while (1) delay(1000);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
    if (ENABLE_CONSOLE_OUTPUT) Serial.println("Display initialized");
  }

  // Initialize GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("GPS initialized. Waiting for GPS fix...");
  if (ENABLE_DISPLAY_OUTPUT) updateDisplay("Waiting for GPS");

  // Initialize SD card
  if (ENABLE_LOG_OUTPUT) {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
      if (ENABLE_CONSOLE_OUTPUT) {
        Serial.println("ERROR: SD Card initialization failed!");
        Serial.println("Check SD card and wiring. System halted.");
      }
      while (1) delay(1000);
    }

    if (ENABLE_CONSOLE_OUTPUT) Serial.println("SD Card initialized successfully");
    logToFile("System started - SignalScout initialized");
  }

  // Set WiFi to station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Initialize BLE
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("Initializing BLE...");
  BLEDevice::init("SignalScout");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  if (ENABLE_CONSOLE_OUTPUT) Serial.println("Initialization complete. Starting scans...\n");
  if (ENABLE_LOG_OUTPUT) logToFile("Initialization complete");
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Check if we have a valid GPS time fix
  if (gps.time.isValid() && gps.date.isValid() && !gpsTimeValid) {
    gpsTimeValid = true;
    if (ENABLE_CONSOLE_OUTPUT) Serial.println("GPS time lock acquired!");
    if (ENABLE_LOG_OUTPUT) logToFile("GPS time lock acquired");
    if (ENABLE_CONSOLE_OUTPUT) displayGPSInfo();
  }

  unsigned long currentTime = millis();

  // Update display
  if (ENABLE_DISPLAY_OUTPUT && currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay("");
    lastDisplayUpdate = currentTime;
  }

  // Perform WiFi and BLE scans together
  if (currentTime - lastScan >= SCAN_INTERVAL) {
    scanWiFi();
    scanBluetooth();
    lastScan = currentTime;
  }

  delay(100);
}

void scanWiFi() {
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("\n--- WiFi Scan Starting ---");

  int networksFound = WiFi.scanNetworks();
  lastWiFiScanCount = networksFound;  // Track last scan count

  if (ENABLE_CONSOLE_OUTPUT) Serial.printf("Networks found: %d\n", networksFound);

  if (networksFound == 0) {
    if (ENABLE_CONSOLE_OUTPUT) Serial.println("No networks found");
  } else {
    for (int i = 0; i < networksFound; i++) {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      wifi_auth_mode_t encryption = WiFi.encryptionType(i);
      int32_t channel = WiFi.channel(i);
      uint8_t* bssid = WiFi.BSSID(i);

      // Format BSSID
      char bssidStr[18];
      snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

      // Track unique devices by BSSID
      std::string bssidKey(bssidStr);
      if (seenWiFiDevices.find(bssidKey) == seenWiFiDevices.end()) {
        seenWiFiDevices[bssidKey] = true;
        uniqueWiFiCount++;
      }

      // Determine frequency band based on channel
      String band = (channel <= 14) ? "2.4GHz" : "5GHz";

      // Encryption type
      String encType;
      switch (encryption) {
        case WIFI_AUTH_OPEN: encType = "OPEN"; break;
        case WIFI_AUTH_WEP: encType = "WEP"; break;
        case WIFI_AUTH_WPA_PSK: encType = "WPA-PSK"; break;
        case WIFI_AUTH_WPA2_PSK: encType = "WPA2-PSK"; break;
        case WIFI_AUTH_WPA_WPA2_PSK: encType = "WPA/WPA2-PSK"; break;
        case WIFI_AUTH_WPA2_ENTERPRISE: encType = "WPA2-ENT"; break;
        case WIFI_AUTH_WPA3_PSK: encType = "WPA3-PSK"; break;
        case WIFI_AUTH_WPA2_WPA3_PSK: encType = "WPA2/WPA3-PSK"; break;
        default: encType = "UNKNOWN"; break;
      }

      // Generate fingerprint (simple hash)
      String fingerprintData = String(bssidStr) + ssid + encType;
      uint32_t fingerprint = 0;
      for (int j = 0; j < fingerprintData.length(); j++) {
        fingerprint = fingerprint * 31 + fingerprintData.charAt(j);
      }
      char fingerprintStr[9];
      snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);

      // Print to Serial
      if (ENABLE_CONSOLE_OUTPUT) {
        Serial.printf("%d: %s (%s) | RSSI: %d dBm | Ch: %d (%s) | Enc: %s | FP: %s\n",
                      i + 1, ssid.c_str(), bssidStr, rssi, channel, band.c_str(),
                      encType.c_str(), fingerprintStr);
      }

      // Log to SD card - one line per device with GPS data
      if (ENABLE_LOG_OUTPUT) {
        logDeviceToFile("WIFI", ssid, String(bssidStr), String(rssi),
                        String(channel), band, encType, fingerprintStr);
      }
    }
  }

  WiFi.scanDelete();
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("--- WiFi Scan Complete ---\n");
}

void scanBluetooth() {
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("\n--- Bluetooth Scan Starting ---");

  BLEScanResults foundDevices = pBLEScan->start(BLE_SCAN_TIME, false);
  int deviceCount = foundDevices.getCount();
  lastBLEScanCount = deviceCount;  // Track last scan count

  if (ENABLE_CONSOLE_OUTPUT) Serial.printf("Bluetooth devices found: %d\n", deviceCount);

  for (int i = 0; i < deviceCount; i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);

    String address = device.getAddress().toString().c_str();
    String name = device.haveName() ? device.getName().c_str() : "Unknown";
    int rssi = device.getRSSI();

    // Track unique devices by address
    std::string addrKey = address.c_str();
    if (seenBLEDevices.find(addrKey) == seenBLEDevices.end()) {
      seenBLEDevices[addrKey] = true;
      uniqueBLECount++;
    }

    // Get additional info
    String manufData = "";
    String serviceUUID = "";
    if (device.haveManufacturerData()) {
      manufData = device.getManufacturerData().c_str();
    }
    if (device.haveServiceUUID()) {
      serviceUUID = device.getServiceUUID().toString().c_str();
    }

    // Generate fingerprint
    String fingerprintData = address + name + manufData;
    uint32_t fingerprint = 0;
    for (int j = 0; j < fingerprintData.length(); j++) {
      fingerprint = fingerprint * 31 + fingerprintData.charAt(j);
    }
    char fingerprintStr[9];
    snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);

    // Print to Serial
    if (ENABLE_CONSOLE_OUTPUT) {
      Serial.printf("%d: %s | RSSI: %d dBm | Addr: %s | FP: %s\n",
                    i + 1, name.c_str(), rssi, address.c_str(), fingerprintStr);

      if (manufData.length() > 0) {
        Serial.printf("   Manufacturer Data: %s\n", manufData.c_str());
      }
      if (serviceUUID.length() > 0) {
        Serial.printf("   Service UUID: %s\n", serviceUUID.c_str());
      }
    }

    // Log to SD card - one line per device with GPS data
    if (ENABLE_LOG_OUTPUT) {
      logDeviceToFile("BLE", name, address, String(rssi),
                      manufData, serviceUUID, "", fingerprintStr);
    }
  }

  pBLEScan->clearResults();
  if (ENABLE_CONSOLE_OUTPUT) Serial.println("--- Bluetooth Scan Complete ---\n");
}

void logDeviceToFile(String deviceType, String param1, String param2, String param3,
                     String param4, String param5, String param6, String fingerprint) {
  if (!ENABLE_LOG_OUTPUT) return;

  File logFile = SD.open(LOG_FILE, FILE_APPEND);

  if (!logFile) {
    if (ENABLE_CONSOLE_OUTPUT) Serial.println("ERROR: Failed to open log file for writing");
    return;
  }

  String timestamp;
  String gpsLat = "N/A";
  String gpsLon = "N/A";
  String gpsAlt = "N/A";
  String gpsSats = "N/A";
  String gpsHdop = "N/A";

  // Get GPS time and location
  if (gpsTimeValid && gps.time.isValid() && gps.date.isValid()) {
    char dateTime[32];
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    timestamp = String(dateTime);

    if (gps.location.isValid()) {
      gpsLat = String(gps.location.lat(), 6);
      gpsLon = String(gps.location.lng(), 6);
    }
    if (gps.altitude.isValid()) {
      gpsAlt = String(gps.altitude.meters(), 2);
    }
    if (gps.satellites.isValid()) {
      gpsSats = String(gps.satellites.value());
    }
    if (gps.hdop.isValid()) {
      gpsHdop = String(gps.hdop.hdop(), 2);
    }
  } else {
    timestamp = String(millis()) + "ms";
  }

  // Format: TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,Param1,Param2,Param3,Param4,Param5,Param6
  String logEntry = deviceType + "," + fingerprint + "," + timestamp + "," +
                    gpsLat + "," + gpsLon + "," + gpsAlt + "," + gpsSats + "," + gpsHdop + ",";

  if (deviceType == "WIFI") {
    // WIFI: SSID, BSSID, RSSI, Channel, Band, Encryption
    logEntry += param1 + "," + param2 + "," + param3 + "," + param4 + "," + param5 + "," + param6;
  } else if (deviceType == "BLE") {
    // BLE: Name, Address, RSSI, ManufData, ServiceUUID
    logEntry += param1 + "," + param2 + "," + param3 + "," + param4 + "," + param5;
  }

  logFile.println(logEntry);
  logFile.close();
}

void logToFile(String message) {
  if (!ENABLE_LOG_OUTPUT) return;

  File logFile = SD.open(LOG_FILE, FILE_APPEND);

  if (!logFile) {
    if (ENABLE_CONSOLE_OUTPUT) Serial.println("ERROR: Failed to open log file for writing");
    return;
  }

  String timestamp;
  String gpsData;

  // Use GPS time if available, otherwise use millis()
  if (gpsTimeValid && gps.time.isValid() && gps.date.isValid()) {
    // Format: YYYY-MM-DD HH:MM:SS
    char dateTime[32];
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    timestamp = String(dateTime);

    // Add GPS location data
    if (gps.location.isValid()) {
      char location[64];
      snprintf(location, sizeof(location), " | Lat: %.6f, Lon: %.6f",
               gps.location.lat(), gps.location.lng());
      gpsData = String(location);
    }

    // Add altitude/elevation
    if (gps.altitude.isValid()) {
      char alt[32];
      snprintf(alt, sizeof(alt), ", Alt: %.2fm", gps.altitude.meters());
      gpsData += String(alt);
    }

    // Add satellite count
    if (gps.satellites.isValid()) {
      gpsData += ", Sats: " + String(gps.satellites.value());
    }

    // Add HDOP (accuracy indicator)
    if (gps.hdop.isValid()) {
      char hdop[16];
      snprintf(hdop, sizeof(hdop), ", HDOP: %.2f", gps.hdop.hdop());
      gpsData += String(hdop);
    }
  } else {
    // Fallback to milliseconds since boot
    timestamp = String(millis()) + "ms";
    gpsData = " | GPS: No fix";
  }

  // Format: [timestamp] gpsData | message
  String logEntry = "[" + timestamp + "]" + gpsData + " | " + message;

  logFile.println(logEntry);
  logFile.close();

  // Also print errors to serial for debugging
  if (ENABLE_CONSOLE_OUTPUT && message.startsWith("ERROR")) {
    Serial.println(logEntry);
  }
}

void displayGPSInfo() {
  if (!ENABLE_CONSOLE_OUTPUT) return;

  Serial.println("\n--- GPS Information ---");

  if (gps.location.isValid()) {
    Serial.printf("Location: Lat: %.6f, Lon: %.6f\n",
                  gps.location.lat(), gps.location.lng());
  } else {
    Serial.println("Location: INVALID");
  }

  if (gps.altitude.isValid()) {
    Serial.printf("Altitude: %.2f meters\n", gps.altitude.meters());
  } else {
    Serial.println("Altitude: INVALID");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("Date/Time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.println("Date/Time: INVALID");
  }

  if (gps.satellites.isValid()) {
    Serial.printf("Satellites: %d\n", gps.satellites.value());
  } else {
    Serial.println("Satellites: INVALID");
  }

  if (gps.hdop.isValid()) {
    Serial.printf("HDOP: %.2f\n", gps.hdop.hdop());
  }

  if (gps.speed.isValid()) {
    Serial.printf("Speed: %.2f km/h\n", gps.speed.kmph());
  }

  Serial.println("--- End GPS Info ---\n");
}

void updateDisplay(String statusMessage) {
  display.clearDisplay();

  // Top Left: Satellite Signal Strength Indicator
  drawSatelliteIndicator();

  // Top Right: Compass Direction
  drawCompass();

  // Center: Status Message, Countdown Timer, or Device Stats
  if (statusMessage.length() > 0 || !gpsTimeValid) {
    display.setTextSize(2);
    display.setCursor(0, 24);
    if (statusMessage.length() > 0) {
      display.println(statusMessage);
    } else if (!gpsTimeValid) {
      display.println("Waiting GPS");
    }
  } else {
    // Show device statistics and countdown
    display.setTextSize(1);

    // WiFi count: Last scan (Total)
    display.setCursor(2, 22);
    display.printf("W:%d(%d)", lastWiFiScanCount, uniqueWiFiCount);

    // BLE count: Last scan (Total)
    display.setCursor(2, 38);
    display.printf("B:%d(%d)", lastBLEScanCount, uniqueBLECount);

    // Countdown timer on right side (right-aligned with margin)
    unsigned long currentTime = millis();
    unsigned long timeSinceLastScan = currentTime - lastScan;
    unsigned long timeUntilNextScan = SCAN_INTERVAL - timeSinceLastScan;
    int secondsLeft = timeUntilNextScan / 1000;

    display.setTextSize(1);
    // Right align "Scan in:" - 8 chars * 6px = 48px from right edge (128-48-2=78)
    display.setCursor(70, 22);
    display.print("Scan in:");

    display.setTextSize(2);
    // Right align seconds - estimate width and position from right
    char secStr[4];
    snprintf(secStr, sizeof(secStr), "%ds", secondsLeft);
    int secWidth = strlen(secStr) * 12;  // 12px per char in size 2
    display.setCursor(SCREEN_WIDTH - secWidth - 2, 34);
    display.print(secStr);
  }

  // Bottom Left: GPS Time
  drawGPSTime();

  // Bottom Right: Speed
  drawSpeed();

  display.display();
}

void drawSatelliteIndicator() {
  // Draw satellite icon (simple dish shape)
  display.drawCircle(6, 6, 4, SSD1306_WHITE);
  display.drawLine(6, 10, 6, 12, SSD1306_WHITE);
  display.drawLine(3, 12, 9, 12, SSD1306_WHITE);

  // Draw signal bars based on satellite count
  int satCount = 0;
  if (gps.satellites.isValid()) {
    satCount = gps.satellites.value();
  }

  // Draw up to 5 bars
  int barHeight[] = {3, 5, 7, 9, 11};
  int maxBars = 5;
  int activeBars = 0;

  if (satCount >= 8) activeBars = 5;
  else if (satCount >= 6) activeBars = 4;
  else if (satCount >= 4) activeBars = 3;
  else if (satCount >= 2) activeBars = 2;
  else if (satCount >= 1) activeBars = 1;

  for (int i = 0; i < maxBars; i++) {
    int x = 14 + (i * 4);
    int y = 12 - barHeight[i];
    if (i < activeBars) {
      display.fillRect(x, y, 3, barHeight[i], SSD1306_WHITE);
    } else {
      display.drawRect(x, y, 3, barHeight[i], SSD1306_WHITE);
    }
  }

  // Display satellite count
  display.setTextSize(1);
  display.setCursor(14, 0);
  if (gps.satellites.isValid()) {
    display.printf("%d", satCount);
  } else {
    display.print("--");
  }
}

void drawCompass() {
  // Calculate compass direction from GPS course
  String direction = "---";
  int courseDegrees = 0;
  bool validCourse = false;

  if (gps.course.isValid() && gps.speed.isValid() && gps.speed.kmph() > 1.0) {
    double course = gps.course.deg();
    courseDegrees = (int)course;
    validCourse = true;

    // Convert degrees to compass direction
    if (course >= 337.5 || course < 22.5) direction = "N";
    else if (course >= 22.5 && course < 67.5) direction = "NE";
    else if (course >= 67.5 && course < 112.5) direction = "E";
    else if (course >= 112.5 && course < 157.5) direction = "SE";
    else if (course >= 157.5 && course < 202.5) direction = "S";
    else if (course >= 202.5 && course < 247.5) direction = "SW";
    else if (course >= 247.5 && course < 292.5) direction = "W";
    else if (course >= 292.5 && course < 337.5) direction = "NW";
  }

  // Draw degree heading (left side, size 1)
  display.setTextSize(1);
  if (validCourse) {
    char degStr[5];
    snprintf(degStr, sizeof(degStr), "%03d", courseDegrees);
    display.setCursor(SCREEN_WIDTH - 48, 0);  // Position before compass direction
    display.print(degStr);
    display.print((char)247);  // Degree symbol
  }

  // Draw compass direction on top right (size 2, right-aligned with 2px margin)
  display.setTextSize(2);
  int textWidth = direction.length() * 12;
  display.setCursor(SCREEN_WIDTH - textWidth - 2, 0);
  display.print(direction);
}

void drawGPSTime() {
  display.setTextSize(1);
  display.setCursor(2, SCREEN_HEIGHT - 8);  // 2px left margin

  if (gpsTimeValid && gps.time.isValid()) {
    char timeStr[12];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    display.print(timeStr);
  } else {
    display.print("--:--:--");
  }
}

void drawSpeed() {
  display.setTextSize(1);

  if (gps.speed.isValid()) {
    double mph = gps.speed.mph();
    double kph = gps.speed.kmph();

    char speedStr[20];
    snprintf(speedStr, sizeof(speedStr), "%.0fM %.0fK", mph, kph);

    int textWidth = strlen(speedStr) * 6;  // 6px per char in size 1
    display.setCursor(SCREEN_WIDTH - textWidth - 2, SCREEN_HEIGHT - 8);  // 2px right margin
    display.print(speedStr);
  } else {
    int textWidth = 5 * 6;  // "0M 0K" = 5 chars
    display.setCursor(SCREEN_WIDTH - textWidth - 2, SCREEN_HEIGHT - 8);  // 2px right margin
    display.print("0M 0K");
  }
}
