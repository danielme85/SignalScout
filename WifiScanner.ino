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
#include <esp_wifi.h>
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
#include <ESP32Time.h>
#include <stdarg.h>

// SD Card SPI Pins (adjust these based on your wiring)
#define SD_CS    2   // Chip Select
#define SD_MOSI  3  // MOSI
#define SD_MISO  1  // MISO
#define SD_SCK   0  // SCK

// GPS UART Pins (adjust these based on your wiring)
#define GPS_RX   14  // ESP32 RX pin (connects to GPS TX)
#define GPS_TX   13  // ESP32 TX pin (connects to GPS RX)
#define GPS_BAUD 9600

// OLED Display SPI Pins (adjust these based on your wiring)
#define OLED_MOSI  26  // Can share with SD card MOSI
#define OLED_CLK   25  // Can share with SD card SCK
#define OLED_DC    9   // Data/Command pin
#define OLED_CS    8  // Chip Select (different from SD card)
#define OLED_RESET 10   // Reset pin
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Scan settings
#define SCAN_INTERVAL 10000  // WiFi and BLE scan every 10 seconds
#define BLE_SCAN_TIME 5      // BLE scan duration in seconds

// Output enable/disable flags
#define ENABLE_CONSOLE_OUTPUT false   // Enable/disable serial console output
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

// RTC instance for persistent time
ESP32Time rtc;
bool rtcSyncedFromGPS = false;  // Track if RTC has been synced from GPS this session

// Dynamic log filename (generated at boot with timestamp)
String logFileName;

// OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

BLEScan* pBLEScan;

// BLE Scan callback class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // This will be called for each BLE device found during scan
  }
};

// Helper function for console output
void consolePrint(const char* msg) {
  if (ENABLE_CONSOLE_OUTPUT) {
    Serial.print(msg);
  }
}

void consolePrintln(const char* msg) {
  if (ENABLE_CONSOLE_OUTPUT) {
    Serial.println(msg);
    Serial.flush();
  }
}

void consolePrintf(const char* format, ...) {
  if (ENABLE_CONSOLE_OUTPUT) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
    Serial.flush();
  }
}

// Generate log filename with timestamp
void generateLogFileName() {
  // Check if RTC has a valid time (year > 2020 indicates valid time)
  if (rtc.getYear() > 2020) {
    // Use RTC time for filename
    char filename[32];
    snprintf(filename, sizeof(filename), "/scan_%04d%02d%02d_%02d%02d%02d.txt",
             rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    logFileName = String(filename);
    consolePrintf("Using RTC time for log file: %s\n", filename);
  } else {
    // No valid RTC time, use boot timestamp
    char filename[32];
    snprintf(filename, sizeof(filename), "/scan_boot_%lu.txt", millis());
    logFileName = String(filename);
    consolePrintf("No RTC time available, using boot timestamp: %s\n", filename);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  consolePrintln("\n\n=== BOOT START ===");
  consolePrintln("SignalScout - WiFi & Bluetooth Scanner with GPS");
  consolePrintln("================================================");

  // Check RTC time on boot
  consolePrintln("Checking RTC time...");
  if (rtc.getYear() > 2020) {
    consolePrintf("RTC time found: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
                  rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    // RTC has valid time, we can use it immediately
    gpsTimeValid = true;  // Allow logging with RTC time until GPS syncs
  } else {
    consolePrintln("No valid RTC time stored");
  }

  // Initialize OLED Display
  consolePrintln("Initializing OLED display...");
  if (ENABLE_DISPLAY_OUTPUT) {
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
      consolePrintln("ERROR: SSD1306 allocation failed!");
      consolePrintln("Continuing without display...");
    } else {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Initializing...");
      display.display();
      consolePrintln("Display initialized");
    }
  } else {
    consolePrintln("Display disabled in config");
  }

  // Initialize GPS
  consolePrintln("Initializing GPS...");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  consolePrintln("GPS initialized. Waiting for GPS fix...");
  if (ENABLE_DISPLAY_OUTPUT) updateDisplay("Waiting for GPS");

  // Initialize SD card
  consolePrintln("Initializing SD card...");
  if (ENABLE_LOG_OUTPUT) {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
      consolePrintln("ERROR: SD Card initialization failed!");
      consolePrintln("Check SD card and wiring. Continuing without SD logging...");
    } else {
      consolePrintln("SD Card initialized successfully");
      // Generate unique filename based on RTC or boot time
      generateLogFileName();
      logToFile("System started - SignalScout initialized");
    }
  } else {
    consolePrintln("SD logging disabled in config");
  }

  // Set WiFi to station mode
  consolePrintln("Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Note: We use ESP-IDF scan APIs directly to support dual-band (2.4GHz + 5GHz) scanning
  // The Arduino WiFi library has issues with WIFI_BAND_MODE_AUTO on ESP32-C5
  consolePrintln("WiFi initialized (dual-band 2.4GHz + 5GHz mode)");

  // Initialize BLE
  consolePrintln("Initializing BLE...");
  BLEDevice::init("SignalScout");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  consolePrintln("BLE initialized");

  consolePrintln("\nInitialization complete. Starting scans...\n");
  if (ENABLE_LOG_OUTPUT) logToFile("Initialization complete");
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Check if we have a valid GPS time fix and sync RTC
  if (gps.time.isValid() && gps.date.isValid()) {
    // Sync RTC from GPS if not already synced this session
    if (!rtcSyncedFromGPS) {
      // Set RTC time from GPS
      rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                  gps.date.day(), gps.date.month(), gps.date.year());
      rtcSyncedFromGPS = true;
      consolePrintln("RTC synced from GPS time!");
      consolePrintf("RTC set to: %04d-%02d-%02d %02d:%02d:%02d\n",
                    rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
                    rtc.getHour(true), rtc.getMinute(), rtc.getSecond());

      // If this is first GPS lock (not using stored RTC), announce it
      if (!gpsTimeValid) {
        gpsTimeValid = true;
        consolePrintln("GPS time lock acquired!");
        if (ENABLE_LOG_OUTPUT) logToFile("GPS time lock acquired");
        displayGPSInfo();
      }
    } else {
      // Periodically update RTC from GPS (every ~60 seconds) to keep it accurate
      static unsigned long lastRTCSync = 0;
      if (millis() - lastRTCSync > 60000) {
        rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                    gps.date.day(), gps.date.month(), gps.date.year());
        lastRTCSync = millis();
      }
    }
  }

  unsigned long currentTime = millis();

  // Update display
  if (ENABLE_DISPLAY_OUTPUT && currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay("");
    lastDisplayUpdate = currentTime;
  }

  // Perform WiFi and BLE scans together
  if (currentTime - lastScan >= SCAN_INTERVAL) {
    consolePrintf("\n[%lu] Starting scan cycle...\n", millis());
    scanWiFi();
    scanBluetooth();
    lastScan = currentTime;
    consolePrintf("[%lu] Scan cycle complete\n", millis());
  }

  delay(100);
}

// Helper function to convert auth mode to string
String getAuthModeString(wifi_auth_mode_t authMode) {
  switch (authMode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK: return "WPA3-PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3-PSK";
    default: return "UNKNOWN";
  }
}

void scanWiFi() {
  consolePrintln("\n--- WiFi Scan Starting (2.4GHz + 5GHz) ---");

  // Use ESP-IDF scan API directly to avoid WIFI_BAND_MODE_AUTO issues
  wifi_scan_config_t scan_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,  // 0 = scan all channels
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = {
      .active = {
        .min = 100,
        .max = 300
      },
      .passive = 300
    }
  };

  // Start scan (blocking mode)
  esp_err_t err = esp_wifi_scan_start(&scan_config, true);
  if (err != ESP_OK) {
    consolePrintf("WiFi scan failed to start: %d\n", err);
    lastWiFiScanCount = 0;
    return;
  }

  // Get number of APs found
  uint16_t ap_count = 0;
  esp_wifi_scan_get_ap_num(&ap_count);
  lastWiFiScanCount = ap_count;

  consolePrintf("Networks found: %d\n", ap_count);

  if (ap_count == 0) {
    consolePrintln("No networks found");
    return;
  }

  // Allocate memory for AP records
  wifi_ap_record_t* ap_records = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * ap_count);
  if (ap_records == NULL) {
    consolePrintln("Failed to allocate memory for scan results");
    return;
  }

  // Get AP records
  err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
  if (err != ESP_OK) {
    consolePrintf("Failed to get scan records: %d\n", err);
    free(ap_records);
    return;
  }

  // Process each AP
  for (int i = 0; i < ap_count; i++) {
    wifi_ap_record_t* ap = &ap_records[i];

    // Get SSID (may be empty for hidden networks)
    String ssid = String((char*)ap->ssid);
    if (ssid.length() == 0) {
      ssid = "<hidden>";
    }

    int32_t rssi = ap->rssi;
    int32_t channel = ap->primary;
    wifi_auth_mode_t authMode = ap->authmode;

    // Format BSSID
    char bssidStr[18];
    snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             ap->bssid[0], ap->bssid[1], ap->bssid[2],
             ap->bssid[3], ap->bssid[4], ap->bssid[5]);

    // Track unique devices by BSSID
    std::string bssidKey(bssidStr);
    if (seenWiFiDevices.find(bssidKey) == seenWiFiDevices.end()) {
      seenWiFiDevices[bssidKey] = true;
      uniqueWiFiCount++;
    }

    // Determine frequency band based on channel
    // 2.4GHz: channels 1-14
    // 5GHz: channels 32-177 (varies by region)
    String band;
    if (channel <= 14) {
      band = "2.4GHz";
    } else {
      band = "5GHz";
    }

    // Get encryption type string
    String encType = getAuthModeString(authMode);

    // Generate fingerprint (simple hash)
    String fingerprintData = String(bssidStr) + ssid + encType;
    uint32_t fingerprint = 0;
    for (unsigned int j = 0; j < fingerprintData.length(); j++) {
      fingerprint = fingerprint * 31 + fingerprintData.charAt(j);
    }
    char fingerprintStr[9];
    snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);

    // Print to Serial
    consolePrintf("%d: %s (%s) | RSSI: %d dBm | Ch: %d (%s) | Enc: %s | FP: %s\n",
                  i + 1, ssid.c_str(), bssidStr, rssi, channel, band.c_str(),
                  encType.c_str(), fingerprintStr);

    // Log to SD card - one line per device with GPS data
    if (ENABLE_LOG_OUTPUT) {
      logDeviceToFile("WIFI", ssid, String(bssidStr), String(rssi),
                      String(channel), band, encType, fingerprintStr);
    }
  }

  // Free allocated memory
  free(ap_records);

  consolePrintln("--- WiFi Scan Complete ---\n");
}

void scanBluetooth() {
  consolePrintln("\n--- Bluetooth Scan Starting ---");

  BLEScanResults* foundDevices = pBLEScan->start(BLE_SCAN_TIME, false);
  int deviceCount = foundDevices->getCount();
  lastBLEScanCount = deviceCount;  // Track last scan count

  consolePrintf("Bluetooth devices found: %d\n", deviceCount);

  for (int i = 0; i < deviceCount; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);

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
    consolePrintf("%d: %s | RSSI: %d dBm | Addr: %s | FP: %s\n",
                  i + 1, name.c_str(), rssi, address.c_str(), fingerprintStr);

    if (manufData.length() > 0) {
      consolePrintf("   Manufacturer Data: %s\n", manufData.c_str());
    }
    if (serviceUUID.length() > 0) {
      consolePrintf("   Service UUID: %s\n", serviceUUID.c_str());
    }

    // Log to SD card - one line per device with GPS data
    if (ENABLE_LOG_OUTPUT) {
      logDeviceToFile("BLE", name, address, String(rssi),
                      manufData, serviceUUID, "", fingerprintStr);
    }
  }

  pBLEScan->clearResults();
  consolePrintln("--- Bluetooth Scan Complete ---\n");
}

void logDeviceToFile(String deviceType, String param1, String param2, String param3,
                     String param4, String param5, String param6, String fingerprint) {
  if (!ENABLE_LOG_OUTPUT) return;

  File logFile = SD.open(logFileName.c_str(), FILE_APPEND);

  if (!logFile) {
    consolePrintln("ERROR: Failed to open log file for writing");
    return;
  }

  String timestamp;
  String gpsLat = "N/A";
  String gpsLon = "N/A";
  String gpsAlt = "N/A";
  String gpsSats = "N/A";
  String gpsHdop = "N/A";

  // Get GPS time and location if available
  if (gps.time.isValid() && gps.date.isValid()) {
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
  } else if (rtc.getYear() > 2020) {
    // Fallback to RTC time if GPS not available
    char dateTime[32];
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
             rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    timestamp = String(dateTime) + " (RTC)";
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

  File logFile = SD.open(logFileName.c_str(), FILE_APPEND);

  if (!logFile) {
    consolePrintln("ERROR: Failed to open log file for writing");
    return;
  }

  String timestamp;
  String gpsData;

  // Use GPS time if available
  if (gps.time.isValid() && gps.date.isValid()) {
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
  } else if (rtc.getYear() > 2020) {
    // Fallback to RTC time
    char dateTime[32];
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
             rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    timestamp = String(dateTime) + " (RTC)";
    gpsData = " | GPS: No fix";
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
  if (message.startsWith("ERROR")) {
    consolePrintln(logEntry.c_str());
  }
}

void displayGPSInfo() {
  consolePrintln("\n--- GPS Information ---");

  if (gps.location.isValid()) {
    consolePrintf("Location: Lat: %.6f, Lon: %.6f\n",
                  gps.location.lat(), gps.location.lng());
  } else {
    consolePrintln("Location: INVALID");
  }

  if (gps.altitude.isValid()) {
    consolePrintf("Altitude: %.2f meters\n", gps.altitude.meters());
  } else {
    consolePrintln("Altitude: INVALID");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    consolePrintf("Date/Time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    consolePrintln("Date/Time: INVALID");
  }

  if (gps.satellites.isValid()) {
    consolePrintf("Satellites: %d\n", gps.satellites.value());
  } else {
    consolePrintln("Satellites: INVALID");
  }

  if (gps.hdop.isValid()) {
    consolePrintf("HDOP: %.2f\n", gps.hdop.hdop());
  }

  if (gps.speed.isValid()) {
    consolePrintf("Speed: %.2f km/h\n", gps.speed.kmph());
  }

  consolePrintln("--- End GPS Info ---\n");
}

void updateDisplay(String statusMessage) {
  display.clearDisplay();

  // Top Left: Satellite Signal Strength Indicator
  drawSatelliteIndicator();

  // Top Right: Compass Direction
  drawCompass();

  // Center: Status Message, Countdown Timer, or Device Stats
  if (statusMessage.length() > 0 || !gpsTimeValid) {
    display.setTextSize(1);
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

  if (gps.time.isValid()) {
    // Use GPS time when available
    char timeStr[12];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    display.print(timeStr);
  } else if (rtc.getYear() > 2020) {
    // Fallback to RTC time
    char timeStr[12];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
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
