/*
 * SignalScout - WiFi, Bluetooth, and Zigbee Scanner for ESP32-C5
 * Scans 2.4GHz and 5GHz WiFi networks, Bluetooth LE devices, and Zigbee networks
 * Logs all activity to SD card over SPI with GPS timestamps and location
 * Displays status on SSD1309 OLED display
 *
 * Hardware: ESP32-C5 (16MB FLASH, 8MB PSRAM)
 * SD Card: Connected via SPI
 * GPS: NEO-6M connected via UART (TX/RX)
 * OLED: SSD1309 128x64 connected via SPI
 * RGB LED: WS2812B on GPIO27 for status indication
 * Battery: 3.7V LiPo with voltage divider on GPIO6
 *
 * Zigbee Note: Requires Arduino IDE settings:
 *   Tools -> Zigbee Mode -> Zigbee ED (End Device) [recommended for scanning]
 *   Tools -> Partition Scheme -> Zigbee with spiffs (size appropriate for your flash)
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
#include <Adafruit_NeoPixel.h>
#include <Zigbee.h>
#include <stdarg.h>

// SD Card SPI Pins (adjust these based on your wiring)
#define SD_CS    1  // Chip Select
#define SD_MOSI  0  // MOSI
#define SD_MISO  3  // MISO
#define SD_SCK   2  // SCK

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

// RGB LED (WS2812B) Pin
#define LED_PIN      27      // WS2812B data pin
#define LED_COUNT    1       // Number of LEDs

// BOOT Button Pin for Light Sleep
#define BOOT_BUTTON_PIN  28  // BOOT button GPIO
#define SLEEP_HOLD_TIME  3000  // Hold for 3 seconds to enter light sleep (ms)
#define WAKE_HOLD_TIME   1000  // Hold for 1 second to wake up (ms)

// Battery ADC Pin
#define BATTERY_PIN  6       // ADC pin for battery voltage
#define BATTERY_SAMPLES 10   // Number of ADC samples to average

// Battery voltage thresholds (3.7V LiPo)
// Voltage divider: 200k (positive side) / 100k (to ground) = divides by 3
// Full charge: 4.2V -> 1.4V at ADC
// Empty: 3.0V -> 1.0V at ADC
#define BATTERY_FULL_VOLTAGE 4.2
#define BATTERY_EMPTY_VOLTAGE 3.0
#define VOLTAGE_DIVIDER_RATIO 3.0  // For 200k/100k voltage divider

// Scan settings
#define SCAN_INTERVAL 4  // WiFi, BLE, and Zigbee scan every x seconds
#define BLE_SCAN_TIME 2      // BLE scan duration in seconds
#define ZIGBEE_SCAN_DURATION 2  // Zigbee scan duration (1-14, higher = longer)

// Zigbee settings
// Note: Requires Arduino IDE settings:
//   Tools -> Zigbee Mode -> Zigbee ZCZR (Coordinator/Router) [required for scanning]
//   Tools -> Partition Scheme -> Zigbee 4MB with spiffs (or appropriate for your flash size)
// ZCZR (Coordinator/Router) mode is required for network scanning
// The "Network steering was not successful" warning is normal and can be ignored
#define ENABLE_ZIGBEE_SCAN true  // Enable/disable Zigbee scanning

// Output enable/disable flags
#define ENABLE_CONSOLE_OUTPUT false   // Enable/disable serial console output
#define ENABLE_DISPLAY_OUTPUT true   // Enable/disable OLED display output
#define ENABLE_LOG_OUTPUT true       // Enable/disable SD card logging

// Timing variables
unsigned long lastScan = 0;
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_UPDATE_INTERVAL 1000  // Update display every 1000ms (1 second)

// FreeRTOS Task Handles
TaskHandle_t wifiScanTaskHandle = NULL;
TaskHandle_t bleScanTaskHandle = NULL;
TaskHandle_t zigbeeScanTaskHandle = NULL;
TaskHandle_t sdLogTaskHandle = NULL;

// FreeRTOS Semaphores and Mutexes
SemaphoreHandle_t deviceMapMutex;
SemaphoreHandle_t sdCardMutex;

// Queue for SD card logging (non-blocking)
QueueHandle_t logQueue;
#define LOG_QUEUE_SIZE 50

// Log entry structure for queue
struct LogEntry {
  char type[10];
  char fingerprint[9];
  char param1[128];
  char param2[64];
  char param3[32];
  char param4[128];
  char param5[128];
  char param6[32];
};

// Flag to indicate log file is ready
bool logFileReady = false;

// Device tracking
#include <map>
#include <string>
std::map<std::string, bool> seenWiFiDevices;    // Track unique WiFi devices by BSSID
std::map<std::string, bool> seenBLEDevices;     // Track unique BLE devices by address
std::map<std::string, bool> seenZigbeeNetworks; // Track unique Zigbee networks by Extended PAN ID
int uniqueWiFiCount = 0;
int uniqueBLECount = 0;
int uniqueZigbeeCount = 0;
int lastWiFiScanCount = 0;    // Devices found in last scan
int lastBLEScanCount = 0;     // Devices found in last scan
int lastZigbeeScanCount = 0;  // Networks found in last scan

// Zigbee initialization flag
bool zigbeeInitialized = false;

// GPS variables
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1
bool gpsTimeValid = false;

// RTC instance for persistent time
ESP32Time rtc;
bool rtcSyncedFromGPS = false;  // Track if RTC has been synced from GPS this session

// Dynamic log filename (generated at boot with timestamp)
String logFileName;

// SD card mount status
bool sdCardMounted = false;

// OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// RGB LED
Adafruit_NeoPixel statusLED(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Battery monitoring
int batteryPercent = 0;
unsigned long lastBatteryRead = 0;
#define BATTERY_READ_INTERVAL 5000  // Read battery every 5 seconds

// WiFi logo animation state (0 = dot only, 1-3 = number of rings)
int wifiAnimationState = 0;

// Deep sleep button state tracking
unsigned long buttonPressStart = 0;
bool buttonPressed = false;

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
  }
}

// LED color helper functions
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  statusLED.setPixelColor(0, statusLED.Color(r, g, b));
  statusLED.show();
}

void setLEDOff() {
  statusLED.setPixelColor(0, 0);
  statusLED.show();
}

// LED status indicators
void ledInitializing() {
  setLEDColor(255, 0, 0);  // Red: Initializing
}

void ledWaitingGPS() {
  setLEDColor(255, 165, 0);  // Orange: Waiting for GPS
}

void ledReady() {
  setLEDColor(0, 255, 0);  // Green: Setup complete, ready
}

// Read battery voltage and calculate percentage
int readBatteryPercent() {
  // Take multiple samples for stability
  long total = 0;
  for (int i = 0; i < BATTERY_SAMPLES; i++) {
    total += analogRead(BATTERY_PIN);
    delay(2);
  }
  int avgReading = total / BATTERY_SAMPLES;

  // Convert ADC reading to voltage
  // ESP32 ADC: 12-bit (0-4095), reference 3.3V
  float adcVoltage = (avgReading / 4095.0) * 3.3;

  // Calculate actual battery voltage (accounting for voltage divider)
  float batteryVoltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;

  // Calculate percentage
  int percent = (int)(((batteryVoltage - BATTERY_EMPTY_VOLTAGE) /
                       (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100);

  // Clamp to 0-100
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  return percent;
}

// Generate log filename with timestamp and create/open the file
bool generateLogFileName() {
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

  // Create the file and write header
  if (xSemaphoreTake(sdCardMutex, portMAX_DELAY) == pdTRUE) {
    File logFile = SD.open(logFileName.c_str(), FILE_WRITE);
    if (logFile) {
      logFile.println("=== SignalScout Log File ===");
      logFile.println("Format: TYPE,Fingerprint,Timestamp,Lat,Lon,Alt,Sats,HDOP,DeviceParams...");
      logFile.close();
      consolePrintf("Log file created successfully: %s\n", logFileName.c_str());
      xSemaphoreGive(sdCardMutex);
      return true;
    } else {
      consolePrintln("ERROR: Failed to create log file!");
      xSemaphoreGive(sdCardMutex);
      return false;
    }
  }
  return false;
}

// FreeRTOS Task: WiFi Scanner
void wifiScanTask(void* parameter) {
  vTaskDelay(pdMS_TO_TICKS(100));  // Initial stagger delay
  while (true) {
    if (logFileReady) {
      consolePrintf("\n[WiFi Task] Starting WiFi scan at %lu ms\n", millis());
      scanWiFi();
    }
    vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL * 1000));  // Convert to milliseconds
  }
}

// FreeRTOS Task: Bluetooth Scanner
void bleScanTask(void* parameter) {
  vTaskDelay(pdMS_TO_TICKS(300));  // Initial stagger delay (offset from WiFi)
  while (true) {
    if (logFileReady) {
      consolePrintf("\n[BLE Task] Starting BLE scan at %lu ms\n", millis());
      scanBluetooth();
    }
    vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL * 1000));  // Convert to milliseconds
  }
}

// FreeRTOS Task: Zigbee Scanner
void zigbeeScanTask(void* parameter) {
  vTaskDelay(pdMS_TO_TICKS(500));  // Initial stagger delay (offset from WiFi and BLE)
  while (true) {
    if (logFileReady && ENABLE_ZIGBEE_SCAN && zigbeeInitialized) {
      consolePrintf("\n[Zigbee Task] Starting Zigbee scan at %lu ms\n", millis());
      scanZigbee();
    }
    vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL * 1000));  // Convert to milliseconds
  }
}


// FreeRTOS Task: SD Card Logger (processes queue)
void sdLogTask(void* parameter) {
  LogEntry entry;
  while (true) {
    // Wait for log entries in the queue
    if (xQueueReceive(logQueue, &entry, portMAX_DELAY) == pdTRUE) {
      // Process the log entry
      if (ENABLE_LOG_OUTPUT && sdCardMounted && logFileReady) {
        if (xSemaphoreTake(sdCardMutex, portMAX_DELAY) == pdTRUE) {
          File logFile = SD.open(logFileName.c_str(), FILE_APPEND);
          if (logFile) {
            // Get timestamp and GPS data
            String timestamp;
            String gpsLat = "N/A";
            String gpsLon = "N/A";
            String gpsAlt = "N/A";
            String gpsSats = "N/A";
            String gpsHdop = "N/A";

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
              char dateTime[32];
              snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
                       rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
                       rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
              timestamp = String(dateTime) + " (RTC)";
            } else {
              timestamp = String(millis()) + "ms";
            }

            // Build log entry string
            String logStr = String(entry.type) + "," + String(entry.fingerprint) + "," +
                           timestamp + "," + gpsLat + "," + gpsLon + "," + gpsAlt + "," +
                           gpsSats + "," + gpsHdop + ",";

            if (String(entry.type) == "WIFI") {
              logStr += String(entry.param1) + "," + String(entry.param2) + "," +
                       String(entry.param3) + "," + String(entry.param4) + "," +
                       String(entry.param5) + "," + String(entry.param6);
            } else if (String(entry.type) == "BLE") {
              logStr += String(entry.param1) + "," + String(entry.param2) + "," +
                       String(entry.param3) + "," + String(entry.param4) + "," +
                       String(entry.param5);
            } else if (String(entry.type) == "ZIGBEE") {
              logStr += String(entry.param1) + "," + String(entry.param2) + "," +
                       String(entry.param3) + "," + String(entry.param4) + "," +
                       String(entry.param5) + "," + String(entry.param6);
            }

            logFile.println(logStr);
            logFile.close();
          } else {
            consolePrintln("ERROR: Failed to open log file in SD task");
          }
          xSemaphoreGive(sdCardMutex);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize status LED first
  statusLED.begin();
  statusLED.setBrightness(100);  // Set brightness (0-255)
  ledInitializing();  // Red: Starting initialization

  consolePrintln("\n\n=== BOOT START ===");
  consolePrintln("SignalScout - WiFi, Bluetooth & Zigbee Scanner with GPS");
  consolePrintln("=========================================================");

  // Create FreeRTOS mutexes and queue
  consolePrintln("\n[0/7] Initializing FreeRTOS resources...");
  deviceMapMutex = xSemaphoreCreateMutex();
  sdCardMutex = xSemaphoreCreateMutex();
  logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogEntry));

  if (deviceMapMutex == NULL || sdCardMutex == NULL || logQueue == NULL) {
    consolePrintln("ERROR: Failed to create FreeRTOS resources!");
    while(1);  // Halt
  }
  consolePrintln("FreeRTOS mutexes and queue created successfully");

  // Initialize BOOT button for light sleep
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // BOOT button (active LOW)
  consolePrintln("BOOT button initialized for light sleep (GPIO28)");

  // ============================================
  // CRITICAL: Initialize SD card FIRST before any radio initialization
  // The Zigbee/BLE radios can interfere with SPI if initialized first
  // ============================================
  consolePrintln("\n[1/7] Initializing SD card (SPI)...");
  if (ENABLE_LOG_OUTPUT) {
    consolePrintf("SD Card pins: CS=%d, MOSI=%d, MISO=%d, SCK=%d\n", SD_CS, SD_MOSI, SD_MISO, SD_SCK);

    // Initialize SPI bus for SD card
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    delay(250);  // Longer delay for SPI to stabilize

    // Try to mount SD card with retries at different speeds
    int retries = 5;
    uint32_t spiSpeeds[] = {1000000, 2000000, 4000000, 8000000};  // Try different speeds
    int speedIndex = 0;

    while (retries > 0 && !sdCardMounted) {
      uint32_t currentSpeed = spiSpeeds[speedIndex % 4];
      consolePrintf("Attempting SD card mount at %lu Hz (attempt %d/%d)...\n",
                    currentSpeed, (6 - retries), 5);

      if (SD.begin(SD_CS, SPI, currentSpeed, "/sd", 5, false)) {
        sdCardMounted = true;
        consolePrintf("SD Card mounted successfully at %lu Hz\n", currentSpeed);

        // Verify we can actually write to the card
        File testFile = SD.open("/test.tmp", FILE_WRITE);
        if (testFile) {
          testFile.println("test");
          testFile.close();
          SD.remove("/test.tmp");
          consolePrintln("SD Card write test passed");
        } else {
          consolePrintln("WARNING: SD Card write test failed");
          sdCardMounted = false;
          SD.end();
        }
      } else {
        retries--;
        speedIndex++;
        if (retries > 0) {
          consolePrintf("SD Card mount failed, retrying with different settings... (%d attempts left)\n", retries);
          SD.end();
          delay(500);
        }
      }
    }

    if (!sdCardMounted) {
      consolePrintln("ERROR: SD Card initialization failed after all retries!");
      consolePrintln("Check:");
      consolePrintln("  1) Card inserted and seated properly?");
      consolePrintln("  2) Card formatted as FAT32?");
      consolePrintln("  3) Wiring: CS->2, MOSI->3, MISO->1, SCK->0");
      consolePrintln("  4) Card not corrupted or damaged?");
      consolePrintln("Continuing without SD logging...");
    }
  } else {
    consolePrintln("SD logging disabled in config");
  }

  // Initialize battery ADC
  consolePrintln("\n[2/7] Initializing battery monitor...");
  analogReadResolution(12);  // 12-bit resolution
  analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
  batteryPercent = readBatteryPercent();
  consolePrintf("Battery level: %d%%\n", batteryPercent);

  // Check RTC time on boot
  consolePrintln("\n[3/7] Checking RTC time...");
  if (rtc.getYear() > 2020) {
    consolePrintf("RTC time found: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
                  rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
  } else {
    consolePrintln("No valid RTC time stored");
  }

  // Initialize OLED Display
  consolePrintln("\n[4/7] Initializing OLED display...");
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
  consolePrintln("\n[5/7] Initializing GPS...");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  consolePrintln("GPS UART initialized");

  // Initialize WiFi
  consolePrintln("\n[6/7] Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
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

  // Initialize Zigbee LAST (IEEE 802.15.4)
  // This must be after SD card to prevent SPI conflicts
  consolePrintln("\n[7/7] Initializing Zigbee (IEEE 802.15.4)...");
  if (ENABLE_ZIGBEE_SCAN) {
    consolePrintln("NOTE: You may see 'No Zigbee EPs to register' - this is NORMAL and expected");
    consolePrintln("      We are using Zigbee for scanning only, not as a device.");
    // Note: Requires Arduino IDE settings:
    //   Tools -> Zigbee Mode -> Zigbee ZCZR (Coordinator/Router)
    //   Tools -> Partition Scheme -> Zigbee 4MB with spiffs
    // Using ZIGBEE_ROUTER role for network scanning capability
    Zigbee.setRebootOpenNetwork(0);  // Don't open network on reboot
    if (Zigbee.begin(ZIGBEE_ROUTER, false)) {
      zigbeeInitialized = true;
      consolePrintln("Zigbee initialized successfully (Router mode for scanning)");
    } else {
      consolePrintln("WARNING: Zigbee initialization failed!");
      consolePrintln("Check Arduino IDE: Tools -> Zigbee Mode -> Zigbee ZCZR");
    }
  } else {
    consolePrintln("Zigbee scanning disabled in config");
  }

  // Wait for GPS signal before starting scans
  ledWaitingGPS();  // Orange: Waiting for GPS
  consolePrintln("\nWaiting for GPS signal...");

  // Initial display update (before tasks are started, no mutex needed)
  if (ENABLE_DISPLAY_OUTPUT) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Waiting for GPS");
    display.display();
  }

  unsigned long gpsWaitStart = millis();
  unsigned long lastGPSStatusUpdate = 0;
  int dotCount = 0;

  while (!gps.location.isValid() || !gps.time.isValid()) {
    // Read GPS data
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // Update display and console periodically
    if (millis() - lastGPSStatusUpdate > 1000) {
      lastGPSStatusUpdate = millis();
      dotCount = (dotCount + 1) % 4;

      // Build status string with dots
      String dots = "";
      for (int i = 0; i < dotCount; i++) dots += ".";

      int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      consolePrintf("Waiting for GPS fix%s (Satellites: %d)\n", dots.c_str(), sats);

      if (ENABLE_DISPLAY_OUTPUT) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Waiting for GPS");
        display.setCursor(0, 16);
        display.printf("Satellites: %d", sats);
        display.setCursor(0, 32);
        unsigned long elapsed = (millis() - gpsWaitStart) / 1000;
        display.printf("Elapsed: %lus", elapsed);

        // Show battery during GPS wait
        display.setCursor(0, 48);
        display.printf("Battery: %d%%", readBatteryPercent());

        display.display();
      }
    }

    delay(100);
  }

  // GPS signal acquired!
  gpsTimeValid = true;
  consolePrintln("\nGPS signal acquired!");
  displayGPSInfo();

  // Sync RTC from GPS
  rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
              gps.date.day(), gps.date.month(), gps.date.year());
  rtcSyncedFromGPS = true;
  consolePrintln("RTC synced from GPS time!");

  // Generate log filename now that we have valid time
  consolePrintln("\nCreating log file...");
  if (ENABLE_LOG_OUTPUT && sdCardMounted) {
    if (generateLogFileName()) {
      logFileReady = true;
      logToFile("System started - SignalScout initialized");
      logToFile("GPS signal acquired");
      consolePrintf("Logging to: %s\n", logFileName.c_str());
    } else {
      consolePrintln("ERROR: Failed to create log file!");
      logFileReady = false;
    }
  } else {
    consolePrintln("WARNING: SD card not mounted - logging disabled");
    logFileReady = false;
  }

  // Setup complete
  ledReady();  // Green: Ready
  consolePrintln("\nInitialization complete. Starting tasks...\n");

  // Check available heap before creating tasks
  consolePrintf("\nFree heap before task creation: %u bytes\n", ESP.getFreeHeap());
  consolePrintf("Largest free block: %u bytes\n", ESP.getMaxAllocHeap());

  // Create FreeRTOS tasks
  consolePrintln("\nCreating FreeRTOS tasks...");

  // SD Logger Task (only create if SD card is mounted and logging enabled)
  if (ENABLE_LOG_OUTPUT && sdCardMounted && logFileReady) {
    BaseType_t result = xTaskCreate(
      sdLogTask,           // Task function
      "SD Logger",         // Task name
      3072,                // Stack size (bytes) - reduced
      NULL,                // Parameters
      3,                   // Priority (0-24, higher = more priority)
      &sdLogTaskHandle     // Task handle
    );
    if (result != pdPASS) {
      consolePrintln("ERROR: Failed to create SD Logger task!");
    } else {
      consolePrintln("SD Logger task created");
    }
  } else {
    consolePrintln("SD Logger task skipped (SD card not ready)");
  }

  // WiFi Scan Task
  BaseType_t result = xTaskCreate(
    wifiScanTask,        // Task function
    "WiFi Scanner",      // Task name
    4096,                // Stack size (reduced from 8192)
    NULL,                // Parameters
    1,                   // Priority
    &wifiScanTaskHandle  // Task handle
  );
  if (result != pdPASS) {
    consolePrintln("ERROR: Failed to create WiFi Scanner task!");
  } else {
    consolePrintln("WiFi Scanner task created");
  }

  // BLE Scan Task
  result = xTaskCreate(
    bleScanTask,         // Task function
    "BLE Scanner",       // Task name
    4096,                // Stack size (reduced from 8192)
    NULL,                // Parameters
    1,                   // Priority
    &bleScanTaskHandle   // Task handle
  );
  if (result != pdPASS) {
    consolePrintln("ERROR: Failed to create BLE Scanner task!");
  } else {
    consolePrintln("BLE Scanner task created");
  }

  // Zigbee Scan Task
  if (ENABLE_ZIGBEE_SCAN && zigbeeInitialized) {
    result = xTaskCreate(
      zigbeeScanTask,        // Task function
      "Zigbee Scanner",      // Task name
      4096,                  // Stack size (reduced from 8192)
      NULL,                  // Parameters
      1,                     // Priority
      &zigbeeScanTaskHandle  // Task handle
    );
    if (result != pdPASS) {
      consolePrintln("ERROR: Failed to create Zigbee Scanner task!");
    } else {
      consolePrintln("Zigbee Scanner task created");
    }
  }

  consolePrintln("FreeRTOS task creation complete!");

  // Check heap after task creation
  consolePrintf("\nFree heap after task creation: %u bytes\n", ESP.getFreeHeap());
  consolePrintf("Minimum free heap ever: %u bytes\n", ESP.getMinFreeHeap());

  // Brief pause to show green LED
  delay(1000);

  // Turn off LED to conserve battery
  setLEDOff();
  consolePrintln("Status LED turned off to conserve battery");

  // Configure light sleep wakeup source (GPIO wakeup - BOOT button)
  // Wake when GPIO28 goes LOW (button pressed)
  gpio_wakeup_enable((gpio_num_t)BOOT_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  consolePrintln("Light sleep wakeup configured (hold BOOT button for 1 second to wake)");

  consolePrintln("\nEntering main loop (GPS monitoring, battery checks, and display updates)...\n");
}

void loop() {
  // Main loop handles GPS reading, battery monitoring, display updates, and light sleep
  // All scanning is handled by FreeRTOS tasks

  // Read GPS data continuously
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Periodically update RTC from GPS (every ~60 seconds) to keep it accurate
  if (gps.time.isValid() && gps.date.isValid()) {
    static unsigned long lastRTCSync = 0;
    if (millis() - lastRTCSync > 60000) {
      rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                  gps.date.day(), gps.date.month(), gps.date.year());
      lastRTCSync = millis();
    }
  }

  unsigned long currentTime = millis();

  // Update battery reading periodically
  if (currentTime - lastBatteryRead >= BATTERY_READ_INTERVAL) {
    batteryPercent = readBatteryPercent();
    lastBatteryRead = currentTime;
  }

  // Update display periodically
  if (ENABLE_DISPLAY_OUTPUT && currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay("");
    lastDisplayUpdate = currentTime;
  }

  // Check for light sleep button press (BOOT button on GPIO28)
  // Button is active LOW (pressed = LOW, released = HIGH)
  bool currentButtonState = (digitalRead(BOOT_BUTTON_PIN) == LOW);

  if (currentButtonState && !buttonPressed) {
    // Button just pressed, start timing
    buttonPressed = true;
    buttonPressStart = millis();
  } else if (currentButtonState && buttonPressed) {
    // Button is being held, check if held long enough for sleep
    unsigned long holdDuration = millis() - buttonPressStart;
    if (holdDuration >= SLEEP_HOLD_TIME) {
      // Enter light sleep
      enterLightSleep();
      // Note: After waking from light sleep, execution continues here
      // Reset button state after wake
      buttonPressed = false;
    }
  } else if (!currentButtonState && buttonPressed) {
    // Button released
    buttonPressed = false;
  }

  // Small delay to prevent tight loop
  delay(50);
}

void enterLightSleep() {
  consolePrintln("\n=== ENTERING LIGHT SLEEP ===");
  consolePrintln("Hold BOOT button for 1 second to wake up");

  // Show "Going to sleep" message on display
  if (ENABLE_DISPLAY_OUTPUT) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.println("Going to");
    display.setCursor(10, 40);
    display.println("sleep...");
    display.display();
    delay(2000);  // Show message for 2 seconds

    // Clear display before sleep
    display.clearDisplay();
    display.display();
  }

  // Log to SD card
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Entering light sleep mode");
  }

  // Turn off status LED
  setLEDOff();

  // Flush serial output
  Serial.flush();

  // Enter light sleep (wake on BOOT button press - GPIO wakeup already configured in setup)
  esp_light_sleep_start();

  // Execution continues here after waking from light sleep
  consolePrintln("\n=== WAKING FROM LIGHT SLEEP ===");

  // Log wake event
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Waking from light sleep mode");
  }

  // Re-enable LED briefly to show wake
  setLEDColor(0, 255, 0);  // Green
  delay(500);
  setLEDOff();

  // Update display
  if (ENABLE_DISPLAY_OUTPUT) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 20);
    display.println("Awake!");
    display.display();
    delay(1000);
  }
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

    // Track unique devices by BSSID (with mutex protection)
    if (xSemaphoreTake(deviceMapMutex, portMAX_DELAY) == pdTRUE) {
      std::string bssidKey(bssidStr);
      if (seenWiFiDevices.find(bssidKey) == seenWiFiDevices.end()) {
        seenWiFiDevices[bssidKey] = true;
        uniqueWiFiCount++;
      }
      xSemaphoreGive(deviceMapMutex);
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

    // Queue log entry for SD card writing (non-blocking)
    if (ENABLE_LOG_OUTPUT && logFileReady) {
      LogEntry entry;
      strncpy(entry.type, "WIFI", sizeof(entry.type) - 1);
      strncpy(entry.fingerprint, fingerprintStr, sizeof(entry.fingerprint) - 1);
      strncpy(entry.param1, ssid.c_str(), sizeof(entry.param1) - 1);
      strncpy(entry.param2, bssidStr, sizeof(entry.param2) - 1);
      snprintf(entry.param3, sizeof(entry.param3), "%d", rssi);
      snprintf(entry.param4, sizeof(entry.param4), "%d", channel);
      strncpy(entry.param5, band.c_str(), sizeof(entry.param5) - 1);
      strncpy(entry.param6, encType.c_str(), sizeof(entry.param6) - 1);

      // Try to add to queue (don't block if queue is full)
      if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        consolePrintln("WARNING: Log queue full, dropping WiFi entry");
      }
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

    // Track unique devices by address (with mutex protection)
    if (xSemaphoreTake(deviceMapMutex, portMAX_DELAY) == pdTRUE) {
      std::string addrKey = address.c_str();
      if (seenBLEDevices.find(addrKey) == seenBLEDevices.end()) {
        seenBLEDevices[addrKey] = true;
        uniqueBLECount++;
      }
      xSemaphoreGive(deviceMapMutex);
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

    // Queue log entry for SD card writing (non-blocking)
    if (ENABLE_LOG_OUTPUT && logFileReady) {
      LogEntry entry;
      strncpy(entry.type, "BLE", sizeof(entry.type) - 1);
      strncpy(entry.fingerprint, fingerprintStr, sizeof(entry.fingerprint) - 1);
      strncpy(entry.param1, name.c_str(), sizeof(entry.param1) - 1);
      strncpy(entry.param2, address.c_str(), sizeof(entry.param2) - 1);
      snprintf(entry.param3, sizeof(entry.param3), "%d", rssi);
      strncpy(entry.param4, manufData.c_str(), sizeof(entry.param4) - 1);
      strncpy(entry.param5, serviceUUID.c_str(), sizeof(entry.param5) - 1);
      entry.param6[0] = '\0';  // Not used for BLE

      // Try to add to queue (don't block if queue is full)
      if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        consolePrintln("WARNING: Log queue full, dropping BLE entry");
      }
    }
  }

  pBLEScan->clearResults();
  consolePrintln("--- Bluetooth Scan Complete ---\n");
}

void scanZigbee() {
  consolePrintln("\n--- Zigbee Scan Starting (IEEE 802.15.4) ---");

  // Note: "Network steering was not successful" warning is normal for scanning-only mode
  // We're not trying to join a network, just scanning for nearby networks

  // Start network scan on all Zigbee channels (11-26)
  // This sends beacon requests and listens for responses
  Zigbee.scanNetworks();

  // Wait for scan to complete
  int16_t scanStatus;
  unsigned long scanStart = millis();
  unsigned long timeout = 30000;  // 30 second timeout

  do {
    scanStatus = Zigbee.scanComplete();
    delay(100);

    // Check for timeout
    if (millis() - scanStart > timeout) {
      consolePrintln("Zigbee scan timeout");
      lastZigbeeScanCount = 0;
      return;
    }
  } while (scanStatus == ZB_SCAN_RUNNING);

  // Check scan result
  if (scanStatus == ZB_SCAN_FAILED) {
    consolePrintln("Zigbee scan failed");
    lastZigbeeScanCount = 0;
    return;
  }

  int networkCount = scanStatus;
  lastZigbeeScanCount = networkCount;

  consolePrintf("Zigbee networks found: %d\n", networkCount);

  if (networkCount == 0) {
    consolePrintln("No Zigbee networks found");
    return;
  }

  // Get pointer to scan results array
  zigbee_scan_result_t* scanResults = Zigbee.getScanResult();
  if (scanResults == NULL) {
    consolePrintln("Failed to get scan results");
    return;
  }

  // Process each network found
  for (int i = 0; i < networkCount; i++) {
    // Access result at index i
    zigbee_scan_result_t* network = &scanResults[i];

    // Format Extended PAN ID (8 bytes, reversed order)
    char extPanIdStr[24];
    snprintf(extPanIdStr, sizeof(extPanIdStr), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             network->extended_pan_id[7], network->extended_pan_id[6],
             network->extended_pan_id[5], network->extended_pan_id[4],
             network->extended_pan_id[3], network->extended_pan_id[2],
             network->extended_pan_id[1], network->extended_pan_id[0]);

    // Format PAN ID (2 bytes) - use short_pan_id field
    char panIdStr[8];
    snprintf(panIdStr, sizeof(panIdStr), "0x%04X", network->short_pan_id);

    // Track unique networks by Extended PAN ID (with mutex protection)
    if (xSemaphoreTake(deviceMapMutex, portMAX_DELAY) == pdTRUE) {
      std::string extPanIdKey(extPanIdStr);
      if (seenZigbeeNetworks.find(extPanIdKey) == seenZigbeeNetworks.end()) {
        seenZigbeeNetworks[extPanIdKey] = true;
        uniqueZigbeeCount++;
      }
      xSemaphoreGive(deviceMapMutex);
    }

    // Get channel number
    uint8_t channel = network->logic_channel;

    // Get network characteristics
    bool permitJoining = network->permit_joining;
    bool routerCapacity = network->router_capacity;
    bool endDeviceCapacity = network->end_device_capacity;

    // Generate fingerprint from Extended PAN ID and PAN ID
    String fingerprintData = String(extPanIdStr) + String(panIdStr);
    uint32_t fingerprint = 0;
    for (unsigned int j = 0; j < fingerprintData.length(); j++) {
      fingerprint = fingerprint * 31 + fingerprintData.charAt(j);
    }
    char fingerprintStr[9];
    snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);

    // Print to Serial
    consolePrintf("%d: PAN: %s | ExtPAN: %s | Ch: %d | Join: %s | FP: %s\n",
                  i + 1, panIdStr, extPanIdStr, channel,
                  permitJoining ? "Yes" : "No", fingerprintStr);
    consolePrintf("   Router Cap: %s | End Device Cap: %s\n",
                  routerCapacity ? "Yes" : "No",
                  endDeviceCapacity ? "Yes" : "No");

    // Queue log entry for SD card writing (non-blocking)
    if (ENABLE_LOG_OUTPUT && logFileReady) {
      LogEntry entry;
      strncpy(entry.type, "ZIGBEE", sizeof(entry.type) - 1);
      strncpy(entry.fingerprint, fingerprintStr, sizeof(entry.fingerprint) - 1);
      strncpy(entry.param1, panIdStr, sizeof(entry.param1) - 1);
      strncpy(entry.param2, extPanIdStr, sizeof(entry.param2) - 1);
      snprintf(entry.param3, sizeof(entry.param3), "%d", channel);
      strncpy(entry.param4, permitJoining ? "Yes" : "No", sizeof(entry.param4) - 1);
      strncpy(entry.param5, routerCapacity ? "Yes" : "No", sizeof(entry.param5) - 1);
      strncpy(entry.param6, endDeviceCapacity ? "Yes" : "No", sizeof(entry.param6) - 1);

      // Try to add to queue (don't block if queue is full)
      if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        consolePrintln("WARNING: Log queue full, dropping Zigbee entry");
      }
    }
  }

  // Clear scan results to free memory
  Zigbee.scanDelete();

  consolePrintln("--- Zigbee Scan Complete ---\n");
}

void logToFile(String message) {
  if (!ENABLE_LOG_OUTPUT || !sdCardMounted || !logFileReady) return;

  if (xSemaphoreTake(sdCardMutex, portMAX_DELAY) == pdTRUE) {
    File logFile = SD.open(logFileName.c_str(), FILE_APPEND);

    if (!logFile) {
      consolePrintln("ERROR: Failed to open log file for writing");
      xSemaphoreGive(sdCardMutex);
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

    xSemaphoreGive(sdCardMutex);

    // Also print errors to serial for debugging
    if (message.startsWith("ERROR")) {
      consolePrintln(logEntry.c_str());
    }
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

  // Top Center: Battery Indicator
  drawBatteryIndicator();

  // Top Right: Compass Direction
  drawCompass();

  // Center: Status Message, Countdown Timer, or Device Stats
  if (statusMessage.length() > 0 || !gpsTimeValid) {
    display.setTextSize(1);
    display.setCursor(0, 24);
    if (statusMessage.length() > 0) {
      display.println(statusMessage);
    } 
  } else {
    // Show device statistics and countdown
    display.setTextSize(1);

    // Read device counts with mutex protection
    int wifi_last, ble_last, zigbee_last;
    int wifi_total, ble_total, zigbee_total;

    if (xSemaphoreTake(deviceMapMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      wifi_last = lastWiFiScanCount;
      ble_last = lastBLEScanCount;
      zigbee_last = lastZigbeeScanCount;
      wifi_total = uniqueWiFiCount;
      ble_total = uniqueBLECount;
      zigbee_total = uniqueZigbeeCount;
      xSemaphoreGive(deviceMapMutex);
    } else {
      // Mutex timeout, use previous values
      wifi_last = ble_last = zigbee_last = 0;
      wifi_total = ble_total = zigbee_total = 0;
    }

    // WiFi count: Last scan (Total)
    display.setCursor(2, 18);
    display.printf("W:%d(%d)", wifi_last, wifi_total);

    // BLE count: Last scan (Total)
    display.setCursor(2, 28);
    display.printf("B:%d(%d)", ble_last, ble_total);

    // Zigbee count: Last scan (Total)
    display.setCursor(2, 38);
    display.printf("Z:%d(%d)", zigbee_last, zigbee_total);

    // Draw animated WiFi logo in center-right of screen
    drawWiFiLogo(85, 18);

    // Advance animation state for next frame
    wifiAnimationState = (wifiAnimationState + 1) % 4;
  }

  // Bottom Left: GPS Time
  drawGPSTime();

  // Bottom Right: Speed
  drawSpeed();

  display.display();
}

void drawSatelliteIndicator() {
  // Draw satellite icon (simple dish shape)
  //display.drawCircle(6, 6, 4, SSD1306_WHITE);
  //display.drawLine(6, 10, 6, 12, SSD1306_WHITE);
  //display.drawLine(3, 12, 9, 12, SSD1306_WHITE);

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
    int x = 20 + (i * 4);
    int y = 12 - barHeight[i];
    if (i < activeBars) {
      display.fillRect(x, y, 3, barHeight[i], SSD1306_WHITE);
    } else {
      display.drawRect(x, y, 3, barHeight[i], SSD1306_WHITE);
    }
  }

  // Display satellite count
  display.setTextSize(1);
  display.setCursor(2, 0);
  if (gps.satellites.isValid()) {
    display.printf("S:%d", satCount);
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
  //display.setTextSize(1);
  //if (validCourse) {
  //  char degStr[5];
  //  snprintf(degStr, sizeof(degStr), "%03d", courseDegrees);
  //  display.setCursor(SCREEN_WIDTH - 48, 0);  // Position before compass direction
  //  display.print(degStr);
  //  display.print((char)247);  // Degree symbol
  //}

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

    char speedStr[24];
    snprintf(speedStr, sizeof(speedStr), "%.0fMPH %.0fKPH", mph, kph);

    int textWidth = strlen(speedStr) * 6;  // 6px per char in size 1
    display.setCursor(SCREEN_WIDTH - textWidth - 2, SCREEN_HEIGHT - 8);  // 2px right margin
    display.print(speedStr);
  } else {
    int textWidth = 5 * 6;  // "0M 0K" = 5 chars
    display.setCursor(SCREEN_WIDTH - textWidth - 2, SCREEN_HEIGHT - 8);  // 2px right margin
    display.print("0M 0K");
  }
}

void drawBatteryIndicator() {
  // Battery icon dimensions
  int battX = 50;  // X position (between satellite and compass)
  int battY = 2;   // Y position
  int battWidth = 16;
  int battHeight = 8;
  int tipWidth = 2;
  int tipHeight = 4;

  // Draw battery outline
  display.drawRect(battX, battY, battWidth, battHeight, SSD1306_WHITE);

  // Draw battery tip (positive terminal)
  display.fillRect(battX + battWidth, battY + (battHeight - tipHeight) / 2,
                   tipWidth, tipHeight, SSD1306_WHITE);

  // Calculate fill width based on percentage
  int fillWidth = ((battWidth - 2) * batteryPercent) / 100;
  if (fillWidth < 0) fillWidth = 0;
  if (fillWidth > battWidth - 2) fillWidth = battWidth - 2;

  // Fill battery based on charge level
  if (fillWidth > 0) {
    display.fillRect(battX + 1, battY + 1, fillWidth, battHeight - 2, SSD1306_WHITE);
  }

  // Draw percentage text below battery if space allows, or to the side
  // Using small font next to battery
  display.setTextSize(1);
  display.setCursor(battX + battWidth + tipWidth + 2, battY);
  display.printf("%d", batteryPercent);
}

void drawWiFiLogo(int x, int y) {
  // Animated WiFi icon - signal arcs radiating upward
  // wifiAnimationState: 0 = dot only, 1 = 1 ring, 2 = 2 rings, 3 = 3 rings
  int cx = x + 16;  // Center x
  int cy = y + 26;  // Bottom point (where dot is)

  // Always draw center dot
  display.fillCircle(cx, cy, 3, SSD1306_WHITE);

  // Draw rings based on animation state
  int ringRadii[] = {8, 15, 22};  // Radii for the 3 rings

  for (int ring = 0; ring < wifiAnimationState && ring < 3; ring++) {
    int r = ringRadii[ring];
    // Draw arc from -50 to +50 degrees
    for (int angle = -55; angle <= 55; angle += 2) {
      float rad = angle * 3.14159 / 180.0;
      int px = cx + (int)(r * sin(rad));
      int py = cy - (int)(r * cos(rad));
      display.drawPixel(px, py, SSD1306_WHITE);
      // Make arcs thicker (draw adjacent pixels)
      display.drawPixel(px, py - 1, SSD1306_WHITE);
      display.drawPixel(px + 1, py, SSD1306_WHITE);
    }
  }
}
