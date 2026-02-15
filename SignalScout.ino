/*
 * SignalScout - WiFi and Bluetooth Scanner for ESP32-C5
 * Scans 2.4GHz and 5GHz WiFi networks and Bluetooth LE devices
 * Logs all activity to SD card over SPI with GPS timestamps and location
 * Displays status on SSD1309 OLED display
 *
 * Hardware: ESP32-C5 (16MB FLASH, 8MB PSRAM)
 * SD Card: Connected via SPI
 * GPS: NEO-6M connected via UART (TX/RX)
 * OLED: SSD1309 128x64 connected via SPI
 * RGB LED: WS2812B on GPIO27 for status indication
 * Battery: 3.7V LiPo with voltage divider on GPIO6
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
#include <stdarg.h>
#include "secrets.h"
// File sharing via WiFi (install ESPAsyncWebServer and AsyncTCP libraries)
#include <ESPAsyncWebServer.h>

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

// Multi-function Button Pin (File Share / Deep Sleep)
#define SHARE_BUTTON_PIN  23    // GPIO for file share / sleep button
#define SHARE_HOLD_TIME   1000  // Hold for 1 second to toggle file sharing (ms)
#define SLEEP_HOLD_TIME   3000  // Hold for 3 seconds to enter deep sleep (ms)

// Battery ADC Pin
#define BATTERY_PIN  6       // ADC pin for battery voltage
#define BATTERY_SAMPLES 10   // Number of ADC samples to average

// Battery voltage thresholds (3.7V LiPo)
// Voltage divider: 200k (positive side) / 100k (to ground) = divides by 3.333333
// Full charge: 3.7V -> 1.4V at ADC
// Empty: 3.0V -> 1.0V at ADC
#define BATTERY_FULL_VOLTAGE  3.7
#define BATTERY_EMPTY_VOLTAGE 3.0
#define VOLTAGE_DIVIDER_RATIO 3.333333

// Scan settings
#define SCAN_INTERVAL 10  // WiFi and BLE scan every x seconds
#define BLE_SCAN_TIME 3      // BLE scan duration in seconds

// Scanner enable/disable flags
#define ENABLE_WIFI_SCAN true    // Enable/disable WiFi scanning
#define ENABLE_BLE_SCAN true     // Enable/disable Bluetooth scanning

// Output enable/disable flags
#define ENABLE_CONSOLE_OUTPUT false   // Enable/disable serial console output
#define ENABLE_DISPLAY_OUTPUT true   // Enable/disable OLED display output
#define ENABLE_LOG_OUTPUT true       // Enable/disable SD card logging

// Timing variables
unsigned long lastScan = 0;
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_UPDATE_INTERVAL 1000  // Update display every 1000ms (1 second)

// Active scanning flags (for display indicators)
volatile bool wifiScanning = false;
volatile bool bleScanning = false;

// Cached device counts for display (reduces mutex contention)
int cached_wifi_last = 0;
int cached_ble_last = 0;
int cached_wifi_total = 0;
int cached_ble_total = 0;
unsigned long lastCountsCacheUpdate = 0;
#define COUNTS_CACHE_INTERVAL 200  // Update cached counts every 200ms

// FreeRTOS Task Handles
TaskHandle_t unifiedScanTaskHandle = NULL;  // Single unified scan task for all scan types
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
int uniqueWiFiCount = 0;
int uniqueBLECount = 0;
int lastWiFiScanCount = 0;    // Devices found in last scan
int lastBLEScanCount = 0;     // Devices found in last scan

// Memory protection: max entries per map before clearing (prevents heap exhaustion)
// ~500 entries ≈ 25KB per map, total ~50KB for tracking
// After clearing, unique counts remain accurate but duplicate detection resets
#define MAX_DEVICE_MAP_ENTRIES 500

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

// File sharing mode
bool fileSharingMode = false;
bool scanMode = false;  // Track if we're in scan mode
bool scanTasksStarted = false;  // Whether scan tasks have been created (persists across mode toggles)
unsigned long gpsWaitStart = 0; // When GPS wait began (for elapsed time display)
int gpsWaitDotCount = 0;        // Dot animation counter for GPS wait console output
String fileSharingIP;
AsyncWebServer server(80);
bool serverRoutesConfigured = false;

// WiFi logo animation state (0 = dot only, 1-3 = number of rings)
int wifiAnimationState = 0;

// Deep sleep button state tracking
unsigned long buttonPressStart = 0;
bool buttonPressed = false;

BLEScan* pBLEScan;

// BLE Scan callback class - single static instance to avoid memory leak
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // This will be called for each BLE device found during scan
  }
};
static MyAdvertisedDeviceCallbacks bleCallbacks;  // Single instance, reused every scan

// Helper function to sanitize strings (remove non-printable ASCII and control characters)
String sanitizeString(const String& input) {
  String output = "";
  output.reserve(input.length());

  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    // Only keep printable ASCII characters (space through tilde: 32-126)
    // Replace newlines, tabs, and other control characters with space
    if (c >= 32 && c <= 126) {
      output += c;
    } else if (c == '\n' || c == '\r' || c == '\t') {
      output += ' ';  // Replace whitespace control chars with space
    }
    // Skip all other non-printable/non-ASCII characters
  }

  // Trim trailing spaces
  while (output.length() > 0 && output.charAt(output.length() - 1) == ' ') {
    output.remove(output.length() - 1);
  }

  return output;
}

// Safe string copy with guaranteed null-termination and sanitization
void safeCopy(char* dest, size_t destSize, const String& src) {
  if (destSize == 0) return;

  String sanitized = sanitizeString(src);
  size_t len = sanitized.length();
  if (len >= destSize) {
    len = destSize - 1;  // Leave room for null terminator
  }

  // Copy characters
  for (size_t i = 0; i < len; i++) {
    dest[i] = sanitized.charAt(i);
  }

  // Always null-terminate
  dest[len] = '\0';
}

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

void ledFileSharing() {
  setLEDColor(0, 100, 255);  // Blue: File sharing mode
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
      logFile.println("GPS-timestamped WiFi and Bluetooth scan results");
      logFile.println();
      logFile.println("=== Column Headers ===");
      logFile.println();
      logFile.println("WIFI Format:");
      logFile.println("Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,SSID,BSSID,RSSI,Channel,Band,Encryption");
      logFile.println();
      logFile.println("BLE Format:");
      logFile.println("Type,Fingerprint,Timestamp,Latitude,Longitude,Altitude,Satellites,HDOP,Name,Address,RSSI,ManufacturerData,ServiceUUID");
      logFile.println();
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

// FreeRTOS Task: Unified Scanner
// Single unified scanning task - scans WiFi and BLE sequentially
// ESP32-C5 has a shared 2.4GHz radio, so we must initialize/deinitialize
// each radio type between scans to avoid conflicts
void unifiedScanTask(void* parameter) {
  consolePrintln("[Scan Task] Unified scanner started");

  while (true) {
    if (!logFileReady) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for log file to be ready
      continue;
    }

    consolePrintln("\n========== SCAN CYCLE START ==========");
    unsigned long cycleStart = millis();

    // ===== WIFI SCAN =====
    if (ENABLE_WIFI_SCAN) {
      consolePrintln("\n[1/2] Initializing WiFi radio...");
      WiFi.mode(WIFI_STA);
      delay(300);  // Allow radio to initialize

      wifiScanning = true;
      scanWiFi();
      wifiScanning = false;

      // Turn off WiFi radio before next scan type
      consolePrintln("Releasing WiFi radio...");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(500);  // Allow radio to fully release
    }

    // ===== BLE SCAN =====
    if (ENABLE_BLE_SCAN) {
      consolePrintln("\n[2/2] Initializing BLE radio...");

      // Ensure clean state
      if (BLEDevice::getInitialized()) {
        BLEDevice::deinit(true);
        delay(300);
      }

      BLEDevice::init("SignalScout");
      delay(200);
      pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks, false);  // Use static instance, don't take ownership
      pBLEScan->setActiveScan(true);
      pBLEScan->setInterval(100);
      pBLEScan->setWindow(99);

      bleScanning = true;
      scanBluetooth();
      bleScanning = false;

      // Deinitialize BLE before next scan type
      consolePrintln("Releasing BLE radio...");
      BLEDevice::deinit(true);
      pBLEScan = NULL;
      delay(500);  // Allow radio to fully release
    }

    unsigned long cycleDuration = millis() - cycleStart;

    // Memory monitoring: warn if heap is getting low
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 30000) {
      consolePrintf("WARNING: Low memory! Free heap: %u bytes\n", freeHeap);
    }

    consolePrintf("========== SCAN CYCLE COMPLETE (%lu ms, heap: %u) ==========\n", cycleDuration, freeHeap);

    // Wait before next scan cycle (account for time already spent scanning)
    unsigned long waitTime = (SCAN_INTERVAL * 1000) > cycleDuration ?
                             (SCAN_INTERVAL * 1000) - cycleDuration : 1000;
    consolePrintf("Next scan cycle in %lu ms\n", waitTime);
    vTaskDelay(pdMS_TO_TICKS(waitTime));
  }
}

// FreeRTOS Task: SD Card Logger (processes queue)
// Uses fixed buffers to avoid heap fragmentation from String concatenation
void sdLogTask(void* parameter) {
  LogEntry entry;
  char logBuffer[512];  // Fixed buffer for log line (avoids heap fragmentation)
  char timestamp[40];
  char gpsLat[16], gpsLon[16], gpsAlt[12], gpsSats[8], gpsHdop[8];

  while (true) {
    // Wait for log entries in the queue
    if (xQueueReceive(logQueue, &entry, portMAX_DELAY) == pdTRUE) {
      // Process the log entry
      if (ENABLE_LOG_OUTPUT && sdCardMounted && logFileReady) {
        if (xSemaphoreTake(sdCardMutex, portMAX_DELAY) == pdTRUE) {
          File logFile = SD.open(logFileName.c_str(), FILE_APPEND);
          if (logFile) {
            // Get timestamp and GPS data using fixed buffers
            if (gps.time.isValid() && gps.date.isValid()) {
              snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
                       gps.date.year(), gps.date.month(), gps.date.day(),
                       gps.time.hour(), gps.time.minute(), gps.time.second());

              if (gps.location.isValid()) {
                snprintf(gpsLat, sizeof(gpsLat), "%.6f", gps.location.lat());
                snprintf(gpsLon, sizeof(gpsLon), "%.6f", gps.location.lng());
              } else {
                strcpy(gpsLat, "N/A");
                strcpy(gpsLon, "N/A");
              }
              if (gps.altitude.isValid()) {
                snprintf(gpsAlt, sizeof(gpsAlt), "%.2f", gps.altitude.meters());
              } else {
                strcpy(gpsAlt, "N/A");
              }
              if (gps.satellites.isValid()) {
                snprintf(gpsSats, sizeof(gpsSats), "%d", gps.satellites.value());
              } else {
                strcpy(gpsSats, "N/A");
              }
              if (gps.hdop.isValid()) {
                snprintf(gpsHdop, sizeof(gpsHdop), "%.2f", gps.hdop.hdop());
              } else {
                strcpy(gpsHdop, "N/A");
              }
            } else if (rtc.getYear() > 2020) {
              snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d (RTC)",
                       rtc.getYear(), rtc.getMonth() + 1, rtc.getDay(),
                       rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
              strcpy(gpsLat, "N/A");
              strcpy(gpsLon, "N/A");
              strcpy(gpsAlt, "N/A");
              strcpy(gpsSats, "N/A");
              strcpy(gpsHdop, "N/A");
            } else {
              snprintf(timestamp, sizeof(timestamp), "%lums", millis());
              strcpy(gpsLat, "N/A");
              strcpy(gpsLon, "N/A");
              strcpy(gpsAlt, "N/A");
              strcpy(gpsSats, "N/A");
              strcpy(gpsHdop, "N/A");
            }

            // Build log entry using fixed buffer (no String allocation)
            if (strcmp(entry.type, "WIFI") == 0) {
              snprintf(logBuffer, sizeof(logBuffer), "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                       entry.type, entry.fingerprint, timestamp,
                       gpsLat, gpsLon, gpsAlt, gpsSats, gpsHdop,
                       entry.param1, entry.param2, entry.param3,
                       entry.param4, entry.param5, entry.param6);
            } else if (strcmp(entry.type, "BLE") == 0) {
              snprintf(logBuffer, sizeof(logBuffer), "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                       entry.type, entry.fingerprint, timestamp,
                       gpsLat, gpsLon, gpsAlt, gpsSats, gpsHdop,
                       entry.param1, entry.param2, entry.param3,
                       entry.param4, entry.param5);
            } else {
              snprintf(logBuffer, sizeof(logBuffer), "%s,%s,%s,%s,%s,%s,%s,%s",
                       entry.type, entry.fingerprint, timestamp,
                       gpsLat, gpsLon, gpsAlt, gpsSats, gpsHdop);
            }

            logFile.println(logBuffer);
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
  consolePrintln("SignalScout - WiFi & Bluetooth Scanner with GPS");
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

  // Initialize multi-function button for file sharing and deep sleep
  pinMode(SHARE_BUTTON_PIN, INPUT_PULLUP);  // Active LOW with internal pullup
  consolePrintln("Share button initialized (GPIO23): 1s = mode toggle, 3s = sleep");

  // If waking from deep sleep, wait for button release to avoid immediate re-trigger
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    consolePrintln("Woke from deep sleep via GPIO - waiting for button release...");
    while (digitalRead(SHARE_BUTTON_PIN) == LOW) {
      delay(50);
    }
    consolePrintln("Button released, continuing boot");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    consolePrintln("Woke from deep sleep via timer (periodic wakeup)");
  }

  // ============================================
  // CRITICAL: Initialize SD card FIRST before any radio initialization
  // The BLE radio can interfere with SPI if initialized first
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
      display.setRotation(2);  // Rotate display 180 degrees
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
  if (ENABLE_WIFI_SCAN) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    consolePrintln("WiFi initialized (dual-band 2.4GHz + 5GHz mode)");
  } else {
    consolePrintln("WiFi scanning disabled in config");
  }

  // Skip BLE initialization on boot - it will be initialized by the unified scan task
  consolePrintln("Skipping BLE init (will initialize when scan task starts)");
  consolePrintln("\n[7/7] Skipped - BLE deferred to scan task");

  // Configure deep sleep wakeup source (GPIO wakeup - share button)
  // For ESP32-C5, use the deep sleep GPIO wakeup (available on newer ESP32 variants)
  // Note: GPIO must support deep sleep wakeup (RTC domain GPIO)
  uint64_t wakeup_pin_mask = 1ULL << SHARE_BUTTON_PIN;
  esp_err_t wakeup_result = esp_deep_sleep_enable_gpio_wakeup(wakeup_pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW);
  if (wakeup_result == ESP_OK) {
    consolePrintln("Deep sleep GPIO wakeup configured (press button to wake)");
  } else {
    consolePrintf("WARNING: GPIO%d may not support deep sleep wakeup (error: %d)\n", SHARE_BUTTON_PIN, wakeup_result);
    consolePrintln("You may need to reset/power cycle to wake from deep sleep");
    // Fallback: enable timer wakeup as backup (wake every 60 seconds to check button)
    esp_sleep_enable_timer_wakeup(60 * 1000000ULL); // 60 seconds
    consolePrintln("Timer wakeup enabled as fallback (60s intervals)");
  }

  // Setup complete - default to scan mode; GPS wait runs non-blocking in loop()
  consolePrintln("\n=== INITIALIZATION COMPLETE ===");
  consolePrintln("Scan mode default - waiting for GPS (press button to switch to file sharing)...\n");

  scanMode = true;
  gpsWaitStart = millis();
  ledWaitingGPS();  // Orange: waiting for GPS
}

void loop() {
  // Main loop handles GPS reading, battery monitoring, display updates, and light sleep
  // All scanning is handled by FreeRTOS tasks

  unsigned long currentTime = millis();

  // Read GPS data continuously (limit reads to prevent blocking)
  int gpsReadCount = 0;
  while (gpsSerial.available() > 0 && gpsReadCount < 100) {
    char c = gpsSerial.read();
    gps.encode(c);
    gpsReadCount++;
  }

  // Periodically update RTC from GPS (every ~60 seconds) to keep it accurate
  if (gps.time.isValid() && gps.date.isValid()) {
    static unsigned long lastRTCSync = 0;
    if (currentTime - lastRTCSync > 60000) {
      rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                  gps.date.day(), gps.date.month(), gps.date.year());
      lastRTCSync = currentTime;
    }
  }

  // Scan mode: non-blocking GPS wait then start tasks (button still works during wait)
  if (scanMode && !scanTasksStarted) {
    if (gps.location.isValid() && gps.time.isValid()) {
      // GPS acquired - sync RTC and start scanning
      gpsTimeValid = true;
      consolePrintln("\nGPS signal acquired!");
      displayGPSInfo();

      rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                  gps.date.day(), gps.date.month(), gps.date.year());
      rtcSyncedFromGPS = true;
      consolePrintln("RTC synced from GPS time!");

      startScanTasks();
    }
  }

  // Update cached device counts periodically (reduces mutex contention during display updates)
  if (currentTime - lastCountsCacheUpdate >= COUNTS_CACHE_INTERVAL) {
    if (xSemaphoreTake(deviceMapMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      cached_wifi_last = lastWiFiScanCount;
      cached_ble_last = lastBLEScanCount;
      cached_wifi_total = uniqueWiFiCount;
      cached_ble_total = uniqueBLECount;
      xSemaphoreGive(deviceMapMutex);
      lastCountsCacheUpdate = currentTime;
    }
  }

  // Update battery reading periodically
  if (currentTime - lastBatteryRead >= BATTERY_READ_INTERVAL) {
    batteryPercent = readBatteryPercent();
    lastBatteryRead = currentTime;
  }

  // Update display periodically - GUARANTEED to run every second
  if (ENABLE_DISPLAY_OUTPUT && currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    if (fileSharingMode) {
      updateDisplayFileSharing();
    } else if (scanMode && !scanTasksStarted) {
      updateDisplayGPSWait();
    } else {
      updateDisplay("");
    }
    lastDisplayUpdate = currentTime;
  }

  // Multi-function button (SHARE_BUTTON_PIN on GPIO23)
  // Hold and release between 1-3s: toggle between file sharing mode and scan mode
  // Hold 3s (while still held): enter deep sleep
  bool shareButtonState = (digitalRead(SHARE_BUTTON_PIN) == LOW);

  if (shareButtonState && !buttonPressed) {
    // Button just pressed, start timing
    buttonPressed = true;
    buttonPressStart = currentTime;
  } else if (shareButtonState && buttonPressed) {
    // Button is being held, check for 3-second sleep threshold
    unsigned long holdDuration = currentTime - buttonPressStart;
    if (holdDuration >= SLEEP_HOLD_TIME) {
      if (scanMode) {
        exitScanMode();
      }
      enterDeepSleep();
      // enterDeepSleep() does not return - device reboots on wake
    }
  } else if (!shareButtonState && buttonPressed) {
    // Button released - check if held long enough for mode toggle
    unsigned long holdDuration = currentTime - buttonPressStart;
    if (holdDuration >= SHARE_HOLD_TIME && holdDuration < SLEEP_HOLD_TIME) {
      // Toggle between scan mode and file sharing mode
      if (scanMode) {
        exitScanMode();
      } else {
        enterScanMode();
      }
    }
    buttonPressed = false;
  }

  // Minimal delay to prevent tight loop but maintain responsiveness (10ms = 100Hz loop)
  delay(10);
}

void enterDeepSleep() {
  consolePrintln("\n=== ENTERING DEEP SLEEP ===");
  consolePrintln("Press button to wake up (device will reboot)");

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

  // Log to SD card before sleep (flush queue first)
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Entering deep sleep mode");
  }

  // Turn off status LED
  setLEDOff();

  // Flush serial output
  Serial.flush();

  // Enter deep sleep (wakes on button press - GPIO wakeup configured in setup)
  // esp_deep_sleep_start() does not return - device reboots when woken
  esp_deep_sleep_start();
}

// Format file size for display in file browser
String formatFileSize(size_t size) {
  if (size < 1024) return String(size) + " B";
  if (size < 1024 * 1024) return String((float)size / 1024.0, 1) + " KB";
  return String((float)size / (1024.0 * 1024.0), 1) + " MB";
}

// Files per page in web file browser
#define FILES_PER_PAGE 20

// Configure web server routes (called once before first server.begin())
void configureServerRoutes() {
  if (serverRoutesConfigured) return;

  // Root page - lists files on SD card with pagination, stats, and delete buttons
  // Uses chunked response to avoid building large strings in memory
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    int page = 0;
    if (request->hasParam("page")) {
      page = request->getParam("page")->value().toInt();
      if (page < 0) page = 0;
    }

    // Check for status message from delete redirect
    String statusMsg = "";
    if (request->hasParam("msg")) {
      statusMsg = request->getParam("msg")->value();
    }

    int skipCount = page * FILES_PER_PAGE;
    int fileIndex = 0;
    int filesShown = 0;
    int totalFiles = 0;
    size_t totalUsedBytes = 0;

    // First pass: count total files and sum total size
    if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
      request->send(503, "text/plain", "SD card busy, try again");
      return;
    }

    File root = SD.open("/");
    if (!root) {
      xSemaphoreGive(sdCardMutex);
      request->send(500, "text/plain", "Failed to open SD card");
      return;
    }
    File countFile = root.openNextFile();
    while (countFile) {
      if (!countFile.isDirectory()) {
        totalFiles++;
        totalUsedBytes += countFile.size();
      }
      countFile.close();
      countFile = root.openNextFile();
    }
    root.close();

    // Get SD card capacity info
    uint64_t sdTotalBytes = SD.totalBytes();
    uint64_t sdUsedBytes = SD.usedBytes();
    xSemaphoreGive(sdCardMutex);

    int totalPages = (totalFiles + FILES_PER_PAGE - 1) / FILES_PER_PAGE;
    if (totalPages == 0) totalPages = 1;
    if (page >= totalPages) page = totalPages - 1;

    // Use chunked response to stream HTML without buffering the whole page
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<!DOCTYPE html><html><head><title>SignalScout Files</title>"
                    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                    "<style>"
                    "body{font-family:sans-serif;margin:20px;background:#1a1a2e;color:#e0e0e0;}"
                    "h1{color:#00d4ff;margin-bottom:5px;}"
                    ".stats{background:#16213e;border:1px solid #333;border-radius:8px;"
                    "padding:12px 16px;margin:12px 0;display:flex;gap:24px;flex-wrap:wrap;}"
                    ".stat{display:flex;flex-direction:column;}"
                    ".stat-val{color:#00d4ff;font-size:1.2em;font-weight:bold;}"
                    ".stat-lbl{color:#888;font-size:0.75em;}"
                    ".bar{background:#333;border-radius:4px;height:8px;width:120px;margin-top:4px;}"
                    ".bar-fill{background:#00d4ff;border-radius:4px;height:100%;}"
                    "a{color:#00d4ff;text-decoration:none;}"
                    "a:hover{text-decoration:underline;}"
                    ".file{padding:8px 0;border-bottom:1px solid #333;"
                    "display:flex;align-items:center;gap:10px;}"
                    ".fname{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;}"
                    ".size{color:#888;font-size:0.85em;white-space:nowrap;}"
                    ".del{color:#ff4444;background:none;border:1px solid #ff4444;"
                    "border-radius:4px;padding:2px 8px;cursor:pointer;font-size:0.8em;white-space:nowrap;}"
                    ".del:hover{background:#ff4444;color:#fff;}"
                    ".empty{color:#666;font-style:italic;}"
                    ".msg{padding:10px 14px;border-radius:6px;margin:10px 0;"
                    "background:#1a3a1a;border:1px solid #2a5a2a;color:#6f6;}"
                    ".msg-err{background:#3a1a1a;border-color:#5a2a2a;color:#f66;}"
                    ".nav{margin-top:20px;display:flex;gap:10px;align-items:center;flex-wrap:wrap;}"
                    ".nav a,.nav span{padding:6px 14px;border:1px solid #444;border-radius:4px;}"
                    ".nav .cur{background:#00d4ff;color:#1a1a2e;border-color:#00d4ff;}"
                    ".delall{margin-top:20px;padding:10px 20px;background:none;"
                    "border:1px solid #ff4444;color:#ff4444;border-radius:6px;"
                    "cursor:pointer;font-size:0.9em;}"
                    ".delall:hover{background:#ff4444;color:#fff;}"
                    ".heap{color:#555;font-size:0.75em;margin-top:15px;}"
                    "</style></head><body>"
                    "<h1>SignalScout Files</h1>");

    // Status message (e.g. after delete)
    if (statusMsg.length() > 0) {
      bool isErr = statusMsg.startsWith("Error");
      response->print(isErr ? "<div class=\"msg msg-err\">" : "<div class=\"msg\">");
      // Sanitize the message to prevent XSS
      for (unsigned int i = 0; i < statusMsg.length() && i < 100; i++) {
        char c = statusMsg.charAt(i);
        if (c == '<') response->print("&lt;");
        else if (c == '>') response->print("&gt;");
        else if (c == '&') response->print("&amp;");
        else if (c >= 32 && c <= 126) { char s[2] = {c, 0}; response->print(s); }
      }
      response->print("</div>");
    }

    // SD card stats
    response->print("<div class=\"stats\">");

    char statBuf[128];
    snprintf(statBuf, sizeof(statBuf),
             "<div class=\"stat\"><span class=\"stat-val\">%d</span>"
             "<span class=\"stat-lbl\">Files</span></div>", totalFiles);
    response->print(statBuf);

    String usedStr = formatFileSize(totalUsedBytes);
    snprintf(statBuf, sizeof(statBuf),
             "<div class=\"stat\"><span class=\"stat-val\">%s</span>"
             "<span class=\"stat-lbl\">Used by files</span></div>", usedStr.c_str());
    response->print(statBuf);

    if (sdTotalBytes > 0) {
      String totalStr = formatFileSize((size_t)(sdTotalBytes));
      String sdUsedStr = formatFileSize((size_t)(sdUsedBytes));
      int usedPct = (int)((sdUsedBytes * 100ULL) / sdTotalBytes);
      snprintf(statBuf, sizeof(statBuf),
               "<div class=\"stat\"><span class=\"stat-val\">%s / %s</span>"
               "<span class=\"stat-lbl\">SD card used (%d%%)</span>"
               "<div class=\"bar\"><div class=\"bar-fill\" style=\"width:%d%%\"></div></div></div>",
               sdUsedStr.c_str(), totalStr.c_str(), usedPct, usedPct);
      response->print(statBuf);
    }

    // Free heap
    snprintf(statBuf, sizeof(statBuf),
             "<div class=\"stat\"><span class=\"stat-val\">%u B</span>"
             "<span class=\"stat-lbl\">Free heap</span></div>",
             (unsigned int)ESP.getFreeHeap());
    response->print(statBuf);

    response->print("</div>");  // end stats

    // File count and page info
    char infoBuf[80];
    snprintf(infoBuf, sizeof(infoBuf),
             "<p style=\"color:#888;font-size:0.9em;\">Page %d of %d</p>",
             page + 1, totalPages);
    response->print(infoBuf);

    if (totalFiles == 0) {
      response->print("<p class=\"empty\">No files found on SD card.</p>");
    } else {
      // Second pass: output only the files for this page
      if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        File root2 = SD.open("/");
        if (root2) {
          File file = root2.openNextFile();
          while (file) {
            if (!file.isDirectory()) {
              if (fileIndex >= skipCount && filesShown < FILES_PER_PAGE) {
                // Stream one file row at a time using a fixed buffer
                char rowBuf[512];
                const char* fname = file.name();
                String sizeStr = formatFileSize(file.size());
                snprintf(rowBuf, sizeof(rowBuf),
                         "<div class=\"file\">"
                         "<span class=\"fname\"><a href=\"/download?name=%s\">%s</a></span>"
                         "<span class=\"size\">%s</span>"
                         "<button class=\"del\" onclick=\"delFile('%s')\">Delete</button>"
                         "</div>",
                         fname, fname, sizeStr.c_str(), fname);
                response->print(rowBuf);
                filesShown++;
              } else if (fileIndex >= skipCount + FILES_PER_PAGE) {
                file.close();
                break;
              }
              fileIndex++;
            }
            file.close();
            file = root2.openNextFile();
          }
          root2.close();
        }
        xSemaphoreGive(sdCardMutex);
      }
    }

    // Pagination navigation
    if (totalPages > 1) {
      response->print("<div class=\"nav\">");
      if (page > 0) {
        char linkBuf[64];
        snprintf(linkBuf, sizeof(linkBuf), "<a href=\"/?page=%d\">&laquo; Prev</a>", page - 1);
        response->print(linkBuf);
      }
      int startPage = (page > 3) ? page - 3 : 0;
      int endPage = startPage + 7;
      if (endPage > totalPages) endPage = totalPages;
      for (int p = startPage; p < endPage; p++) {
        char pgBuf[64];
        if (p == page) {
          snprintf(pgBuf, sizeof(pgBuf), "<span class=\"cur\">%d</span>", p + 1);
        } else {
          snprintf(pgBuf, sizeof(pgBuf), "<a href=\"/?page=%d\">%d</a>", p, p + 1);
        }
        response->print(pgBuf);
      }
      if (page < totalPages - 1) {
        char linkBuf[64];
        snprintf(linkBuf, sizeof(linkBuf), "<a href=\"/?page=%d\">Next &raquo;</a>", page + 1);
        response->print(linkBuf);
      }
      response->print("</div>");
    }

    // Delete all button (only show if there are files)
    if (totalFiles > 0) {
      char delAllBuf[128];
      snprintf(delAllBuf, sizeof(delAllBuf),
               "<button class=\"delall\" onclick=\"delAll(%d)\">Delete All Files (%d)</button>",
               totalFiles, totalFiles);
      response->print(delAllBuf);
    }

    // JavaScript for delete confirmation
    response->print("<script>"
                    "function delFile(n){"
                    "if(confirm('Delete '+n+'?')){"
                    "fetch('/delete?name='+encodeURIComponent(n),{method:'POST'})"
                    ".then(r=>r.text()).then(t=>{"
                    "window.location='/?msg='+encodeURIComponent(t);"
                    "}).catch(e=>{alert('Error: '+e);});}}"
                    "function delAll(cnt){"
                    "if(confirm('Delete ALL '+cnt+' files? This cannot be undone.')){"
                    "if(confirm('Are you really sure? All scan data will be lost.')){"
                    "fetch('/delete-all',{method:'POST'})"
                    ".then(r=>r.text()).then(t=>{"
                    "window.location='/?msg='+encodeURIComponent(t);"
                    "}).catch(e=>{alert('Error: '+e);});}}}"
                    "</script>");

    response->print("</body></html>");
    request->send(response);
  });

  // File download - streams file directly from SD card
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("name")) {
      request->send(400, "text/plain", "Missing filename");
      return;
    }
    String filename = request->getParam("name")->value();
    // Block path traversal attempts
    if (filename.indexOf("..") >= 0) {
      request->send(400, "text/plain", "Invalid filename");
      return;
    }
    String filepath = "/" + filename;
    // Note: SD mutex not held here because ESPAsyncWebServer streams the file
    // asynchronously over multiple TCP callbacks - holding a mutex across that
    // would block other tasks for the entire transfer duration.
    // In file sharing mode, scan tasks are suspended so there are no SD conflicts.
    if (SD.exists(filepath.c_str())) {
      request->send(SD, filepath, "application/octet-stream", true);
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });

  // Delete a single file
  server.on("/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("name")) {
      request->send(400, "text/plain", "Missing filename");
      return;
    }
    String filename = request->getParam("name")->value();
    if (filename.indexOf("..") >= 0) {
      request->send(400, "text/plain", "Invalid filename");
      return;
    }
    String filepath = "/" + filename;

    if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
      request->send(503, "text/plain", "Error: SD card busy");
      return;
    }
    bool existed = SD.exists(filepath.c_str());
    bool removed = false;
    if (existed) {
      removed = SD.remove(filepath.c_str());
    }
    xSemaphoreGive(sdCardMutex);

    if (!existed) {
      request->send(404, "text/plain", "Error: File not found");
    } else if (removed) {
      consolePrintf("Deleted file: %s\n", filepath.c_str());
      request->send(200, "text/plain", String("Deleted " + filename).c_str());
    } else {
      request->send(500, "text/plain", "Error: Failed to delete file");
    }
  });

  // Delete all files
  server.on("/delete-all", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
      request->send(503, "text/plain", "Error: SD card busy");
      return;
    }

    int deleted = 0;
    int failed = 0;
    File root = SD.open("/");
    if (root) {
      // Collect filenames first (can't delete while iterating on some FS implementations)
      // Use a fixed array to avoid heap fragmentation
      const int MAX_BATCH = 50;
      String names[MAX_BATCH];
      int count = 0;

      // Process in batches to limit memory usage
      bool moreFiles = true;
      while (moreFiles) {
        count = 0;
        root.rewindDirectory();
        // Skip already-processed files by scanning past deleted ones
        File file = root.openNextFile();
        while (file && count < MAX_BATCH) {
          if (!file.isDirectory()) {
            names[count++] = String("/") + file.name();
          }
          file.close();
          file = root.openNextFile();
        }
        if (file) file.close();
        moreFiles = (count == MAX_BATCH);

        // Delete this batch
        for (int i = 0; i < count; i++) {
          if (SD.remove(names[i].c_str())) {
            deleted++;
          } else {
            failed++;
          }
        }
        if (count == 0) break;
      }
      root.close();
    }
    xSemaphoreGive(sdCardMutex);

    consolePrintf("Delete all: %d deleted, %d failed\n", deleted, failed);
    char msg[64];
    if (failed == 0) {
      snprintf(msg, sizeof(msg), "Deleted %d file%s", deleted, deleted == 1 ? "" : "s");
    } else {
      snprintf(msg, sizeof(msg), "Deleted %d, %d failed", deleted, failed);
    }
    request->send(200, "text/plain", msg);
  });

  serverRoutesConfigured = true;
}

void enterFileSharingMode() {
  consolePrintln("\n=== ENTERING FILE SHARING MODE ===");
  fileSharingMode = true;

  // Suspend scanning and logging tasks to avoid SD and radio conflicts (if they exist)
  if (unifiedScanTaskHandle != NULL) vTaskSuspend(unifiedScanTaskHandle);
  if (sdLogTaskHandle != NULL) vTaskSuspend(sdLogTaskHandle);

  // Deinitialize BLE to free up 2.4GHz radio for WiFi (if it was initialized)
  // BLE and WiFi share radio resources on ESP32-C5
  // The unified scan task releases radios after each scan, but we double-check here
  if (ENABLE_BLE_SCAN && BLEDevice::getInitialized()) {
    consolePrintln("Deinitializing BLE to free radio for WiFi...");
    BLEDevice::deinit(true);  // true = release all BLE memory for clean radio handoff
    pBLEScan = NULL;          // Mark as deinitialized
    delay(500);  // Allow BLE to fully release radio (increased for ESP32-C5)
  }

  // Connect to WiFi using credentials from secrets.h
  // Full WiFi radio reset sequence for ESP32-C5 shared radio
  // (BLE/WiFi share the 2.4GHz radio, needs clean handoff)
  WiFi.mode(WIFI_OFF);    // Completely turn off WiFi radio
  delay(1000);            // Allow radio to fully release (longer for ESP32-C5)

  WiFi.persistent(false); // Don't save credentials to flash (avoids flash wear and corruption issues)
  WiFi.setAutoReconnect(false);  // We handle reconnection ourselves
  WiFi.mode(WIFI_STA);
  delay(500);             // Stabilization delay after mode change

  // Configure WiFi for better 5GHz compatibility
  // ESP32-C5 may have issues with 5GHz after BLE radio release
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max TX power for better range
  WiFi.setSleep(false);                  // Disable modem sleep for reliable connection

  // Perform a full scan to warm up the radio before connecting
  // This helps stabilize the radio after BLE handoff
  consolePrintln("Scanning to warm up radio...");
  int numNetworks = WiFi.scanNetworks();  // Full scan with default timing
  consolePrintf("Found %d networks\n", numNetworks);

  // Check if our target network is visible
  bool foundNetwork = false;
  for (int i = 0; i < numNetworks; i++) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      consolePrintf("  Target found: %s (RSSI: %d, Ch: %d)\n",
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
      foundNetwork = true;
    }
  }
  WiFi.scanDelete();  // Free scan results memory

  if (!foundNetwork) {
    consolePrintf("WARNING: Network '%s' not found in scan, trying anyway...\n", WIFI_SSID);
  }

  // Simple connection - let ESP32 driver choose best AP
  consolePrintf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (ENABLE_DISPLAY_OUTPUT) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(2, 0);
    display.println("File Share Mode");
    display.setCursor(2, 16);
    display.println("Connecting...");
    display.display();
  }

  // Try connecting with retries - ESP32-C5 sometimes needs multiple attempts after radio switch
  bool connected = false;
  for (int retry = 0; retry < 3 && !connected; retry++) {
    if (retry > 0) {
      consolePrintf("\nRetry %d/3 - Resetting WiFi...\n", retry + 1);
      WiFi.disconnect(true);
      delay(500);
      WiFi.mode(WIFI_OFF);
      delay(500);
      WiFi.mode(WIFI_STA);
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    consolePrintf("Waiting for connection (attempt %d/3, timeout: 10s)...\n", retry + 1);

    // Wait for connection with 10-second timeout
    for (int attempt = 0; attempt < 10 && WiFi.status() != WL_CONNECTED; attempt++) {
      delay(1000);
      wl_status_t status = WiFi.status();
      consolePrintf("  %ds - Status: %d", attempt + 1, status);
      if (status == WL_CONNECT_FAILED) {
        consolePrintln(" (CONNECT_FAILED - bad password?)");
        break;  // Don't waste time if password is wrong
      } else if (status == WL_NO_SSID_AVAIL) {
        consolePrintln(" (NO_SSID - network not found)");
      } else {
        consolePrintln("");
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
    }
  }

  if (!connected) {
    wl_status_t status = WiFi.status();
    consolePrintf("ERROR: Failed to connect to WiFi! Status: %d\n", status);
    consolePrintf("Status meanings: 0=IDLE, 1=NO_SSID, 3=CONNECTED, 4=CONNECT_FAILED, 6=DISCONNECTED\n");
    consolePrintf("SSID: '%s' (length: %d)\n", WIFI_SSID, strlen(WIFI_SSID));
    consolePrintln("Check SSID and password in secrets.h");

    // Print available networks for debugging
    consolePrintln("Scanning for available networks...");
    // Reset WiFi to clean state for scanning
    WiFi.disconnect(true);
    delay(200);
    WiFi.mode(WIFI_OFF);
    delay(300);
    WiFi.mode(WIFI_STA);
    delay(500);
    int n = WiFi.scanNetworks();
    if (n == 0) {
      consolePrintln("  No networks found");
    } else if (n < 0) {
      consolePrintf("  Scan failed with error: %d\n", n);
    } else {
      for (int i = 0; i < n && i < 5; i++) {
        consolePrintf("  %d: %s (RSSI: %d)\n", i+1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
      }
    }

    fileSharingMode = false;

    // Resume tasks on failure
    if (unifiedScanTaskHandle) vTaskResume(unifiedScanTaskHandle);
    if (sdLogTaskHandle) vTaskResume(sdLogTaskHandle);

    if (ENABLE_DISPLAY_OUTPUT) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(2, 0);
      display.println("WiFi Connect");
      display.setCursor(2, 16);
      display.println("FAILED - check");
      display.setCursor(2, 32);
      display.println("secrets.h");
      display.display();
    }
    delay(3000);
    return;
  }

  fileSharingIP = WiFi.localIP().toString();
  consolePrintf("Connected! IP: %s\n", fileSharingIP.c_str());

  // Configure routes (first time only) and start server
  configureServerRoutes();
  server.begin();
  consolePrintln("File server started on port 80");

  // Update LED and log
  ledFileSharing();
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Entered file sharing mode - IP: " + fileSharingIP);
  }
  consolePrintln((String("Open http://") + fileSharingIP + " in a browser to browse files").c_str());
}

void exitFileSharingMode() {
  consolePrintln("\n=== EXITING FILE SHARING MODE ===");

  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Exiting file sharing mode");
  }

  // Stop web server and fully turn off WiFi to release radio for BLE
  server.end();
  WiFi.disconnect(true);  // Disconnect and clear credentials
  delay(200);
  WiFi.mode(WIFI_OFF);    // Fully turn off WiFi radio
  delay(500);             // Allow radio to fully release

  fileSharingMode = false;

  setLEDOff();
  consolePrintln("File sharing mode exited, WiFi radio released");
}

void enterScanMode() {
  consolePrintln("\n=== ENTERING SCAN MODE ===");

  // Exit file sharing mode if active
  if (fileSharingMode) {
    exitFileSharingMode();
  }

  scanMode = true;

  if (scanTasksStarted) {
    // Tasks already created previously (we're returning from file sharing) - resume them
    consolePrintln("Resuming scan tasks...");
    if (unifiedScanTaskHandle != NULL) vTaskResume(unifiedScanTaskHandle);
    if (sdLogTaskHandle != NULL) vTaskResume(sdLogTaskHandle);
    if (ENABLE_LOG_OUTPUT && logFileReady) {
      logToFile("Entering scan mode");
    }
    setLEDOff();
    consolePrintln("Scan mode active!\n");
  } else if (gps.location.isValid() && gps.time.isValid()) {
    // GPS already acquired (e.g. acquired while in file sharing) - start tasks now
    consolePrintln("GPS already available, starting tasks immediately");
    gpsTimeValid = true;
    if (!rtcSyncedFromGPS) {
      rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(),
                  gps.date.day(), gps.date.month(), gps.date.year());
      rtcSyncedFromGPS = true;
      consolePrintln("RTC synced from GPS time!");
    }
    startScanTasks();
  } else {
    // GPS not yet acquired - loop() will poll and call startScanTasks() when ready
    consolePrintln("Waiting for GPS signal (non-blocking)...");
    gpsWaitStart = millis();
    ledWaitingGPS();  // Orange: waiting for GPS
  }
}

// Create log file and FreeRTOS scan/log tasks. Called once when GPS first locks.
void startScanTasks() {
  consolePrintln("\n=== STARTING SCAN TASKS ===");

  // Generate log filename if not already created
  if (!logFileReady && ENABLE_LOG_OUTPUT && sdCardMounted) {
    consolePrintln("Creating log file...");
    if (generateLogFileName()) {
      logFileReady = true;
      logToFile("System started - SignalScout initialized");
      logToFile("GPS signal acquired");
      logToFile("Entering scan mode");
      consolePrintf("Logging to: %s\n", logFileName.c_str());
    } else {
      consolePrintln("ERROR: Failed to create log file!");
      logFileReady = false;
    }
  } else if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Entering scan mode");
  }

  // SD Logger Task (only create if SD card is mounted and logging enabled)
  if (ENABLE_LOG_OUTPUT && sdCardMounted && logFileReady) {
    if (sdLogTaskHandle == NULL) {
      BaseType_t result = xTaskCreate(
        sdLogTask,           // Task function
        "SD Logger",         // Task name
        8192,                // Stack size (bytes)
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
      vTaskResume(sdLogTaskHandle);
      consolePrintln("SD Logger task resumed");
    }
  }

  // Unified Scan Task - handles WiFi and BLE sequentially
  // This ensures only one radio type is active at a time on ESP32-C5's shared radio
  if (ENABLE_WIFI_SCAN || ENABLE_BLE_SCAN) {
    if (unifiedScanTaskHandle == NULL) {
      BaseType_t result = xTaskCreate(
        unifiedScanTask,       // Task function
        "Unified Scanner",     // Task name
        8192,                  // Stack size (larger for all scan types)
        NULL,                  // Parameters
        1,                     // Priority
        &unifiedScanTaskHandle // Task handle
      );
      if (result != pdPASS) {
        consolePrintln("ERROR: Failed to create Unified Scanner task!");
      } else {
        consolePrintln("Unified Scanner task created (WiFi->BLE sequential)");
      }
    } else {
      vTaskResume(unifiedScanTaskHandle);
      consolePrintln("Unified Scanner task resumed");
    }
  }

  ledReady();  // Green: Ready
  delay(1000);  // Brief pause to show green LED
  setLEDOff();  // Turn off LED to conserve battery

  scanTasksStarted = true;
  consolePrintln("Scan tasks started!\n");
}

void exitScanMode() {
  consolePrintln("\n=== EXITING SCAN MODE ===");

  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Exiting scan mode");
  }

  // Suspend scanning and logging tasks (only if they were started)
  if (scanTasksStarted) {
    if (unifiedScanTaskHandle != NULL) vTaskSuspend(unifiedScanTaskHandle);
    if (sdLogTaskHandle != NULL) vTaskSuspend(sdLogTaskHandle);
    consolePrintln("Scanning tasks suspended");
  }

  scanMode = false;

  // Enter file sharing mode
  enterFileSharingMode();
}

void updateDisplayFileSharing() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(2, 0);
  display.println("FILE SHARE");
  display.setTextSize(1);
  display.setCursor(2, 22);
  display.print("IP: ");
  display.println(fileSharingIP);
  display.setCursor(2, 38);
  display.println("Hold 1s: scan mode");
  display.setCursor(2, 50);
  display.println("Hold 3s: sleep");
  display.display();
}

void updateDisplayGPSWait() {
  gpsWaitDotCount = (gpsWaitDotCount + 1) % 4;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  unsigned long elapsed = (millis() - gpsWaitStart) / 1000;

  consolePrintf("Waiting for GPS fix (Satellites: %d, Elapsed: %lus)\n", sats, elapsed);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Waiting for GPS...");
  display.setCursor(0, 12);
  display.printf("Sats: %d  Time: %lus", sats, elapsed);
  display.setCursor(0, 24);
  display.printf("Battery: %d%%", batteryPercent);
  display.setCursor(0, 40);
  display.println("Hold 1s: file share");
  display.setCursor(0, 52);
  display.println("Hold 3s: sleep");
  display.display();
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
  // Scan time settings adjusted for ~3 second total scan time
  wifi_scan_config_t scan_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,  // 0 = scan all channels
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = {
      .active = {
        .min = 120,  // Minimum scan time per channel (ms)
        .max = 150   // Maximum scan time per channel (ms) - increased for ~3s total
      },
      .passive = 400  // Passive scan time per channel (ms)
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
      // Memory protection: clear map if it grows too large
      if (seenWiFiDevices.size() >= MAX_DEVICE_MAP_ENTRIES) {
        seenWiFiDevices.clear();
        consolePrintln("WARNING: WiFi device map cleared to prevent memory exhaustion");
      }
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

    // Generate fingerprint using BSSID only (stable hardware identifier)
    // This ensures the same device always gets the same fingerprint
    uint32_t fingerprint = 0;
    for (int j = 0; j < 6; j++) {
      fingerprint = (fingerprint << 8) | ap->bssid[j];
    }
    // Add more entropy by rotating the bits
    fingerprint = (fingerprint ^ (fingerprint >> 16)) * 0x45d9f3b;
    fingerprint = (fingerprint ^ (fingerprint >> 16)) * 0x45d9f3b;
    fingerprint = fingerprint ^ (fingerprint >> 16);

    char fingerprintStr[9];
    snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);

    // Print to Serial
    consolePrintf("%d: %s (%s) | RSSI: %d dBm | Ch: %d (%s) | Enc: %s | FP: %s\n",
                  i + 1, ssid.c_str(), bssidStr, rssi, channel, band.c_str(),
                  encType.c_str(), fingerprintStr);

    // Queue log entry for SD card writing (non-blocking)
    if (ENABLE_LOG_OUTPUT && logFileReady) {
      LogEntry entry;
      // Zero out the entire structure first to ensure no garbage
      memset(&entry, 0, sizeof(LogEntry));

      // Use safe copy functions with guaranteed null-termination
      safeCopy(entry.type, sizeof(entry.type), "WIFI");
      safeCopy(entry.fingerprint, sizeof(entry.fingerprint), fingerprintStr);
      safeCopy(entry.param1, sizeof(entry.param1), ssid);
      safeCopy(entry.param2, sizeof(entry.param2), bssidStr);
      snprintf(entry.param3, sizeof(entry.param3), "%d", rssi);
      snprintf(entry.param4, sizeof(entry.param4), "%d", channel);
      safeCopy(entry.param5, sizeof(entry.param5), band);
      safeCopy(entry.param6, sizeof(entry.param6), encType);

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
      // Memory protection: clear map if it grows too large
      if (seenBLEDevices.size() >= MAX_DEVICE_MAP_ENTRIES) {
        seenBLEDevices.clear();
        consolePrintln("WARNING: BLE device map cleared to prevent memory exhaustion");
      }
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
      // Get raw manufacturer data as hex string (sanitized)
      String rawData = device.getManufacturerData();
      char hexStr[65] = {0};  // Limit to 32 bytes (64 hex chars + null)
      for (size_t i = 0; i < rawData.length() && i < 32; i++) {
        snprintf(hexStr + (i * 2), 3, "%02X", (uint8_t)rawData[i]);
      }
      manufData = String(hexStr);
    }
    if (device.haveServiceUUID()) {
      serviceUUID = device.getServiceUUID().toString().c_str();
    }

    // Generate fingerprint using BLE address only (stable hardware identifier)
    // Parse MAC address string (format: "xx:xx:xx:xx:xx:xx")
    char fingerprintStr[9] = "00000000";  // Default if parsing fails
    uint8_t macBytes[6] = {0};
    if (sscanf(address.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &macBytes[0], &macBytes[1], &macBytes[2],
               &macBytes[3], &macBytes[4], &macBytes[5]) == 6) {
      uint32_t fingerprint = 0;
      for (int j = 0; j < 6; j++) {
        fingerprint = (fingerprint << 8) | macBytes[j];
      }
      // Add entropy
      fingerprint = (fingerprint ^ (fingerprint >> 16)) * 0x45d9f3b;
      fingerprint = (fingerprint ^ (fingerprint >> 16)) * 0x45d9f3b;
      fingerprint = fingerprint ^ (fingerprint >> 16);

      snprintf(fingerprintStr, sizeof(fingerprintStr), "%08X", fingerprint);
    }

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
      // Zero out the entire structure first to ensure no garbage
      memset(&entry, 0, sizeof(LogEntry));

      // Use safe copy functions with guaranteed null-termination
      safeCopy(entry.type, sizeof(entry.type), "BLE");
      safeCopy(entry.fingerprint, sizeof(entry.fingerprint), fingerprintStr);
      safeCopy(entry.param1, sizeof(entry.param1), name);
      safeCopy(entry.param2, sizeof(entry.param2), address);
      snprintf(entry.param3, sizeof(entry.param3), "%d", rssi);
      safeCopy(entry.param4, sizeof(entry.param4), manufData);
      safeCopy(entry.param5, sizeof(entry.param5), serviceUUID);

      // Try to add to queue (don't block if queue is full)
      if (xQueueSend(logQueue, &entry, 0) != pdTRUE) {
        consolePrintln("WARNING: Log queue full, dropping BLE entry");
      }
    }
  }

  pBLEScan->clearResults();
  consolePrintln("--- Bluetooth Scan Complete ---\n");
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

    // Use cached device counts (updated in main loop every 200ms)
    // This prevents display lag from mutex contention with scanning tasks
    int wifi_last = cached_wifi_last;
    int ble_last = cached_ble_last;
    int wifi_total = cached_wifi_total;
    int ble_total = cached_ble_total;

    // WiFi count: Last scan (Total) - add asterisk when scanning
    display.setCursor(2, 22);
    if (wifiScanning) {
      display.printf("W:%d(%d)*", wifi_last, wifi_total);
    } else {
      display.printf("W:%d(%d)", wifi_last, wifi_total);
    }

    // BLE count: Last scan (Total) - add asterisk when scanning
    display.setCursor(2, 34);
    if (bleScanning) {
      display.printf("B:%d(%d)*", ble_last, ble_total);
    } else {
      display.printf("B:%d(%d)", ble_last, ble_total);
    }

    // Draw animated WiFi logo in center-right of screen
    drawWiFiLogo(85, 22);

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
