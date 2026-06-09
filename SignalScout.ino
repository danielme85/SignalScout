/*
 * SignalScout - WiFi and Bluetooth Scanner for ESP32-C5
 * Scans 2.4GHz and 5GHz WiFi networks and Bluetooth LE devices
 * Logs all activity to SD card over SPI with GPS timestamps and location
 * Displays status on SSD1309 OLED display
 *
 * Hardware: ESP32-C5 (16MB FLASH, 8MB PSRAM) or Seeed XIAO ESP32-C5 (8MB FLASH, 8MB PSRAM)
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
#include <U8g2lib.h>
#include <clib/mui.h>
#include <clib/mui_u8g2.h>
#include <ESP32Time.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include <stdarg.h>
#include "secrets.h"
// File sharing via WiFi (install ESPAsyncWebServer and AsyncTCP libraries)
#include <ESPAsyncWebServer.h>
#include <Preferences.h>    // NVS key-value store for persisting settings

// === Board v1 PCB Pinout ===
// SD Card and OLED share the same SPI bus (GPIO 8/10)

// SD Card SPI Pins
#define SD_CS    4   // Chip Select (D3)
#define SD_MOSI  10  // MOSI — shared with OLED (D10)
#define SD_MISO  9   // MISO (D9)
#define SD_SCK   8   // SCK — shared with OLED (D8)

// GPS UART Pins
#define GPS_RX   6   // ESP32 RX ← GPS TX (D6)
#define GPS_TX   7   // ESP32 TX → GPS RX (D7)
#define GPS_BAUD 9600

// OLED Display SPI Pins (shares bus with SD card)
#define OLED_DC    3              // Data/Command (D2)
#define OLED_CS    11             // Chip Select (D4)
#define OLED_RESET U8X8_PIN_NONE  // RST tied to 3V3 on PCB — no GPIO needed
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Font baseline offsets for u8g2 (y-as-baseline vs Adafruit y-as-top-left)
#define FONT_SM_ASCENT 6   // u8g2_font_5x8_tf  ascent above baseline
#define FONT_LG_ASCENT 14  // u8g2_font_10x20_tf ascent above baseline

// Rotary Encoder Pins (replaces single mode button)
#define ENC_A_PIN    1     // Encoder A / CLK (D0) — external 10k pull-up + 10nF debounce on PCB
#define ENC_B_PIN    2     // Encoder B / DT  (D1) — external 10k pull-up + 10nF debounce on PCB
#define ENC_SW_PIN   12    // Encoder switch  (D5) — external 10k pull-up on PCB
#define ENC_HOLD_TIME 1000 // Hold for 1 second to trigger action (ms)

// Board v1 has no RGB LED and no battery ADC — both features disabled

// Scan settings
#define SCAN_INTERVAL 10  // WiFi and BLE scan every x seconds
#define BLE_SCAN_TIME 3      // BLE scan duration in seconds

// Scanner enable/disable flags (runtime bools, persisted in NVS, changeable via pause menu)
bool enableWifiScan = true;   // Enable/disable WiFi scanning
bool enableBleScan  = true;   // Enable/disable BLE scanning
bool flockOnlyMode  = false;  // When true, ignore all non-Flock devices

// Output enable/disable flags
#define ENABLE_CONSOLE_OUTPUT false   // Enable/disable serial console output
#define ENABLE_DISPLAY_OUTPUT true   // Enable/disable OLED display output
#define ENABLE_LOG_OUTPUT true       // Enable/disable SD card logging

// GPS Debug mode - forces serial output of raw NMEA data and TinyGPSPlus stats
// Helps diagnose whether GPS issue is wiring, baud rate, antenna, or software
// Set to true to enable, upload, then open Serial Monitor at 115200 baud
// Diagnostics: 0 chars = wiring/power issue; chars but no $GP = baud mismatch;
//              $GP sentences but no fix = antenna/cold start; fix but stuck = SW bug
#define GPS_DEBUG 0

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
#include <set>
#include <string>
std::map<std::string, bool> seenWiFiDevices;    // Track unique WiFi devices by BSSID
std::map<std::string, bool> seenBLEDevices;     // Track unique BLE devices by address
// Never cleared — keeps Flock dedup accurate even after WiFi/BLE maps are evicted
std::set<std::string> seenFlockKeys;
int uniqueWiFiCount = 0;
int uniqueBLECount = 0;
int uniqueFlockCount = 0;
int lastWiFiScanCount = 0;    // Devices found in last scan
int lastBLEScanCount = 0;     // Devices found in last scan
volatile unsigned long flockLastSeenMs = 0;    // millis() when last Flock device was detected
volatile unsigned long flockAlertUntilMs = 0;  // millis() until full-screen alert is shown

// Display off timer
const char* const dispTimerLabels[] = {"OFF", "30s", "1m", "5m", "10m"};
const uint32_t    dispTimerMs[]     = {0, 30000UL, 60000UL, 300000UL, 600000UL};
#define DISP_TIMER_COUNT 5
uint8_t           displayTimerIdx       = 0;
unsigned long     lastDisplayActivityMs = 0;
bool              displayCurrentlyOff   = false;

// Cached Flock count for display
int cached_flock_total = 0;

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

// OLED Display (SSD1309 128x64, software SPI, 180° rotation via U8G2_R2)
// Board v1: OLED shares HW SPI bus with SD card. RST is tied to 3V3 (U8X8_PIN_NONE).
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R2, OLED_CS, OLED_DC, OLED_RESET);

// MUI (Menu User Interface) - settings menu rendered via u8g2
mui_t mui;

// Forward declarations for MUI callbacks
uint8_t mui_resume_cb(mui_t *ui, uint8_t msg);
uint8_t mui_disptimer_cb(mui_t *ui, uint8_t msg);

muif_t muif_list[] = {
  MUIF_U8G2_LABEL(),
  MUIF_VARIABLE("W", (uint8_t*)&enableWifiScan, mui_u8g2_u8_chkbox_wm_pi),
  MUIF_VARIABLE("B", (uint8_t*)&enableBleScan,  mui_u8g2_u8_chkbox_wm_pi),
  MUIF_VARIABLE("F", (uint8_t*)&flockOnlyMode,  mui_u8g2_u8_chkbox_wm_pi),
  MUIF_BUTTON("D", mui_disptimer_cb),
  MUIF_BUTTON("R", mui_resume_cb),
};

// Content starts at y=12 (below status bar + separator). 8px row spacing.
fds_t fds_settings[] =
MUI_FORM(1)
MUI_LABEL(2, 19, "== Settings ==")
MUI_LABEL(2, 27, "WiFi Scan")
MUI_XY("W", 110, 27)
MUI_LABEL(2, 35, "BLE Scan")
MUI_XY("B", 110, 35)
MUI_LABEL(2, 43, "Flock Only")
MUI_XY("F", 110, 43)
MUI_XY("D", 2, 51)
MUI_XY("R", 2, 59)
;

// Cycles through display-off timer options on each hold-select
uint8_t mui_disptimer_cb(mui_t *ui, uint8_t msg) {
  uint8_t x = mui_get_x(ui);
  uint8_t y = mui_get_y(ui);
  char buf[20];
  snprintf(buf, sizeof(buf), "Disp off: %s", dispTimerLabels[displayTimerIdx]);
  switch (msg) {
    case MUIF_MSG_DRAW:
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(x, y, buf);
      break;
    case MUIF_MSG_CURSOR_ENTER:
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, y - 7, 128, 9);
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(x, y, buf);
      u8g2.setDrawColor(1);
      break;
    case MUIF_MSG_CURSOR_SELECT:
      displayTimerIdx = (displayTimerIdx + 1) % DISP_TIMER_COUNT;
      lastDisplayActivityMs = millis();  // Reset timer when user changes setting
      break;
  }
  return 0;
}

uint8_t mui_resume_cb(mui_t *ui, uint8_t msg) {
  uint8_t x = mui_get_x(ui);
  uint8_t y = mui_get_y(ui);
  switch (msg) {
    case MUIF_MSG_DRAW:
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(x, y, "  Resume Scan  ");
      break;
    case MUIF_MSG_CURSOR_ENTER:
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, y - 7, 128, 9);
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(x, y, "  Resume Scan  ");
      u8g2.setDrawColor(1);
      break;
    case MUIF_MSG_CURSOR_SELECT:
      // Settings already saved by caller before mui_SelectField()
      resumeScanning();
      break;
  }
  return 0;
}

// Board v1: no RGB LED

// Board v1: no battery ADC — percent fixed at 0
int batteryPercent = 0;

// Operating mode flags
bool fileSharingMode = false;
bool scanMode = false;
bool scanTasksStarted = false;
bool scanPaused = false;  // Scan paused for safe power-off (button hold in scan mode)
Preferences prefs;        // NVS key-value store for settings persistence
unsigned long gpsWaitStart = 0; // When GPS wait began (for elapsed time display)
int gpsWaitDotCount = 0;        // Dot animation counter for GPS wait console output
String fileSharingIP;
AsyncWebServer server(80);
bool serverRoutesConfigured = false;

// WiFi logo animation state (0 = dot only, 1-3 = number of rings)
int wifiAnimationState = 0;

// Rotary encoder state
volatile int8_t encoderDelta = 0;   // Accumulated rotation ticks (ISR-written)
unsigned long encSwPressStart = 0;
bool encSwPressed = false;

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

// Board v1: no RGB LED — LED functions are no-ops
void setLEDOff() {}
void ledInitializing() {}
void ledWaitingGPS() {}
void ledReady() {}
void ledFileSharing() {}

// Wake the display and reset the inactivity timer.
// Only called from the main loop (loop() context) — keeps SPI access single-threaded.
// Scan tasks signal a wake by writing lastDisplayActivityMs directly (atomic 32-bit write).
void wakeDisplay() {
  lastDisplayActivityMs = millis();
  if (displayCurrentlyOff && ENABLE_DISPLAY_OUTPUT) {
    u8g2.setPowerSave(0);
    displayCurrentlyOff = false;
    lastDisplayUpdate = 0;  // Force immediate redraw on next loop iteration
  }
}

// Load scanner settings from NVS (called once at boot)
void loadSettings() {
  prefs.begin("scout", true);  // true = read-only
  enableWifiScan   = prefs.getBool("wifi",       true);
  enableBleScan    = prefs.getBool("ble",        true);
  flockOnlyMode    = prefs.getBool("flockonly",  false);
  displayTimerIdx  = prefs.getUChar("disptimer", 0);
  if (displayTimerIdx >= DISP_TIMER_COUNT) displayTimerIdx = 0;
  prefs.end();
  consolePrintf("Settings loaded: WiFi=%s, BLE=%s, FlockOnly=%s, DispTimer=%s\n",
                enableWifiScan ? "ON" : "OFF",
                enableBleScan  ? "ON" : "OFF",
                flockOnlyMode  ? "ON" : "OFF",
                dispTimerLabels[displayTimerIdx]);
}

// Save scanner settings to NVS (called when user changes a toggle)
void saveSettings() {
  prefs.begin("scout", false);  // false = read-write
  prefs.putBool("wifi",       enableWifiScan);
  prefs.putBool("ble",        enableBleScan);
  prefs.putBool("flockonly",  flockOnlyMode);
  prefs.putUChar("disptimer", displayTimerIdx);
  prefs.end();
}

// Board v1: no battery ADC
int readBatteryPercent() { return 0; }

// Interrupt handler for rotary encoder (fires on any A-pin edge)
void IRAM_ATTR encoderISR() {
  bool a = digitalRead(ENC_A_PIN);
  bool b = digitalRead(ENC_B_PIN);
  if (a == b) encoderDelta++;   // CW  → next field
  else        encoderDelta--;   // CCW → prev field
}

// Boot screen: shows hardware initialisation status on OLED.
// Call after each init step with updated flags.
void drawBootScreen(bool sdOk, bool sdDone, bool gpsOk) {
  if (!ENABLE_DISPLAY_OUTPUT) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(14, 8,  "SignalScout v1");
  u8g2.drawStr(10, 16, "-- Board Test --");
  u8g2.drawHLine(0, 18, 128);

  // OLED row — always OK if we're drawing
  u8g2.drawStr(2,  30, "Display");
  u8g2.drawStr(74, 30, "[  OK   ]");

  // SD row
  u8g2.drawStr(2,  42, "SD Card");
  if (!sdDone)       u8g2.drawStr(74, 42, "[ INIT  ]");
  else if (sdOk)     u8g2.drawStr(74, 42, "[  OK   ]");
  else               u8g2.drawStr(74, 42, "[ FAIL  ]");

  // GPS row
  u8g2.drawStr(2,  54, "GPS");
  if (!gpsOk)        u8g2.drawStr(74, 54, "[ WAIT  ]");
  else               u8g2.drawStr(74, 54, "[  OK   ]");

  u8g2.sendBuffer();
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
// Scans WiFi then BLE sequentially. BLE is initialized once and kept alive across
// cycles — the ESP32-C5 hardware coexistence arbiter handles radio sharing automatically.
// Repeatedly calling BLEDevice::init()/deinit() every cycle corrupts radio state after
// 3-5 cycles, which is what was causing WiFi to hang. Keeping BLE initialized avoids this.
// Proactive WiFi mode reset interval (ms).
// Uses WiFi.mode() APIs only — NEVER call esp_wifi_start() inside this reset without
// immediately setting a mode. The normal scan setup always calls WiFi.mode(WIFI_OFF) right
// after the proactive block; if esp_wifi_start() was called here, that becomes a
// start→stop→start chain that corrupts the ESP32-C5 radio and is the root cause of WiFi
// dying at the ~5-minute mark. 15 minutes avoids constant churn while still refreshing state.
#define WIFI_DRIVER_RESET_INTERVAL_MS (15UL * 60UL * 1000UL)

void unifiedScanTask(void* parameter) {
  consolePrintln("[Scan Task] Unified scanner started");
  int wifiFailCount = 0;
  int wifiZeroApCount = 0;  // consecutive zero-AP cycles (silently failing driver)
  unsigned long lastWifiDriverReset = millis();

  while (true) {
    if (!logFileReady) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    consolePrintln("\n========== SCAN CYCLE START ==========");
    unsigned long cycleStart = millis();

    // ===== WIFI SCAN =====
    if (enableWifiScan) {
      consolePrintln("\n[WiFi] Initializing radio...");

      // Proactive mode reset: clear stale WiFi state using WiFi.mode() APIs only.
      // Do NOT call esp_wifi_start() here — the normal scan setup below calls
      // WiFi.mode(WIFI_OFF) immediately after, turning start→stop→start into a
      // start→stop→stop→start chain that corrupts the ESP32-C5 radio arbiter.
      if (millis() - lastWifiDriverReset >= WIFI_DRIVER_RESET_INTERVAL_MS) {
        consolePrintln("[WiFi] Scheduled proactive driver reset...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(500);  // Let driver fully stop before normal scan setup takes over
        wifiFailCount = 0;
        lastWifiDriverReset = millis();
        consolePrintln("[WiFi] Proactive reset complete");
      }

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(300);

      bool wifiReady = false;
      for (int attempt = 0; attempt < 3 && !wifiReady; attempt++) {
        if (WiFi.mode(WIFI_STA)) {
          // Clear stale AP list now that the driver is actually running
          esp_wifi_clear_ap_list();
          wifiReady = true;
        } else {
          consolePrintf("[WiFi] mode set failed (attempt %d/3), retrying...\n", attempt + 1);
          WiFi.mode(WIFI_OFF);
          delay(300);
        }
      }

      if (!wifiReady) {
        consolePrintln("[WiFi] WARNING: Radio init failed after 3 attempts, skipping this cycle");
        wifiFailCount++;
      } else {
        delay(300);
        wifiScanning = true;
        bool scanOk = scanWiFi();
        wifiScanning = false;

        if (scanOk) {
          wifiFailCount = 0;
          if (lastWiFiScanCount > 0) {
            wifiZeroApCount = 0;
          } else {
            // Scan succeeded but returned 0 APs — could be a silently broken driver
            wifiZeroApCount++;
            if (wifiZeroApCount >= 5) {
              consolePrintf("[WiFi] %d consecutive zero-AP scans — likely silent driver failure, forcing recovery\n", wifiZeroApCount);
              wifiFailCount = 3;  // trigger deep reset on next check
              wifiZeroApCount = 0;
            }
          }
        } else {
          wifiFailCount++;
          wifiZeroApCount = 0;
          consolePrintf("[WiFi] Scan failed (consecutive failures: %d)\n", wifiFailCount);
        }

        consolePrintln("[WiFi] Releasing radio...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(300);
      }

      // Deep recovery after repeated failures: force full ESP-IDF WiFi driver stop.
      // Do NOT call esp_wifi_start() here — WiFi.mode(WIFI_STA) in the next cycle's
      // retry loop will call it properly with a mode set. Calling start() here without
      // a mode leaves the driver in an indeterminate state and the subsequent
      // WiFi.mode(WIFI_OFF) immediately stops it again, corrupting radio state.
      if (wifiFailCount >= 3) {
        consolePrintln("[WiFi] Too many failures - deep radio reset...");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(200);
        esp_wifi_stop();  // ESP-IDF level stop for stubborn driver state
        delay(500);
        wifiFailCount = 0;
        lastWifiDriverReset = millis();
        consolePrintln("[WiFi] Deep reset complete");
      }
    }

    // ===== BLE SCAN =====
    if (enableBleScan) {
      consolePrintln("\n[BLE] Scanning...");

      // Initialize BLE if not already done (first run, or re-enabled via settings)
      if (!BLEDevice::getInitialized()) {
        consolePrintln("[BLE] Initializing stack...");
        BLEDevice::init("SignalScout");
        delay(200);
        pBLEScan = BLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks, false);
        pBLEScan->setActiveScan(true);
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(50);  // 50% duty cycle — leaves room for coexistence arbiter
      } else if (pBLEScan == NULL) {
        pBLEScan = BLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks, false);
        pBLEScan->setActiveScan(true);
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(50);
      }

      bleScanning = true;
      scanBluetooth();
      bleScanning = false;
      // Explicitly stop before clearResults — the blocking start() completes but the
      // BLE GAP state machine can linger and block WiFi radio acquisition next cycle.
      pBLEScan->stop();
      pBLEScan->clearResults();
      delay(500);  // Give the radio time to fully release after BLE
    } else if (BLEDevice::getInitialized()) {
      // User disabled BLE via settings - release the stack
      consolePrintln("[BLE] Disabled - releasing stack...");
      BLEDevice::deinit(true);
      pBLEScan = NULL;
      delay(300);
    }

    unsigned long cycleDuration = millis() - cycleStart;

    size_t freeHeap = ESP.getFreeHeap();
    UBaseType_t scanStackWatermark = uxTaskGetStackHighWaterMark(NULL);
    if (freeHeap < 30000) {
      consolePrintf("WARNING: Low memory! Free heap: %u bytes\n", freeHeap);
    }
    if (scanStackWatermark < 1024) {
      consolePrintf("WARNING: Scan task stack nearly full! Watermark: %u bytes\n", scanStackWatermark * 4);
    }

    consolePrintf("========== SCAN CYCLE COMPLETE (%lu ms, heap: %u, stack_wm: %u) ==========\n",
                  cycleDuration, freeHeap, scanStackWatermark * 4);

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
  delay(500);

  consolePrintln("\n\n=== BOOT START (Board v1) ===");
  consolePrintln("SignalScout - WiFi & Bluetooth Scanner with GPS");

  // ── Step 0: Shared SPI bus init (SD + OLED share GPIO 8/10) ──────────────
  // Must happen before OLED begin() so u8g2 HW SPI uses the correct pins.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  delay(100);

  // ── Step 1: OLED first — so we can show a boot status screen ─────────────
  consolePrintln("[1/6] Initializing OLED display...");
  if (ENABLE_DISPLAY_OUTPUT) {
    if (!u8g2.begin()) {
      consolePrintln("ERROR: u8g2 display init failed!");
    } else {
      consolePrintln("Display initialized");
    }
  }
  drawBootScreen(false, false, false);  // Show initial state before SD attempt

  // ── Step 2: FreeRTOS resources ────────────────────────────────────────────
  consolePrintln("[2/6] Initializing FreeRTOS resources...");
  deviceMapMutex = xSemaphoreCreateMutex();
  sdCardMutex = xSemaphoreCreateMutex();
  logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogEntry));
  if (deviceMapMutex == NULL || sdCardMutex == NULL || logQueue == NULL) {
    consolePrintln("ERROR: Failed to create FreeRTOS resources!");
    while(1);
  }
  consolePrintln("FreeRTOS mutexes and queue created");

  // ── Step 3: Rotary encoder ────────────────────────────────────────────────
  consolePrintln("[3/6] Initializing rotary encoder...");
  pinMode(ENC_A_PIN,  INPUT);  // External pull-ups on PCB
  pinMode(ENC_B_PIN,  INPUT);
  pinMode(ENC_SW_PIN, INPUT);  // External pull-up on PCB
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  consolePrintln("Encoder initialized (A=GPIO1, B=GPIO2, SW=GPIO12)");

  // ── Step 4: SD card ───────────────────────────────────────────────────────
  consolePrintln("[4/6] Initializing SD card (SPI, shared bus)...");
  if (ENABLE_LOG_OUTPUT) {
    consolePrintf("SD pins: CS=%d, MOSI=%d, MISO=%d, SCK=%d\n", SD_CS, SD_MOSI, SD_MISO, SD_SCK);

    uint32_t spiSpeeds[] = {1000000, 2000000, 4000000, 8000000};
    int retries = 5, speedIndex = 0;

    while (retries > 0 && !sdCardMounted) {
      uint32_t spd = spiSpeeds[speedIndex % 4];
      consolePrintf("Trying SD at %lu Hz (attempt %d/5)...\n", spd, 6 - retries);
      if (SD.begin(SD_CS, SPI, spd, "/sd", 5, false)) {
        File testFile = SD.open("/test.tmp", FILE_WRITE);
        if (testFile) {
          testFile.println("test");
          testFile.close();
          SD.remove("/test.tmp");
          sdCardMounted = true;
          consolePrintf("SD mounted at %lu Hz\n", spd);
        } else {
          consolePrintln("WARNING: SD write test failed");
          SD.end();
        }
      }
      if (!sdCardMounted) { retries--; speedIndex++; SD.end(); delay(500); }
    }
    if (!sdCardMounted) consolePrintln("SD init failed — continuing without logging");
  } else {
    consolePrintln("SD logging disabled in config");
  }
  drawBootScreen(sdCardMounted, true, false);  // SD result now known

  // ── Step 5: RTC + GPS UART ────────────────────────────────────────────────
  consolePrintln("[5/6] Checking RTC + starting GPS UART...");
  if (rtc.getYear() > 2020) {
    consolePrintf("RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtc.getYear(), rtc.getMonth()+1, rtc.getDay(),
                  rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
  } else {
    consolePrintln("No valid RTC time stored");
  }
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  consolePrintf("GPS UART started (RX=GPIO%d, TX=GPIO%d)\n", GPS_RX, GPS_TX);

  // ── Step 6: Load settings + watchdog ─────────────────────────────────────
  consolePrintln("[6/6] Loading saved settings...");
  loadSettings();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,
    .idle_core_mask = 0,
    .trigger_panic = false
  };
  esp_task_wdt_reconfigure(&wdt_config);
  consolePrintln("Task watchdog reconfigured (30s, no panic)");
  consolePrintln("=== INIT COMPLETE ===");

  // ── Boot mode selection: hold encoder button for file-share mode ──────────
  // Show 2-second window; boot screen remains visible during this time.
  if (ENABLE_DISPLAY_OUTPUT) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2,  8, "SignalScout v1");
    u8g2.drawHLine(0, 10, 128);
    u8g2.drawStr(2, 22, "Hold enc btn:");
    u8g2.drawStr(2, 32, "  file share mode");
    u8g2.drawStr(2, 44, "Release:");
    u8g2.drawStr(2, 54, "  scan mode (default)");
    u8g2.sendBuffer();
  }
  delay(2000);

  if (digitalRead(ENC_SW_PIN) == LOW) {
    consolePrintln("Encoder button held — entering FILE SHARING mode");
    while (digitalRead(ENC_SW_PIN) == LOW) delay(50);
    encSwPressed = false;
    enterFileSharingMode();
  } else {
    consolePrintln("Entering SCAN mode — waiting for GPS...");
    scanMode = true;
    gpsWaitStart = millis();
  }
}

void loop() {
  // Main loop handles GPS reading, battery monitoring, display updates, and button handling
  // All scanning is handled by FreeRTOS tasks

  unsigned long currentTime = millis();

  // Read GPS data continuously (limit reads to prevent blocking)
  int gpsReadCount = 0;
  while (gpsSerial.available() > 0 && gpsReadCount < 100) {
    char c = gpsSerial.read();
    gps.encode(c);
    gpsReadCount++;
#if GPS_DEBUG
    Serial.write(c);  // Echo raw NMEA to serial monitor
#endif
  }

#if GPS_DEBUG
  // Print TinyGPSPlus stats every 5 seconds
  static unsigned long lastGpsDebug = 0;
  if (currentTime - lastGpsDebug >= 5000) {
    lastGpsDebug = currentTime;
    Serial.println(F("\n--- GPS Debug ---"));
    Serial.print(F("Chars received: "));    Serial.println(gps.charsProcessed());
    Serial.print(F("Sentences with fix: ")); Serial.println(gps.sentencesWithFix());
    Serial.print(F("Failed checksum: "));   Serial.println(gps.failedChecksum());
    Serial.print(F("Location valid: "));    Serial.println(gps.location.isValid() ? "YES" : "NO");
    Serial.print(F("Time valid: "));        Serial.println(gps.time.isValid() ? "YES" : "NO");
    Serial.print(F("Date valid: "));        Serial.println(gps.date.isValid() ? "YES" : "NO");
    Serial.print(F("Satellites: "));        Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "N/A");
    Serial.print(F("HDOP: "));              Serial.println(gps.hdop.isValid() ? String(gps.hdop.hdop(), 2) : "N/A");
    if (gps.charsProcessed() < 10) {
      Serial.println(F("!! NO DATA from GPS - check wiring and power"));
    } else if (gps.sentencesWithFix() == 0 && gps.charsProcessed() > 100) {
      Serial.println(F("!! Receiving data but no fix yet - check antenna sky view"));
    }
    Serial.println(F("-----------------"));
    Serial.flush();
  }
#endif

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
      cached_flock_total = uniqueFlockCount;
      xSemaphoreGive(deviceMapMutex);
      lastCountsCacheUpdate = currentTime;
    }
  }

  // Display off timer: sleep/wake based on inactivity.
  // Scan tasks signal a wake by writing lastDisplayActivityMs (atomic 32-bit write).
  if (ENABLE_DISPLAY_OUTPUT && dispTimerMs[displayTimerIdx] > 0) {
    bool shouldBeOff = (currentTime - lastDisplayActivityMs >= dispTimerMs[displayTimerIdx]);
    if (shouldBeOff && !displayCurrentlyOff) {
      u8g2.setPowerSave(1);
      displayCurrentlyOff = true;
    } else if (!shouldBeOff && displayCurrentlyOff) {
      // Woken by Flock detection (scan task updated lastDisplayActivityMs)
      u8g2.setPowerSave(0);
      displayCurrentlyOff = false;
      lastDisplayUpdate = 0;  // Force immediate redraw
    }
  }

  // Update display periodically - faster refresh during Flock alert for smooth blinking
  unsigned long displayInterval = (flockAlertUntilMs > currentTime) ? 250 : DISPLAY_UPDATE_INTERVAL;
  if (ENABLE_DISPLAY_OUTPUT && !displayCurrentlyOff && currentTime - lastDisplayUpdate >= displayInterval) {
    if (scanPaused) {
      updateDisplayPaused();
    } else if (fileSharingMode) {
      updateDisplayFileSharing();
    } else if (scanMode && !scanTasksStarted) {
      updateDisplayGPSWait();
    } else {
      updateDisplay("");
    }
    lastDisplayUpdate = currentTime;
  }

  // ── Rotary encoder: rotation ──────────────────────────────────────────────
  // encoderDelta is written by ISR; read atomically here and reset.
  int8_t delta = encoderDelta;
  if (delta != 0) {
    encoderDelta = 0;
    wakeDisplay();
    if (scanMode && scanPaused) {
      if (delta > 0) {
        mui_NextField(&mui);
      } else {
        mui_PrevField(&mui);
      }
      updateDisplayPaused();
    }
  }

  // ── Rotary encoder: button (ENC_SW active-LOW) ────────────────────────────
  bool encSwState = (digitalRead(ENC_SW_PIN) == LOW);
  if (encSwState && !encSwPressed) {
    encSwPressed = true;
    encSwPressStart = currentTime;
    wakeDisplay();
  } else if (!encSwState && encSwPressed) {
    unsigned long holdDuration = currentTime - encSwPressStart;
    bool tapDetected  = (holdDuration < ENC_HOLD_TIME);
    bool holdDetected = (holdDuration >= ENC_HOLD_TIME);
    if (scanMode && scanTasksStarted) {
      if (!scanPaused) {
        if (holdDetected) pauseScanning();
      } else {
        // In menu: any press activates the focused item (rotation handles navigation)
        if (tapDetected || holdDetected) {
          saveSettings();
          mui_SendSelect(&mui);
          if (scanPaused) updateDisplayPaused();
        }
      }
    }
    encSwPressed = false;
  }

  // Minimal delay to prevent tight loop but maintain responsiveness (10ms = 100Hz loop)
  delay(10);
}


// Pause scanning for safe power-off: suspends scan task, drains log queue,
// waits for the last SD write to finish, then suspends the SD task.
void pauseScanning() {
  consolePrintln("\n=== PAUSING SCAN - flushing SD card ===");

  // Immediate display feedback before the queue-drain wait
  if (ENABLE_DISPLAY_OUTPUT) {
    u8g2.clearBuffer();
    drawStatusBar();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 22, "Pausing...");
    u8g2.drawStr(2, 32, "Flushing SD card");
    u8g2.sendBuffer();
  }

  // Stop new scans immediately
  if (unifiedScanTaskHandle != NULL) vTaskSuspend(unifiedScanTaskHandle);
  wifiScanning = false;
  bleScanning = false;

  // Queue a final "paused" marker before draining (sdLogTask still running)
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Scanning paused - safe to power off");
  }

  // Wait for queue to drain (sdLogTask processes any remaining entries)
  unsigned long waitStart = millis();
  while (uxQueueMessagesWaiting(logQueue) > 0 && millis() - waitStart < 10000) {
    delay(100);
  }
  delay(300);  // Final settle time for last SD write to complete

  // Now safe to suspend SD task
  if (sdLogTaskHandle != NULL) vTaskSuspend(sdLogTaskHandle);

  scanPaused = true;
  // Initialize MUI settings form
  mui_Init(&mui, &u8g2, fds_settings, muif_list, sizeof(muif_list) / sizeof(muif_list[0]));
  mui_GotoForm(&mui, 1, 0);
  setLEDOff();
  consolePrintln("Scan paused - safe to power off. Hold button 1s to resume.");
}

void resumeScanning() {
  consolePrintln("\n=== RESUMING SCAN ===");
  if (sdLogTaskHandle != NULL) vTaskResume(sdLogTaskHandle);
  if (unifiedScanTaskHandle != NULL) vTaskResume(unifiedScanTaskHandle);
  if (ENABLE_LOG_OUTPUT && logFileReady) {
    logToFile("Scanning resumed");
  }
  scanPaused = false;
  consolePrintln("Scanning resumed.");
}

void updateDisplayPaused() {
  if (!ENABLE_DISPLAY_OUTPUT) return;
  u8g2.clearBuffer();
  drawStatusBar();
  u8g2.setFont(u8g2_font_5x8_tf);
  mui_Draw(&mui);
  u8g2.sendBuffer();
}

// Format file size for display in file browser
String formatFileSize(size_t size) {
  if (size < 1024) return String(size) + " B";
  if (size < 1024 * 1024) return String((float)size / 1024.0, 1) + " KB";
  return String((float)size / (1024.0 * 1024.0), 1) + " MB";
}

// HTML-escape a filename for use in HTML attribute values and text content.
// SD card filenames are firmware-generated and shouldn't contain these chars, but
// defense-in-depth prevents XSS if someone physically inserts a crafted card.
void htmlEscapeFilename(const char* src, char* dst, size_t dstSize) {
  size_t out = 0;
  for (size_t i = 0; src[i] && out + 7 < dstSize; i++) {
    char c = src[i];
    if      (c == '<')  { memcpy(dst + out, "&lt;",   4); out += 4; }
    else if (c == '>')  { memcpy(dst + out, "&gt;",   4); out += 4; }
    else if (c == '&')  { memcpy(dst + out, "&amp;",  5); out += 5; }
    else if (c == '"')  { memcpy(dst + out, "&quot;", 6); out += 6; }
    else if (c == '\'') { memcpy(dst + out, "&#39;",  5); out += 5; }
    else                { dst[out++] = c; }
  }
  dst[out] = '\0';
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
                    "*{box-sizing:border-box;}"
                    "html,body{height:100%;margin:0;padding:0;}"
                    "body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;"
                    "display:flex;flex-direction:column;min-height:100vh;padding:16px 20px;}"
                    "h1{color:#00d4ff;margin:0 0 8px 0;font-size:1.4em;}"
                    ".stats{background:#16213e;border:1px solid #333;border-radius:8px;"
                    "padding:10px 14px;margin:0 0 10px 0;display:flex;gap:20px;flex-wrap:wrap;}"
                    ".stat{display:flex;flex-direction:column;}"
                    ".stat-val{color:#00d4ff;font-size:1.1em;font-weight:bold;}"
                    ".stat-lbl{color:#888;font-size:0.7em;}"
                    ".bar{background:#333;border-radius:4px;height:6px;width:100px;margin-top:3px;}"
                    ".bar-fill{background:#00d4ff;border-radius:4px;height:100%;}"
                    "a{color:#00d4ff;text-decoration:none;}"
                    "a:hover{text-decoration:underline;}"
                    ".tbl-wrap{flex:1;overflow-y:auto;margin:0 0 10px 0;}"
                    "table{width:100%;border-collapse:collapse;}"
                    "th{position:sticky;top:0;background:#16213e;color:#888;font-size:0.75em;"
                    "text-transform:uppercase;letter-spacing:0.5px;text-align:left;"
                    "padding:8px 10px;border-bottom:2px solid #00d4ff;}"
                    "td{padding:7px 10px;border-bottom:1px solid #2a2a3e;}"
                    "tr:hover{background:#16213e;}"
                    ".sz{color:#888;white-space:nowrap;}"
                    ".dt{color:#888;white-space:nowrap;font-size:0.85em;}"
                    ".del{color:#ff4444;background:none;border:1px solid #ff4444;"
                    "border-radius:4px;padding:2px 8px;cursor:pointer;font-size:0.8em;white-space:nowrap;}"
                    ".del:hover{background:#ff4444;color:#fff;}"
                    ".empty{color:#666;font-style:italic;}"
                    ".msg{padding:10px 14px;border-radius:6px;margin:0 0 10px 0;"
                    "background:#1a3a1a;border:1px solid #2a5a2a;color:#6f6;}"
                    ".msg-err{background:#3a1a1a;border-color:#5a2a2a;color:#f66;}"
                    ".nav{display:flex;gap:8px;align-items:center;flex-wrap:wrap;}"
                    ".nav a,.nav span{padding:5px 12px;border:1px solid #444;border-radius:4px;font-size:0.9em;}"
                    ".nav .cur{background:#00d4ff;color:#1a1a2e;border-color:#00d4ff;}"
                    ".foot{margin-top:auto;padding-top:10px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;}"
                    ".delall{padding:8px 16px;background:none;"
                    "border:1px solid #ff4444;color:#ff4444;border-radius:6px;"
                    "cursor:pointer;font-size:0.85em;}"
                    ".delall:hover{background:#ff4444;color:#fff;}"
                    ".heap{color:#555;font-size:0.7em;}"
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

    if (totalFiles == 0) {
      response->print("<p class=\"empty\">No files found on SD card.</p>");
    } else {
      response->print("<div class=\"tbl-wrap\"><table>"
                      "<thead><tr><th>Name</th><th>Date</th><th>Size</th><th></th></tr></thead>"
                      "<tbody>");
      // Second pass: output only the files for this page
      if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        File root2 = SD.open("/");
        if (root2) {
          File file = root2.openNextFile();
          while (file) {
            if (!file.isDirectory()) {
              if (fileIndex >= skipCount && filesShown < FILES_PER_PAGE) {
                char rowBuf[512];
                const char* fname = file.name();
                String sizeStr = formatFileSize(file.size());
                // Get last write time
                time_t t = file.getLastWrite();
                char dateBuf[20] = "-";
                if (t > 0) {
                  struct tm *tm = localtime(&t);
                  if (tm) {
                    snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d %02d:%02d",
                             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                             tm->tm_hour, tm->tm_min);
                  }
                }
                char fnameEsc[128];
                htmlEscapeFilename(fname, fnameEsc, sizeof(fnameEsc));
                snprintf(rowBuf, sizeof(rowBuf),
                         "<tr><td><a href=\"/download?name=%s\">%s</a></td>"
                         "<td class=\"dt\">%s</td>"
                         "<td class=\"sz\">%s</td>"
                         "<td><button class=\"del\" onclick=\"delFile('%s')\">Delete</button></td></tr>",
                         fnameEsc, fnameEsc, dateBuf, sizeStr.c_str(), fnameEsc);
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
      response->print("</tbody></table></div>");
    }

    // Footer: delete all (left) + pagination (right), pinned to bottom
    response->print("<div class=\"foot\">");

    // Delete all button
    if (totalFiles > 0) {
      char delAllBuf[128];
      snprintf(delAllBuf, sizeof(delAllBuf),
               "<button class=\"delall\" onclick=\"delAll(%d)\">Delete All (%d)</button>",
               totalFiles, totalFiles);
      response->print(delAllBuf);
    } else {
      response->print("<div></div>");  // spacer
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

    response->print("</div>");  // end foot

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

  // File download - streams file from SD card using per-chunk open/seek/close
  // This avoids holding an SD file handle across async TCP callbacks, which crashes
  // on ESP32 with large files (AsyncFileResponse is not safe for FreeRTOS multi-task).
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("name")) {
      request->send(400, "text/plain", "Missing filename");
      return;
    }
    String filename = request->getParam("name")->value();
    // Block path traversal attempts
    if (filename.indexOf("..") >= 0 || filename.indexOf("/") >= 0) {
      request->send(400, "text/plain", "Invalid filename");
      return;
    }
    String filepath = "/" + filename;

    // Get file size before starting the response
    size_t fileSize = 0;
    if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      File f = SD.open(filepath.c_str(), FILE_READ);
      if (f) {
        fileSize = f.size();
        f.close();
      }
      xSemaphoreGive(sdCardMutex);
    }
    if (fileSize == 0) {
      request->send(404, "text/plain", "File not found or empty");
      return;
    }

    // Stream with per-chunk mutex-protected reads: open, seek to index, read, close
    // Avoids holding a File handle across async callbacks (causes crashes on large files)
    String fp = filepath;
    AsyncWebServerResponse *response = request->beginResponse(
      "application/octet-stream", fileSize,
      [fp](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        size_t bytesRead = 0;
        if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          File f = SD.open(fp.c_str(), FILE_READ);
          if (f) {
            f.seek(index);
            bytesRead = f.read(buffer, maxLen);
            f.close();
          }
          xSemaphoreGive(sdCardMutex);
        }
        return bytesRead;
      }
    );
    response->addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    request->send(response);
  });

  // Delete a single file
  server.on("/delete", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("name")) {
      request->send(400, "text/plain", "Missing filename");
      return;
    }
    String filename = request->getParam("name")->value();
    if (filename.indexOf("..") >= 0 || filename.indexOf("/") >= 0) {
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

      // Process in batches to limit memory usage. Cap at 100 batches to prevent
      // an infinite loop if a file can't be deleted (e.g. SD write-protected).
      bool moreFiles = true;
      int batchLimit = 100;
      while (moreFiles && batchLimit-- > 0) {
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

  // Immediate display feedback before radio init delays
  if (ENABLE_DISPLAY_OUTPUT) {
    u8g2.clearBuffer();
    drawStatusBar();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 22, "File Share Mode");
    u8g2.drawStr(2, 32, "Loading...");
    u8g2.sendBuffer();
  }

  // Suspend scanning and logging tasks to avoid SD and radio conflicts (if they exist)
  if (unifiedScanTaskHandle != NULL) vTaskSuspend(unifiedScanTaskHandle);
  if (sdLogTaskHandle != NULL) vTaskSuspend(sdLogTaskHandle);

  // Deinitialize BLE to free up 2.4GHz radio for WiFi (if it was initialized)
  // BLE and WiFi share radio resources on ESP32-C5
  // The unified scan task releases radios after each scan, but we double-check here
  if (enableBleScan && BLEDevice::getInitialized()) {
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
    u8g2.clearBuffer();
    drawStatusBar();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(2, 22, "File Share Mode");
    u8g2.drawStr(2, 32, "Connecting...");
    u8g2.sendBuffer();
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
      u8g2.clearBuffer();
      drawStatusBar();
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(2, 22, "WiFi Connect");
      u8g2.drawStr(2, 32, "FAILED - check");
      u8g2.drawStr(2, 42, "secrets.h");
      u8g2.sendBuffer();
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
  if (ENABLE_LOG_OUTPUT && sdCardMounted && logFileReady) {  // ENABLE_LOG_OUTPUT is still a #define
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
  // Always create the task; it reads enableWifiScan/enableBleScan at runtime each cycle
  if (unifiedScanTaskHandle == NULL) {
    BaseType_t result = xTaskCreate(
      unifiedScanTask,       // Task function
      "Unified Scanner",     // Task name
      20480,                 // Stack size - BLE library needs significant stack space
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

  ledReady();  // Green: Ready
  delay(1000);  // Brief pause to show green LED
  setLEDOff();  // Turn off LED to conserve battery

  scanTasksStarted = true;
  consolePrintln("Scan tasks started!\n");
}


void updateDisplayFileSharing() {
  if (!ENABLE_DISPLAY_OUTPUT) return;
  u8g2.clearBuffer();
  drawStatusBar();
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(2, 12 + FONT_LG_ASCENT, "FILE SHARE");  // baseline y=26
  u8g2.setFont(u8g2_font_5x8_tf);
  {
    char ipBuf[40];
    snprintf(ipBuf, sizeof(ipBuf), "IP: %s", fileSharingIP.c_str());
    u8g2.drawStr(2, 38, ipBuf);
  }
  u8g2.drawStr(2, 48, "Power off when done");
  u8g2.drawStr(2, 57, "Boot normally: scan");
  u8g2.sendBuffer();
}

void updateDisplayGPSWait() {
  gpsWaitDotCount = (gpsWaitDotCount + 1) % 4;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  unsigned long elapsed = (millis() - gpsWaitStart) / 1000;
  consolePrintf("Waiting for GPS fix (Satellites: %d, Elapsed: %lus)\n", sats, elapsed);
  if (!ENABLE_DISPLAY_OUTPUT) return;
  u8g2.clearBuffer();
  drawStatusBar();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(2, 22, "Waiting for GPS...");
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "Sats: %d  Time: %lus", sats, elapsed);
    u8g2.drawStr(2, 32, buf);
  }
  u8g2.sendBuffer();
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

// --- Flock Safety Device Detection ---
// Signatures sourced from github.com/MaxwellDPS/Flock-You-Android
// and github.com/justcallmekoko/ESP32Marauder

static bool startsWithCI(const char* str, const char* prefix) {
  return strncasecmp(str, prefix, strlen(prefix)) == 0;
}

// Returns true if WiFi AP matches Flock Safety signatures (SSID or OUI)
bool isFlockWiFi(const char* ssid, uint8_t* bssid) {
  // SSID patterns (case-insensitive prefix match)
  if (startsWithCI(ssid, "flock"))        return true;
  if (startsWithCI(ssid, "fs-"))          return true;
  if (startsWithCI(ssid, "fs_"))          return true;
  if (startsWithCI(ssid, "falcon"))       return true;
  if (startsWithCI(ssid, "sparrow"))      return true;
  if (startsWithCI(ssid, "condor"))       return true;
  if (startsWithCI(ssid, "penguin"))      return true;   // Flock external battery AP
  if (startsWithCI(ssid, "pigvision"))    return true;   // Flock camera firmware variant
  if (startsWithCI(ssid, "fs ext batt")) return true;   // "FS Ext Battery" AP name

  // OUI lookup — inline byte comparison is faster than a loop on embedded hardware.
  // Sources: colonelpanichacks/flock-you (31 field-verified OUIs), MaxwellDPS/Flock-You-Android,
  //          justcallmekoko/ESP32Marauder, and FCC filings.
  uint8_t b0=bssid[0], b1=bssid[1], b2=bssid[2];

  // --- colonelpanichacks/flock-you field-verified OUI list (31 entries) ---
  if (b0==0x70 && b1==0xC9 && b2==0x4E) return true;
  if (b0==0x3C && b1==0x91 && b2==0x80) return true;
  if (b0==0xD8 && b1==0xF3 && b2==0xBC) return true;
  if (b0==0x80 && b1==0x30 && b2==0x49) return true;
  if (b0==0xB8 && b1==0x35 && b2==0x32) return true;
  if (b0==0x14 && b1==0x5A && b2==0xFC) return true;
  if (b0==0x74 && b1==0x4C && b2==0xA1) return true;
  if (b0==0x08 && b1==0x3A && b2==0x88) return true;
  if (b0==0x9C && b1==0x2F && b2==0x9D) return true;
  if (b0==0xC0 && b1==0x35 && b2==0x32) return true;
  if (b0==0x94 && b1==0x08 && b2==0x53) return true;
  if (b0==0xE4 && b1==0xAA && b2==0xEA) return true;
  if (b0==0xF4 && b1==0x6A && b2==0xDD) return true;
  if (b0==0xF8 && b1==0xA2 && b2==0xD6) return true;
  if (b0==0x24 && b1==0xB2 && b2==0xB9) return true;
  if (b0==0x00 && b1==0xF4 && b2==0x8D) return true;
  if (b0==0xD0 && b1==0x39 && b2==0x57) return true;
  if (b0==0xE8 && b1==0xD0 && b2==0xFC) return true;
  if (b0==0xE0 && b1==0x4F && b2==0x43) return true;
  if (b0==0xB8 && b1==0x1E && b2==0xA4) return true;
  if (b0==0x70 && b1==0x08 && b2==0x94) return true;
  if (b0==0x58 && b1==0x8E && b2==0x81) return true;
  if (b0==0xEC && b1==0x1B && b2==0xBD) return true;
  if (b0==0x3C && b1==0x71 && b2==0xBF) return true;
  if (b0==0x58 && b1==0x00 && b2==0xE3) return true;
  if (b0==0x90 && b1==0x35 && b2==0xEA) return true;
  if (b0==0x5C && b1==0x93 && b2==0xA2) return true;
  if (b0==0x64 && b1==0x6E && b2==0x69) return true;
  if (b0==0x48 && b1==0x27 && b2==0xEA) return true;
  if (b0==0xA4 && b1==0xCF && b2==0x12) return true;
  if (b0==0x82 && b1==0x6B && b2==0xF2) return true;  // DeFlockJoplin field research

  // --- Additional OUIs from MaxwellDPS/ESP32Marauder research ---
  // Quectel LTE modem (primary Flock modem manufacturer)
  if (b0==0x50 && b1==0x29 && b2==0x4D) return true;
  if (b0==0x86 && b1==0x25 && b2==0x19) return true;
  // Telit LTE modem (alternate Flock modem)
  if (b0==0x00 && b1==0x14 && b2==0x2D) return true;
  if (b0==0xD8 && b1==0xC7 && b2==0x71) return true;
  // OUI registered directly to Flock Safety (FCC filing)
  if (b0==0xB4 && b1==0x1E && b2==0x52) return true;

  return false;
}

// Returns true if BLE device matches Flock Safety signatures (name, service UUID, or manufacturer data)
// manufDataHex: hex string of raw manufacturer data bytes (first 2 bytes = company ID, little-endian)
bool isFlockBLE(const char* name, const char* serviceUUID, const char* manufDataHex) {
  // BLE name patterns (case-insensitive prefix match)
  if (startsWithCI(name, "flock"))        return true;
  if (startsWithCI(name, "falcon"))       return true;
  if (startsWithCI(name, "raven"))        return true;  // Raven acoustic sensor
  if (startsWithCI(name, "penguin"))      return true;  // Flock Penguin external battery (any variant)
  if (startsWithCI(name, "pigvision"))   return true;  // Flock/Pigvision camera firmware
  if (startsWithCI(name, "fs ext batt")) return true;  // Flock external battery (legacy name)
  if (startsWithCI(name, "soundthinking")) return true; // SoundThinking (parent company) rebranding
  if (startsWithCI(name, "shotspotter"))  return true;  // ShotSpotter acoustic sensor (SoundThinking)
  // Raven custom BLE service UUIDs (firmware 1.2.x+)
  if (serviceUUID && strstr(serviceUUID, "00003100") != NULL) return true;  // GPS Location
  if (serviceUUID && strstr(serviceUUID, "00003200") != NULL) return true;  // Power Management
  if (serviceUUID && strstr(serviceUUID, "00003300") != NULL) return true;  // Network Status
  if (serviceUUID && strstr(serviceUUID, "00003400") != NULL) return true;  // Upload Statistics
  if (serviceUUID && strstr(serviceUUID, "00003500") != NULL) return true;  // Error/Diagnostics
  // Legacy Raven BLE service UUIDs (firmware 1.1.x)
  if (serviceUUID && strstr(serviceUUID, "00001809") != NULL) return true;  // Health Thermometer profile
  if (serviceUUID && strstr(serviceUUID, "00001819") != NULL) return true;  // Location/Navigation profile
  // Xuntong company ID 0x09C8 (LE bytes: C8 09) — Flock Penguin ODM manufacturer
  if (manufDataHex && startsWithCI(manufDataHex, "C809")) return true;
  return false;
}

// Returns true on success, false if scan could not be started or records could not be read.
bool scanWiFi() {
  consolePrintln("\n--- WiFi Scan Starting (2.4GHz + 5GHz) ---");

  wifi_scan_config_t scan_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,  // 0 = scan all channels
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = {
      .active = {
        .min = 120,
        .max = 150
      },
      .passive = 400
    }
  };

  // Start scan (blocking mode)
  esp_err_t err = esp_wifi_scan_start(&scan_config, true);
  if (err != ESP_OK) {
    consolePrintf("[WiFi] scan_start failed: %d\n", err);
    lastWiFiScanCount = 0;
    return false;
  }

  // Static buffer avoids malloc/free every cycle, which causes heap fragmentation
  // over long sessions and eventually causes malloc to fail (silently killing WiFi scans).
  // 64 APs is plenty for mobile scanning; extras are silently truncated.
  #define MAX_AP_RECORDS 64
  static wifi_ap_record_t ap_records[MAX_AP_RECORDS];

  // Pass buffer size as input; function fills it and writes actual count as output.
  // This also frees the driver's internal AP list (unlike esp_wifi_scan_get_ap_num).
  uint16_t ap_count = MAX_AP_RECORDS;
  err = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
  if (err != ESP_OK) {
    consolePrintf("[WiFi] get_ap_records failed: %d\n", err);
    esp_wifi_clear_ap_list();
    lastWiFiScanCount = 0;
    return false;
  }
  lastWiFiScanCount = ap_count;

  consolePrintf("Networks found: %d\n", ap_count);

  if (ap_count == 0) {
    consolePrintln("No networks found");
    return true;  // Scan succeeded, just no results
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

    // Flock Safety detection
    bool isFlock = isFlockWiFi(ssid.c_str(), ap->bssid);
    if (isFlock) flockLastSeenMs = millis();

    // In Flock-only mode, skip all non-Flock devices entirely
    if (flockOnlyMode && !isFlock) continue;

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
      // Flock dedup uses a separate never-cleared set so map eviction doesn't re-trigger alerts
      if (isFlock && seenFlockKeys.find(bssidKey) == seenFlockKeys.end()) {
        seenFlockKeys.insert(bssidKey);
        uniqueFlockCount++;
        flockAlertUntilMs = millis() + 3000;
        lastDisplayActivityMs = millis();  // Wake display for Flock alert
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
      safeCopy(entry.type, sizeof(entry.type), isFlock ? "FLOCK-WIFI" : "WIFI");
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

  consolePrintln("--- WiFi Scan Complete ---\n");
  return true;
}

void scanBluetooth() {
  consolePrintln("\n--- Bluetooth Scan Starting ---");

  BLEScanResults* foundDevices = pBLEScan->start(BLE_SCAN_TIME, false);
  if (foundDevices == NULL) {
    consolePrintln("ERROR: BLE scan returned NULL results, skipping");
    lastBLEScanCount = 0;
    return;
  }
  int deviceCount = foundDevices->getCount();
  lastBLEScanCount = deviceCount;  // Track last scan count

  consolePrintf("Bluetooth devices found: %d\n", deviceCount);

  for (int i = 0; i < deviceCount; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);

    String address = device.getAddress().toString().c_str();
    String name = device.haveName() ? device.getName().c_str() : "Unknown";
    int rssi = device.getRSSI();

    // Get additional info (extracted before mutex so Flock detection has full data)
    String manufData = "";
    String serviceUUID = "";
    if (device.haveManufacturerData()) {
      String rawData = device.getManufacturerData();
      char hexStr[65] = {0};
      for (size_t i = 0; i < rawData.length() && i < 32; i++) {
        snprintf(hexStr + (i * 2), 3, "%02X", (uint8_t)rawData[i]);
      }
      manufData = String(hexStr);
    }
    if (device.haveServiceUUID()) {
      serviceUUID = device.getServiceUUID().toString().c_str();
    }

    // Flock Safety detection (full name + UUID + manufacturer data check)
    bool isFlock = isFlockBLE(name.c_str(), serviceUUID.c_str(), manufData.c_str());
    if (isFlock) flockLastSeenMs = millis();

    // In Flock-only mode, skip all non-Flock devices entirely
    if (flockOnlyMode && !isFlock) continue;

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
      // Flock dedup uses a separate never-cleared set so map eviction doesn't re-trigger alerts
      if (isFlock && seenFlockKeys.find(addrKey) == seenFlockKeys.end()) {
        seenFlockKeys.insert(addrKey);
        uniqueFlockCount++;
        flockAlertUntilMs = millis() + 3000;
        lastDisplayActivityMs = millis();  // Wake display for Flock alert
      }
      xSemaphoreGive(deviceMapMutex);
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
      safeCopy(entry.type, sizeof(entry.type), isFlock ? "FLOCK-BLE" : "BLE");
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

// Status bar: GPS bars + sat count (left) | compass (centre) | battery + % (right)
// Occupies y=0-10; draws separator line at y=11. Used by every screen.
void drawStatusBar() {
  u8g2.setFont(u8g2_font_5x8_tf);

  // GPS signal bars — 5 ascending bars, bottom-aligned at y=10
  int satCount = gps.satellites.isValid() ? gps.satellites.value() : 0;
  int activeBars = 0;
  if      (satCount >= 8) activeBars = 5;
  else if (satCount >= 6) activeBars = 4;
  else if (satCount >= 4) activeBars = 3;
  else if (satCount >= 2) activeBars = 2;
  else if (satCount >= 1) activeBars = 1;

  int barH[] = {2, 4, 6, 8, 10};
  for (int i = 0; i < 5; i++) {
    int bx = 2 + i * 4;
    int by = 10 - barH[i];
    if (i < activeBars) u8g2.drawBox(bx, by, 3, barH[i]);
    else                u8g2.drawFrame(bx, by, 3, barH[i]);
  }

  // Satellite count next to bars
  u8g2.setCursor(23, 10);
  if (gps.satellites.isValid()) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", satCount);
    u8g2.print(buf);
  } else {
    u8g2.print("--");
  }

  // Compass direction — centred at x=62
  {
    const char* dir = "---";
    if (gps.course.isValid() && gps.speed.isValid() && gps.speed.kmph() > 1.0) {
      double c = gps.course.deg();
      if      (c >= 337.5 || c <  22.5) dir = "N";
      else if (c >=  22.5 && c <  67.5) dir = "NE";
      else if (c >=  67.5 && c < 112.5) dir = "E";
      else if (c >= 112.5 && c < 157.5) dir = "SE";
      else if (c >= 157.5 && c < 202.5) dir = "S";
      else if (c >= 202.5 && c < 247.5) dir = "SW";
      else if (c >= 247.5 && c < 292.5) dir = "W";
      else                               dir = "NW";
    }
    u8g2.setCursor(62 - (int)(strlen(dir) * 3), 10);
    u8g2.print(dir);
  }

  // Board v1: no battery ADC — right side of status bar left blank

  // Separator line
  u8g2.drawHLine(0, 11, 128);
}

void updateDisplay(String statusMessage) {
  if (!ENABLE_DISPLAY_OUTPUT) return;

  // Full-screen blinking alert for new Flock detection — no status bar, maximum impact
  if (millis() < flockAlertUntilMs) {
    bool inverted = (millis() / 350) % 2;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_10x20_tf);
    if (inverted) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }
    u8g2.drawStr(46, 28, "GET");
    u8g2.drawStr(4, 50, "FLOCKED!");
    u8g2.setDrawColor(1);
    u8g2.sendBuffer();
    return;
  }

  u8g2.clearBuffer();
  drawStatusBar();
  u8g2.setFont(u8g2_font_5x8_tf);

  // Content area below status bar (y=12-53), bottom bar (y=56-63)
  if (statusMessage.length() > 0 || !gpsTimeValid) {
    if (statusMessage.length() > 0) {
      u8g2.setCursor(2, 22);
      u8g2.print(statusMessage);
    }
  } else {
    int wifi_last  = cached_wifi_last;
    int ble_last   = cached_ble_last;
    int wifi_total = cached_wifi_total;
    int ble_total  = cached_ble_total;

    // WiFi count row
    u8g2.setCursor(2, 22);
    {
      char buf[24];
      if (!enableWifiScan) {
        u8g2.print("W:--");
      } else if (wifiScanning) {
        snprintf(buf, sizeof(buf), "W:%d(%d)*", wifi_last, wifi_total);
        u8g2.print(buf);
      } else {
        snprintf(buf, sizeof(buf), "W:%d(%d)", wifi_last, wifi_total);
        u8g2.print(buf);
      }
    }

    // BLE count row
    u8g2.setCursor(2, 32);
    {
      char buf[24];
      if (!enableBleScan) {
        u8g2.print("B:--");
      } else if (bleScanning) {
        snprintf(buf, sizeof(buf), "B:%d(%d)*", ble_last, ble_total);
        u8g2.print(buf);
      } else {
        snprintf(buf, sizeof(buf), "B:%d(%d)", ble_last, ble_total);
        u8g2.print(buf);
      }
    }

    // Flock row (inverted badge when devices seen)
    if (cached_flock_total > 0) {
      char buf[16];
      snprintf(buf, sizeof(buf), "FLOCK:%d", cached_flock_total);
      int bw = strlen(buf) * 6 + 2;
      u8g2.drawBox(2, 38, bw, 9);
      u8g2.setDrawColor(0);
      u8g2.setCursor(3, 38 + FONT_SM_ASCENT);
      u8g2.print(buf);
      u8g2.setDrawColor(1);
    } else if (flockOnlyMode) {
      u8g2.setCursor(2, 38 + FONT_SM_ASCENT);
      u8g2.print("[FLOCK ONLY]");
    }

    // Animated WiFi logo (right side, cx≈101 cy≈40)
    drawWiFiLogo(85, 14);
    wifiAnimationState = (wifiAnimationState + 1) % 4;
  }

  drawGPSTime();
  drawSpeed();
  u8g2.sendBuffer();
}

void drawGPSTime() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(2, SCREEN_HEIGHT - 2);  // Baseline near bottom edge
  if (gps.time.isValid()) {
    char timeStr[12];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    u8g2.print(timeStr);
  } else if (rtc.getYear() > 2020) {
    char timeStr[12];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    u8g2.print(timeStr);
  } else {
    u8g2.print("--:--:--");
  }
}

void drawSpeed() {
  u8g2.setFont(u8g2_font_5x8_tf);
  char speedStr[24];
  if (gps.speed.isValid()) {
    snprintf(speedStr, sizeof(speedStr), "%.0fMPH %.0fKPH",
             gps.speed.mph(), gps.speed.kmph());
  } else {
    snprintf(speedStr, sizeof(speedStr), "0M 0K");
  }
  int textWidth = strlen(speedStr) * 6;
  u8g2.setCursor(SCREEN_WIDTH - textWidth - 2, SCREEN_HEIGHT - 2);
  u8g2.print(speedStr);
}


void drawWiFiLogo(int x, int y) {
  int cx = x + 16;
  int cy = y + 26;
  u8g2.drawDisc(cx, cy, 3);
  int ringRadii[] = {8, 15, 22};
  for (int ring = 0; ring < wifiAnimationState && ring < 3; ring++) {
    int r = ringRadii[ring];
    for (int angle = -55; angle <= 55; angle += 2) {
      float rad = angle * PI / 180.0;
      int px = cx + (int)(r * sin(rad));
      int py = cy - (int)(r * cos(rad));
      u8g2.drawPixel(px, py);
      u8g2.drawPixel(px, py - 1);
      u8g2.drawPixel(px + 1, py);
    }
  }
}
