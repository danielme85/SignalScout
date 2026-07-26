#pragma once
// SignalScout Board v1 — XIAO ESP32-C6
// Same PCB as v1-C5; GPIO numbers differ because XIAO C6 maps different chip
// GPIOs to the same physical pads.  Pad → GPIO cross-reference:
//
//   Physical pad | C5 GPIO | C6 GPIO
//   -------------|---------|--------
//   D0           |  GPIO1  |  GPIO0
//   D3           |  GPIO7  |  GPIO21
//   D6 (TX)      |  GPIO11 |  GPIO16
//   D7 (RX)      |  GPIO12 |  GPIO17
//   D8 (SCK)     |  GPIO8  |  GPIO19
//   D9 (MISO)    |  GPIO9  |  GPIO20
//   D10 (MOSI)   |  GPIO10 |  GPIO18
//   MTMS         |  GPIO2  |  GPIO4
//   MTDI         |  GPIO3  |  GPIO5
//   MTCK         |  GPIO4  |  GPIO6
//
// Chip: Seeed XIAO ESP32-C6 (8MB flash, no PSRAM)

// SD Card SPI (shares HW SPI bus with OLED)
#define SD_CS    6   // MTCK pad  (C5=GPIO4  → C6=GPIO6)
#define SD_MOSI  18  // D10 pad   (C5=GPIO10 → C6=GPIO18)
#define SD_MISO  20  // D9 pad    (C5=GPIO9  → C6=GPIO20)
#define SD_SCK   19  // D8 pad    (C5=GPIO8  → C6=GPIO19)

// GPS UART
// ⚠ HARDWARE REWORK REQUIRED: on the C5 XIAO, GPS_RX used GPIO6 which is the
// internal ADC_BAT pad — that pad has no GPIO equivalent at the same position on
// the C6 XIAO.  The GPS_RX trace on the PCB will need to be cut and re-routed to
// the D2 pad (GPIO2 on C6), which is otherwise unused on this board.
// Update GPS_RX below once the rework is done.
#define GPS_RX   2   // D2 pad (C6=GPIO2) — REWORK from original ADC_BAT trace
#define GPS_TX   21  // D3 pad   (C5=GPIO7  → C6=GPIO21)
#define GPS_BAUD 9600

// OLED Display SPI (RST tied to 3V3 on PCB — no GPIO needed)
#define OLED_DC    5              // MTDI pad (C5=GPIO3  → C6=GPIO5)
#define OLED_CS    16             // D6 pad   (C5=GPIO11 → C6=GPIO16)
#define OLED_RESET U8X8_PIN_NONE

// Rotary Encoder
#define ENC_A_PIN    0    // D0 pad   (C5=GPIO1 → C6=GPIO0)
#define ENC_B_PIN    4    // MTMS pad (C5=GPIO2 → C6=GPIO4)
#define ENC_SW_PIN   17   // D7 pad   (C5=GPIO12 → C6=GPIO17)
#define ENC_HOLD_TIME 1000

// Capabilities
#define HAS_NEOPIXEL       0  // no RGB LED on v1 PCB
#define HAS_BATTERY_ADC    0  // no voltage divider on v1 PCB
#define HAS_DUAL_BAND_WIFI 0  // ESP32-C6 is 2.4 GHz only; 5 GHz channels will not appear
#define HAS_FILE_SERVER    0  // ESPAsyncWebServer does not support ESP32-C6 yet
#define HAS_BLE            1  // BLE supported on C6
