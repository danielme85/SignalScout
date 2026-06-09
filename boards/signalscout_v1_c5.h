#pragma once
// SignalScout Board v1 — XIAO ESP32-C5
// Custom PCB: rotary encoder, shared SPI bus (SD + OLED), no NeoPixel, no battery ADC.
// Chip: Seeed XIAO ESP32-C5 (8MB flash, 8MB PSRAM)

// SD Card SPI (shares HW SPI bus with OLED; CS lines are separate)
#define SD_CS    4   // MTCK pad  → GPIO4
#define SD_MOSI  10  // D10 pad   → GPIO10
#define SD_MISO  9   // D9 pad    → GPIO9
#define SD_SCK   8   // D8 pad    → GPIO8

// GPS UART
#define GPS_RX   6   // ADC_BAT pad → GPIO6  (see note in board_select.h)
#define GPS_TX   7   // D3 pad    → GPIO7
#define GPS_BAUD 9600

// OLED Display SPI (RST tied to 3V3 on PCB — no GPIO needed)
#define OLED_DC    3              // MTDI pad → GPIO3
#define OLED_CS    11             // D6 pad   → GPIO11
#define OLED_RESET U8X8_PIN_NONE

// Rotary Encoder (external 10k pull-ups + 10nF debounce caps on PCB)
#define ENC_A_PIN    1    // D0 pad  → GPIO1
#define ENC_B_PIN    2    // MTMS pad → GPIO2
#define ENC_SW_PIN   12   // D7 pad  → GPIO12
#define ENC_HOLD_TIME 1000

// Capabilities
#define HAS_NEOPIXEL       0  // no RGB LED on v1 PCB
#define HAS_BATTERY_ADC    0  // no voltage divider on v1 PCB
#define HAS_DUAL_BAND_WIFI 1  // ESP32-C5 supports 2.4 GHz + 5 GHz
