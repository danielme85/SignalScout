#pragma once
// ============================================================
//  BOARD SELECTION — edit this section for Arduino IDE users
// ============================================================
// Uncomment exactly ONE line to match your hardware.
// Leave all commented out to use the default (SignalScout PCB v1 + XIAO C5).
//
#define BOARD_SIGNALSCOUT_V1_C5   // Custom PCB v1 + XIAO ESP32-C5 (default)
// #define BOARD_SIGNALSCOUT_V1_C6   // Custom PCB v1 + XIAO ESP32-C6
//
// arduino-cli / CI users: leave the lines above commented out and pass
//   --build-property "build.extra_flags=-DBOARD_xxx"  instead.
// ============================================================

#if defined(BOARD_SIGNALSCOUT_V1_C5)
  #include "boards/signalscout_v1_c5.h"

#elif defined(BOARD_SIGNALSCOUT_V1_C6)
  #include "boards/signalscout_v1_c6.h"

#else
  // Default: original custom PCB with XIAO ESP32-C5
  #define BOARD_SIGNALSCOUT_V1_C5
  #include "boards/signalscout_v1_c5.h"
#endif
