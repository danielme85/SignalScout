#pragma once
// Board selector for SignalScout.
//
// Pass exactly one -D flag at compile time to choose your hardware:
//
//   -DBOARD_SIGNALSCOUT_V1_C5   Custom PCB v1 with XIAO ESP32-C5  (default)
//   -DBOARD_SIGNALSCOUT_V1_C6   Custom PCB v1 with XIAO ESP32-C6
//
// Arduino IDE: add the flag under Sketch > Optimize > Custom build options,
// or edit the #define below to match your board.
//
// arduino-cli / CI: pass --build-property "build.extra_flags=-DBOARD_xxx"
//
// If no flag is defined the C5 PCB config is used (original hardware).

#if defined(BOARD_SIGNALSCOUT_V1_C5)
  #include "boards/signalscout_v1_c5.h"

#elif defined(BOARD_SIGNALSCOUT_V1_C6)
  #include "boards/signalscout_v1_c6.h"

#else
  // Default: original custom PCB with XIAO ESP32-C5
  #define BOARD_SIGNALSCOUT_V1_C5
  #include "boards/signalscout_v1_c5.h"
#endif
