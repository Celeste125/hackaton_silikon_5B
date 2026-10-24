#pragma once
/*
 * mixer_esp32 configuration: pins and settings.
 * Edit this file, then reflash. Everything else is in main.cpp.
 */

#include <stdint.h>

// ============================================================================
// CONFIGURATION: change pins and settings here
// ============================================================================
// Pin numbers are ESP32 GPIO numbers (the "GPIOxx"/"Dxx" printed on the
// DevKit), not physical pin positions.
//
// Pins to AVOID on a classic ESP32:
//   GPIO 6-11      connected to the flash chip, using them crashes the ESP
//   GPIO 0,2,5,12,15  strapping pins, can break booting/flashing
//   GPIO 34-39     input only: fine for HX711 DT, NOT for SCK or motors
//   GPIO 1,3       USB serial (TX/RX), used to talk to ROS

// ---- Load cells (HX711, Joy-IT SEN-HX711-05) ------------------------------
// Each HX711 board: VCC -> ESP 3.3V, GND -> ESP GND,
//                   DT and SCK -> pins below.
// Load cell wires on the HX711 board: E+ red, E- black, A- green, A+ white.
struct CellPins {
  int dt;   // HX711 DT (data out)  -> ESP input  (34-39 are fine here)
  int sck;  // HX711 SCK (clock in) -> ESP output
};

const CellPins CELLS[] = {
    {13, 12},  // cell 0: DT -> GPIO13, SCK -> GPIO12 (12 is a strapping pin:
               //         only OK as SCK, never put DT on it)
    {26, 25},  // cell 1: DT -> GPIO26, SCK -> GPIO25
    {14, 27},  // cell 2: DT -> GPIO14, SCK -> GPIO27
    {33, 32},  // cell 3: DT -> GPIO33, SCK -> GPIO32
};  

// A cell that sends nothing for this long is reported with an "E" line.
const uint32_t CELL_TIMEOUT_MS = 1000;

// ---- Motors (BTS7960 / IBT-2) ----------------------------------------------
// Each BTS7960 board: VCC -> ESP 5V/VIN (try 3.3V if it reacts unreliably),
//                     GND -> ESP GND (REQUIRED, common ground),
//                     R_IS / L_IS -> leave unconnected,
//                     B+ / B- -> motor battery, M+ / M- -> motor.
struct MotorPins {
  int rpwm;  // BTS7960 RPWM: PWM here = forward
  int lpwm;  // BTS7960 LPWM: PWM here = reverse
  int en;    // BTS7960 R_EN + L_EN joined together.
             // -1 = not used, tie R_EN and L_EN to 3.3V instead.
             // Or a GPIO: driver is only enabled while this firmware runs.
};

const MotorPins MOTORS[] = {
    {23, 22, -1},  // motor 0: RPWM -> GPIO23, LPWM -> GPIO22, EN -> 3.3V
    {18, 19, -1},  // motor 1: RPWM -> GPIO18, LPWM -> GPIO19, EN -> 3.3V
};

// 20 kHz is above hearing, so the motors don't whine. BTS7960 max is 25 kHz.
const uint32_t PWM_FREQ_HZ = 20000;
const uint8_t PWM_BITS = 10;  // duty resolution: 0..1023

// Motors ramp to the new speed instead of jumping (protects gearbox and
// power supply). Units: full speed per second, so 2.0 = 0 -> 100% in 0.5 s.
// 0 = no ramp, jump instantly.
const float RAMP_PER_S = 2.0f;

// Motors stop if no "M" command arrives for this long. 0 = never (only for
// testing by hand in a serial monitor!).
const uint32_t CMD_TIMEOUT_MS = 500;

const uint32_t SERIAL_BAUD = 115200;
// ============================================================================
// END OF CONFIGURATION
// ============================================================================
