#ifndef GRIPPER_H
#define GRIPPER_H

#include "Arduino.h"
#include "arm.h"
#include <cmath>
#include <iostream>

/*
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  // For Uno/Mega, use hardware serial for the motor, and software serial for PC input
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
#else
  // For boards with native USB (Leonardo, MKR, etc.)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
#endif

using namespace ControlTableItem;
const uint8_t DXL_ID = 1;               // Default Dynamixel ID
const float DXL_PROTOCOL_VERSION = 2.0;

extern DynamixelShield dxl;
*/
// --- Timer Variables ---
extern unsigned long lastPrintTime;
const unsigned long PRINT_INTERVAL = 100; // How often to print in milliseconds

const int GRIPPER_OPEN_POSITION = 2800;   // Position for open gripper
const int GRIPPER_CLOSED_POSITION = 1100; // Position for closed gripper
const int MAX_GRIPPER_CURRENT = 200; // Maximum current threshold for detecting a grip
const int GRIP_DETECTION_THRESHOLD = 300; // Maximum current threshold for detecting a grip


enum GripperState {
    OPEN,
    OBJECT_DETECTED,
    CLOSED,
    UNKNOWN_GRIPPER_STATE,
    ERROR
};

extern GripperState gripperState;

bool openGripper();
bool closeGripper();
GripperState getGripperState();

#endif