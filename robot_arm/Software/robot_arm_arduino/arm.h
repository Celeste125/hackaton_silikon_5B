#ifndef ARM_H
#define ARM_H

#include "communication.h"
#include <cmath>
#include <iostream>
#include <DynamixelShield.h>

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
const int GRIPPER_DXL_ID = 3;
const int ARM_1_DXL_ID = 1;
const int ARM_2_DXL_ID = 4;
const int SAFE_HEIGHT_1 = 40;

extern DynamixelShield dxl;

struct JointAngles {
    double theta1;
    double theta2;
    bool isReachable;
};

JointAngles calculateIK(double x, bool stack = false);
void setupDynamixel();
bool movePosDeg(double theta1, double theta2);
bool movePreset(int preset);
float getPos1();
float getPos2();
void armFeedback(String message);
bool moveDist(double x, bool stack = false);

#endif

