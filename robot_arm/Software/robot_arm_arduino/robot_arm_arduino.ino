#include "gripper.h"
#include "sensor.h"

DynamixelShield dxl;

enum State {
  IDLE = 0,
  CALCULATING_IK = 1,
  MOVING_ARM = 2,
  STOPPED = 3,
  GRIPPER_CLOSING = 4,
  GRIPPER_OPENING = 5
};

CommandStruct command;
JointAngles angles;
const unsigned long MEASUREMENT_INTERVAL = 100;
bool scanMode = false;

void setup() {
  Serial.begin(115200);
  setupSensor();
  setupDynamixel();
}
unsigned long lastMeasurementTime1 = 0;
unsigned long lastMeasurementTime2 = 0;

void loop() {
  if (millis() - lastMeasurementTime1 >= 200) { // Uses the 100ms interval from gripper.h
    lastMeasurementTime1 = millis();
    if (true){
      send_status();
    }
    
  }
  if (Serial.available() > 0) {
        command = read_usb();
        switch (command.commandType) {
          case MOVE_ARM:
            scanMode = false;
            moveDist(command.value1);
            break;
          case STACK_MOVE_ARM:
            scanMode = false;
            moveDist(command.value1, true);
          case STOP_ARM:
            // Handle stop arm command
            break;
          case CLOSE_GRIPPER:
            // Handle close gripper command
            closeGripper();
            break;
          case OPEN_GRIPPER:
            // Handle open gripper command
            openGripper();
            break;
          case MOVE_ARM_POS:
            scanMode = false;
            movePosDeg(command.value1, command.value2);
            break;
          case MOVE_ARM_PRE:
            // Handle move arm to preset position command
            movePreset(command.value1);
            if (command.value1 == 4) {
              scanMode = true;
            } else {
              scanMode = false;
            }
            
            break;
          case GET_STATUS:
            // Handle get status command
            send_full_status();
            break;
            
          default:
            // Handle unknown command
            Serial.println("Unknown command received.");
            break;
        }
      }
  if (scanMode) {
    if (millis() - lastMeasurementTime2 >= 100) {
      lastMeasurementTime2 = millis();
      send_scan();
    }
  }
}
