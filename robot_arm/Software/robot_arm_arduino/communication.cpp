#include "communication.h"
String incomingByte = "";

CommandStruct read_usb(){
  CommandStruct command;
  command.command = Serial.readString();
  if (command.command.startsWith("C.MOVE_ARM")) {
    command.commandType = MOVE_ARM;
    command.value1 = command.command.substring(11).toInt();
    command.value2 = -1;
  } else if (command.command.startsWith("C.STACK_MOVE_ARM")) { //260
    command.commandType = STACK_MOVE_ARM;
    command.value1 = command.command.substring(17).toInt();
    command.value2 = -2;
  } else if (command.command.startsWith("C.STOP_ARM")) {
    command.commandType = STOP_ARM;
    command.value1 = -1;
    command.value2 = -1;
  } else if (command.command.startsWith("C.CLOSE_GRIPPER")) {
    command.commandType = CLOSE_GRIPPER;
    command.value1 = -1;
    command.value2 = -1;
  } else if (command.command.startsWith("C.OPEN_GRIPPER")) {
    command.commandType = OPEN_GRIPPER;
    command.value1 = -1;
    command.value2 = -1;
  } else if (command.command.startsWith("C.POS_MOVE_ARM")) {
    //read the two angles as float after the command, separated by a semicolon, e.g., "MOVE_ARM_POS:45.0;30.0"
    command.commandType = MOVE_ARM_POS;
    int separatorIndex = command.command.indexOf(';');
    if (separatorIndex != -1) {
      String theta1Str = command.command.substring(15, separatorIndex);
      String theta2Str = command.command.substring(separatorIndex + 1);
      command.value1 = theta1Str.toFloat();
      command.value2 = theta2Str.toFloat();
    } else {
      command.value1 = -1;
      command.value2 = -1;
    }
  } else if (command.command.startsWith("C.PRE_MOVE_ARM")) {
    //read label of preset position, "MOVE_ARM_PRE:UP_LONG"
    command.commandType = MOVE_ARM_PRE;
    String presetLabel = command.command.substring(15, 22);
    Serial.println(presetLabel);
    if (presetLabel == "UP_LONG") {
      command.value1 = 1;
    } else if (presetLabel == "UP_HIGH") {
      command.value1 = 2;
    } else if (presetLabel == "UP_SHRT") {
      command.value1 = 3;
    } else if (presetLabel == "UP_SCAN") {
      command.value1 = 4;
    } else if (presetLabel == "UP_SENS") {
      command.value1 = 5; // unknown preset
    } else {
      command.value1 = -1; // unknown preset
    }
    command.value2 = -1;

  } else if (command.command.startsWith("C.GET_STATUS")) {
    command.commandType = GET_STATUS;
    command.value1 = -1;
    command.value2 = -1;
  } else {
    command.commandType = UNKNOWN;
    command.value1 = -1;
    command.value2 = -1;
  }
  Serial.println("Received command, Type: " + String(command.commandType) + ", Value: " + String(command.value1) + "; " + String(command.value2));
  return command;
}
void sendFeedback(Part part, String feedback) {
  switch (part) {
    case ARM:
      Serial.println("A.ARM.FEEDBACK:" + feedback);
      break;
    case GRIPPER:
      Serial.println("A.GRIPPER.FEEDBACK:" + feedback);
      break;
    case SENSOR:
      Serial.println("A.SENSOR.FEEDBACK:" + feedback);
      break;
    case GENERAL:
      Serial.println("A.GENERAL.FEEDBACK:" + feedback);
      break;
    default:
      Serial.println("A.UNKNOWN.FEEDBACK:" + feedback);
      break;
  }
  //Serial.println(feedback);
}

void sendCompletion(Part part, String message) {
  switch (part) {
    case ARM:
      Serial.println("A.ARM.END:" + message);
      break;
    case GRIPPER:
      Serial.println("A.GRIPPER.END:" + message);
      break;
    case SENSOR:
      Serial.println("A.SENSOR.END:" + message);
      break;
    case GENERAL:
      Serial.println("A.GENERAL.END:" + message);
      break;
    default:
      Serial.println("A.UNKNOWN.END:" + message);
      break;
  }
  //Serial.println(message);
}

void send_distance(long distance) {
  Serial.println("A.SENSOR.DIST:" + String(distance));
}
void send_status() {
  Serial.println("A.STATUS:DISTANCE_1:" + String(sensorDistance_1()) + ";DISTANCE_2:" + String(sensorDistance_2()) + ";ANGLE1:" + String(getPos1()) + ";ANGLE2:" + String(getPos2()) + ";GRIPPERSTATE:" + String(getGripperState()));
}

void send_full_status() {
  Serial.println("A.STATUS:DISTANCE_1:" + String(sensorDistance_1()) + ";DISTANCE_2:" + String(sensorDistance_2()) + ";ANGLE1:" + String(getPos1()) + ";ANGLE2:" + String(getPos2()) + ";GRIPPERSTATE:" + String(getGripperState()) + ";GRIPPERPOS:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";GRIPPERCURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)));
}
void send_scan() {
  Serial.println("A.SCAN:DISTANCE_1:" + String(sensorDistance_1()) + ";DISTANCE_2:" + String(sensorDistance_2()));
}






