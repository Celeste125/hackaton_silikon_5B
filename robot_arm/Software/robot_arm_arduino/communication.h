#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Arduino.h"
#include "sensor.h"
#include "arm.h"
#include "gripper.h"

enum CommandType {
  MOVE_ARM,
  STACK_MOVE_ARM,
  STOP_ARM,
  CLOSE_GRIPPER,
  OPEN_GRIPPER,
  MOVE_ARM_POS,
  MOVE_ARM_PRE,
  GET_STATUS,
  UNKNOWN,
  CMD_NONE
};

enum Part {
  ARM,
  GRIPPER,
  SENSOR,
  GENERAL,
  PART_NONE
};

struct CommandStruct{
  String command;
  CommandType commandType;
  int value1;
  int value2;
};

CommandStruct read_usb();
void sendFeedback(Part part, String status);
void sendCompletion(Part part, String status);
void send_distance(long distance);
void send_status();
void send_full_status();
void send_scan();
#endif