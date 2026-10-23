#include "gripper.h"

unsigned long lastPrintTime = 0;
GripperState gripperState = UNKNOWN_GRIPPER_STATE; // Initialize gripper state

bool closeGripper() {
        sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Starting to close gripper");
        dxl.begin(57600); // Default baudrate for X-series
        dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
        dxl.torqueOff(GRIPPER_DXL_ID);
        dxl.setOperatingMode(GRIPPER_DXL_ID, OP_CURRENT_BASED_POSITION);
        dxl.torqueOn(GRIPPER_DXL_ID);
        dxl.writeControlTableItem(PROFILE_VELOCITY, GRIPPER_DXL_ID, 50);
        dxl.writeControlTableItem(GOAL_CURRENT, GRIPPER_DXL_ID, MAX_GRIPPER_CURRENT + 100);
        dxl.setGoalPosition(GRIPPER_DXL_ID, GRIPPER_CLOSED_POSITION - 10);
        unsigned long startTime = millis();
        while (dxl.getPresentPosition(GRIPPER_DXL_ID) > GRIPPER_CLOSED_POSITION) {
            int current = abs(dxl.getPresentCurrent(GRIPPER_DXL_ID));
            if (millis() - lastPrintTime >= PRINT_INTERVAL) { // Uses the 100ms interval from gripper.h
                lastPrintTime = millis();
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Closing gripper...");
}
            if (current > MAX_GRIPPER_CURRENT) { // Threshold for detecting a grip
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Gripper stopped due to current threshold. Possible object detection.");
                dxl.setGoalPosition(GRIPPER_DXL_ID, dxl.getPresentPosition(GRIPPER_DXL_ID)); // Stop the gripper
                break; 
            }
            if (millis() - startTime > 5000) { // Timeout after 5 seconds
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Timeout while closing gripper.");
                dxl.setGoalPosition(GRIPPER_DXL_ID, dxl.getPresentPosition(GRIPPER_DXL_ID)); // Stop the gripper
                break;
            }
        }
        if (dxl.getPresentPosition(GRIPPER_DXL_ID) <= GRIPPER_CLOSED_POSITION + 20) {
            sendCompletion(Part::GRIPPER, "SUCCESS:TRUE;GRIPPED:FALSE;STATUS:" + String(GripperState::CLOSED) + ";MESSAGE:Gripper fully closed.");
            gripperState = CLOSED;
            return false; 
        } else if (dxl.getPresentPosition(GRIPPER_DXL_ID) < 1700) {
            sendCompletion(Part::GRIPPER, "SUCCESS:TRUE;GRIPPED:TRUE;STATUS:" + String(GripperState::OBJECT_DETECTED) + ";MESSAGE:Gripper closed with object detected.");
            gripperState = OBJECT_DETECTED;
            return false; 
        } else {
            sendCompletion(Part::GRIPPER, "SUCCESS:FALSE;GRIPPED:FALSE;STATUS:" + String(GripperState::ERROR) + ";MESSAGE:Gripper closing failed.");
            gripperState = ERROR;
            return true;
        }    
}
bool openGripper() {
        sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Starting to open gripper");
        dxl.begin(57600); // Default baudrate for X-series
        dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
        dxl.torqueOff(GRIPPER_DXL_ID);
        dxl.setOperatingMode(GRIPPER_DXL_ID, OP_CURRENT_BASED_POSITION);
        dxl.torqueOn(GRIPPER_DXL_ID);
        dxl.writeControlTableItem(PROFILE_VELOCITY, GRIPPER_DXL_ID, 50);
        dxl.writeControlTableItem(GOAL_CURRENT, GRIPPER_DXL_ID, MAX_GRIPPER_CURRENT + 20);
        dxl.setGoalPosition(GRIPPER_DXL_ID, GRIPPER_OPEN_POSITION + 20);
        unsigned long startTime = millis();
        while (dxl.getPresentPosition(GRIPPER_DXL_ID) < GRIPPER_OPEN_POSITION) {
            int current = abs(dxl.getPresentCurrent(GRIPPER_DXL_ID));
            if (millis() - lastPrintTime >= PRINT_INTERVAL) { // Uses the 100ms interval from gripper.h
                lastPrintTime = millis();
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Opening gripper...");
            }
            if (current > MAX_GRIPPER_CURRENT) { // Threshold for detecting a grip
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Gripper stopped due to current threshold.");
                dxl.setGoalPosition(GRIPPER_DXL_ID, dxl.getPresentPosition(GRIPPER_DXL_ID));
                break; 
            }
            if (millis() - startTime > 5000) { // Timeout after 5 seconds
                sendFeedback(Part::GRIPPER, "POSITION:" + String(dxl.getPresentPosition(GRIPPER_DXL_ID)) + ";CURRENT:" + String(dxl.getPresentCurrent(GRIPPER_DXL_ID)) + ";MESSAGE:Timeout while opening gripper.");
                dxl.setGoalPosition(GRIPPER_DXL_ID, dxl.getPresentPosition(GRIPPER_DXL_ID));
                break;
            }
        }
        if (dxl.getPresentPosition(GRIPPER_DXL_ID) >= GRIPPER_OPEN_POSITION - 20) {
            sendCompletion(Part::GRIPPER, "SUCCESS:TRUE;GRIPPED:FALSE;STATUS:" + String(GripperState::OPEN) + ";MESSAGE:Gripper fully opened.");
            gripperState = OPEN;
            return true;
        } else {
            sendCompletion(Part::GRIPPER, "SUCCESS:FALSE;GRIPPED:FALSE;STATUS:" + String(GripperState::ERROR) + ";MESSAGE:Gripper opening failed.");
            gripperState = ERROR;
            return false;
        }    
}

GripperState getGripperState() {
    return gripperState;
}
