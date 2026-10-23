#include "arm.h"


void setupDynamixel() {

    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // Setup first motor (DXL_ID = 1)
    dxl.torqueOff(ARM_1_DXL_ID);
    dxl.setOperatingMode(ARM_1_DXL_ID, OP_POSITION);
    // 2. Tune PID Gains (Exactly the same as before)
    //dxl.writeControlTableItem(POSITION_D_GAIN, ARM_1_DXL_ID, 16000);
    //dxl.writeControlTableItem(POSITION_P_GAIN, ARM_1_DXL_ID, 200);

    // 3. Set Motion Profiles (Exactly the same as before)
    //dxl.writeControlTableItem(PROFILE_ACCELERATION, ARM_1_DXL_ID, 20);
    dxl.torqueOn(ARM_1_DXL_ID);
    dxl.writeControlTableItem(PROFILE_VELOCITY, ARM_1_DXL_ID, 20);
     


    // Setup second motor (DXL_ID = 4)
    dxl.torqueOff(ARM_2_DXL_ID);
    //dxl.writeControlTableItem(POSITION_D_GAIN, ARM_2_DXL_ID, 16000);
    //dxl.writeControlTableItem(POSITION_P_GAIN, ARM_2_DXL_ID, 20);
    dxl.setOperatingMode(ARM_2_DXL_ID, OP_POSITION);
    dxl.torqueOn(ARM_2_DXL_ID);
    dxl.writeControlTableItem(PROFILE_VELOCITY, ARM_2_DXL_ID, 20);

    // Setup Gripper motor (DXL_ID = 3)
    dxl.torqueOff(GRIPPER_DXL_ID);
    dxl.setOperatingMode(GRIPPER_DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.torqueOn(GRIPPER_DXL_ID);
    dxl.writeControlTableItem(PROFILE_VELOCITY, GRIPPER_DXL_ID, 50);

    
}

JointAngles calculateIK(double x, bool stack) {
    JointAngles angles;
    const double L1 = 134.5; // Length of first arm segment
    const double L2 = 190.0; // Length of second arm segment + length to gripper center
    const double OFFSET_X = 0.0; // X offset for the second arm segment (Gripper)
    const double OFFSET_Y = -7.2; // Y offset for the second arm segment (Gripper)
    const bool ELBOW_UP = (true); 
    double y = 110.0; // 150 mm - 40 for safety and to account for gripper center used
    
    x = x - 90; //Robot base to Arm Start

    JointAngles result = {0.0, 0.0, false};
    if (stack) {
        y -= 30; // Adjust y for stacking
    }

    // Calculate the effective length and angular offset of the second link
    double effectiveL2 = std::sqrt(std::pow(L2 + OFFSET_X, 2) + std::pow(OFFSET_Y, 2));
    double gamma = std::atan2(OFFSET_Y, L2 + OFFSET_X);

    // Calculate distance base to target
    double distanceSq = std::pow(x, 2) + std::pow(y, 2);

    // Law of Cosines to find the elbow angle
    double cosElbow = (distanceSq - std::pow(L1, 2) - std::pow(effectiveL2, 2)) / (2 * L1 * effectiveL2);

    // Check if the target is physically reachable
    if (cosElbow < -1.0 || cosElbow > 1.0) {
        // Target is out of reach
        return result; 
    }

    // Determine elbow angle based on desired configuration (up or down)
    double thetaElbow = std::acos(cosElbow);
    if (!ELBOW_UP) {
        thetaElbow = -thetaElbow;
    }

    // Calculate actual motor angles
    result.theta2 = thetaElbow - gamma;
    
    double k1 = L1 + effectiveL2 * std::cos(thetaElbow);
    double k2 = effectiveL2 * std::sin(thetaElbow);
    
    result.theta1 = std::atan2(y, x) - std::atan2(k2, k1);
    result.theta1 = abs(result.theta1 * (180.0 / M_PI)); // Convert to degrees
    result.theta2 = (result.theta2 * (180.0 / M_PI)) + 180.0; // Convert to degrees

    if (result.theta1 < 5 || result.theta1 > 90 || result.theta2 < 170 || result.theta2 > 305) {
        result.isReachable = false;
    } else {
        result.isReachable = true;
    }
    return result;
}



bool moveDist(double x, bool stack) {
    JointAngles angles = calculateIK(x, stack);
    if (angles.isReachable) {
        armFeedback("Angles calculated successfully. Moving to Theta1 = " + String(angles.theta1) + ", Theta2 = " + String(angles.theta2));
        return movePosDeg(angles.theta1, angles.theta2);
    }
    sendCompletion(Part::ARM, "SUCCESS:FALSE;REACHABLE:FALSE;MESSAGE:Target position unreachable. Theta1 = " + String(angles.theta1) + ", Theta2 = " + String(angles.theta2));
    return false;

}
bool movePreset(int preset) {
    double theta1, theta2;
    switch (preset) {
        case 1: // UP_LONG
            theta1 = 38.0;
            theta2 = 250.0;
            break;
        case 2: // UP_HIGH
            theta1 = 90.0;
            theta2 = 210.0;
            break;
        case 3: // UP_SHRT
            theta1 = 38.0;
            theta2 = 295.0;
            break;
        case 4: // UP_SCAN
            theta1 = 10.0;
            theta2 = 95.0;
            break;
        case 5: // UP_SENS
            theta1 = 83.0;
            theta2 = 91.0;
            break;
        default:
            sendCompletion(Part::ARM, "SUCCESS:FALSE;REACHABLE:FALSE;MESSAGE:Unknown preset position.");
            return false; // Unknown preset
    }
    armFeedback("Moving to preset position: Theta1 = " + String(theta1) + ", Theta2 = " + String(theta2));
    return movePosDeg(theta1, theta2);
}

bool movePosDeg(double theta1, double theta2) {
    armFeedback("Starting move to Theta1 = " + String(theta1) + ", Theta2 = " + String(theta2));
    unsigned long startTime = millis();
    if (theta2 - dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE) < 0) {
        dxl.setGoalPosition(ARM_1_DXL_ID, SAFE_HEIGHT_1, UNIT_DEGREE); // Convert radians to Dynamixel units
        startTime = millis();
        while (abs(dxl.getPresentPosition(ARM_1_DXL_ID, UNIT_DEGREE) - SAFE_HEIGHT_1) > 5) {
            armFeedback("Moving Servo 1 up to SAFE_HEIGHT_1 = " + String(SAFE_HEIGHT_1));
            if (millis() - startTime > 10000) { // Timeout after 10 seconds
                armFeedback("Timeout reached while moving Servo 1.");
                break; // Exit the loop if it takes too long
            }
        }
        armFeedback("Servo 1 reached SAFE_HEIGHT_1.");
        dxl.setGoalPosition(ARM_2_DXL_ID, theta2, UNIT_DEGREE); // Convert radians to Dynamixel units
        startTime = millis();
        while (abs(dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE) - theta2) > 5) {
            armFeedback("Moving Servo 2 to " + String(theta2) + ", Theta1 = " + String(theta1));
            if (millis() - startTime > 10000) { // Timeout after 10 seconds
                armFeedback("Timeout reached while moving Servo 2.");
                break; // Exit the loop if it takes too long
            }
        }
        armFeedback("Servo 2 reached target position.");
    }

    dxl.setGoalPosition(ARM_1_DXL_ID, theta1, UNIT_DEGREE); // Convert radians to Dynamixel units
    startTime = millis();
    while (abs(dxl.getPresentPosition(ARM_1_DXL_ID, UNIT_DEGREE) - theta1) > 5) {
        armFeedback("Moving Servo 1 to " + String(theta1) + ", Theta2 = " + String(theta2));
        if (millis() - startTime > 10000) { // Timeout after 10 seconds
            armFeedback("Timeout reached while moving Servo 1.");
            break; // Exit the loop if it takes too long
        }
    }
    armFeedback("Servo 1 reached target position.");
    dxl.setGoalPosition(ARM_2_DXL_ID, theta2, UNIT_DEGREE); // Convert radians to Dynamixel units
    startTime = millis();
    while (abs(dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE) - theta2) > 5) {
        armFeedback("Moving Servo 2 to " + String(theta2) + ", Theta1 = " + String(theta1));
        if (millis() - startTime > 10000) { // Timeout after 10 seconds
            armFeedback("Timeout reached while moving Servo 2.");
            break; // Exit the loop if it takes too long
        }
    }
    armFeedback("Servo 2 reached target position.");
    if (abs(dxl.getPresentPosition(ARM_1_DXL_ID, UNIT_DEGREE) - theta1) <= 5 && abs(dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE) - theta2) <= 5) {
        sendCompletion(Part::ARM, "SUCCESS:TRUE;REACHABLE:TRUE;MESSAGE:Target position reached. Theta1 = " + String(theta1) + ", Theta2 = " + String(theta2));
        return true;
    } else {
        sendCompletion(Part::ARM, "SUCCESS:FALSE;REACHABLE:TRUE;MESSAGE:Failed to reach target position. Theta1 = " + String(theta1) + ", Theta2 = " + String(theta2));
        return false;
    }
    return true; // Placeholder for actual movement success
}

void armFeedback(String message) {
    sendFeedback(Part::ARM, "POSITION_1:" + String(dxl.getPresentPosition(ARM_1_DXL_ID, UNIT_DEGREE)) + ";POSITION_2:" + String(dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE)) + ";MESSAGE:" + message);
}

float getPos1() {
    return dxl.getPresentPosition(ARM_1_DXL_ID, UNIT_DEGREE);
}

float getPos2() {
    return dxl.getPresentPosition(ARM_2_DXL_ID, UNIT_DEGREE);
}