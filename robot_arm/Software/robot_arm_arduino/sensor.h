#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"

extern const int ECHO_PIN_1; // Echo Eingangs-Pin
extern const int TRIGGER_PIN_1; // Trigger Ausgangs-Pin
extern const int MAXIMUM_RANGE; // Maximum range for distance measurement
extern const int MINIMUM_RANGE; // Minimum range for distance measurement


void setupSensor();
long sensorDistance_1();
long sensorDistance_2();



#endif // SENSOR_H
