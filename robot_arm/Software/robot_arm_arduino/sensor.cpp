#include "sensor.h"

const int ECHO_PIN_1 = 4; // Echo Eingangs-Pin unterer Sensor
const int TRIGGER_PIN_1 = 5; // Trigger Ausgangs-Pin unterer Sensor
const int ECHO_PIN_2 = 6; // Echo Eingangs-Pin Arm Sensor
const int TRIGGER_PIN_2 = 7; // Trigger Ausgangs-Pin Arm Sensor
const int MAXIMUM_RANGE = 300;
const int MINIMUM_RANGE = 2;

void setupSensor() {
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
}

long sensorDistance_1() {
  long Abstand;
  long dauer;
  // Abstandsmessung wird mittels des 10us langen Triggersignals gestartet
  digitalWrite(TRIGGER_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_1, LOW);
  // Nun wird am Echo-Eingang gewartet, bis das Signal aktiviert wurde
  // und danach die Zeit gemessen, wie lang es aktiviert bleibt
  dauer = pulseIn(ECHO_PIN_1, HIGH, 30000);
  // Nun wird der Abstand mittels der aufgenommenen Zeit berechnet
  Abstand = dauer / 58.2;
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (Abstand >= MAXIMUM_RANGE || Abstand <= MINIMUM_RANGE) {
    return -1; // Indicate out of range
  } else {
    return (Abstand + 19) * 10;
  }
}
long sensorDistance_2() {
  long Abstand;
  long Dauer;
  // Abstandsmessung wird mittels des 10us langen Triggersignals gestartet
  digitalWrite(TRIGGER_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_2, LOW);
  // Nun wird am Echo-Eingang gewartet, bis das Signal aktiviert wurde
  // und danach die Zeit gemessen, wie lang es aktiviert bleibt
  Dauer = pulseIn(ECHO_PIN_2, HIGH, 30000);
  // Nun wird der Abstand mittels der aufgenommenen Zeit berechnet
  Abstand = Dauer / 58.2;
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (Abstand >= MAXIMUM_RANGE || Abstand <= MINIMUM_RANGE) {
    return -1; // Indicate out of range
  } else {
    return Abstand * 10;
  }
}