/*
 * mixer_esp32: feed mixer I/O board
 * ==================================
 * Reads 4 HX711 load cells and drives 2 BTS7960 (IBT-2) motor drivers.
 * Talks to the ROS 2 driver on the Jetson/Pi over USB serial, one text line
 * per message. Calibration (tare, kg) is done on the ROS side, so this
 * firmware only sends raw counts and never needs reflashing for that.
 *
 * Board: classic ESP32 (ESP32-WROOM DevKit). Arduino core 2.x or 3.x.
 *
 * ---- Serial protocol (115200 baud, lines end with '\n') -------------------
 *
 *   Host -> ESP
 *     M <speed0> <speed1>   Motor speeds, -1.0 .. 1.0 (negative = reverse).
 *                           e.g. "M 0.5 -0.25"
 *                           Applied instantly, no ramp: smoothing is done by
 *                           the ROS driver. Must be resent at least every
 *                           CMD_TIMEOUT_MS, otherwise the motors stop (safety
 *                           if the ROS node or the USB cable dies).
 *     S <us>                Door servo pulse width in microseconds,
 *                           e.g. "S 1500". Clamped to SERVO_MIN_US..
 *                           SERVO_MAX_US. The servo only starts getting
 *                           pulses after the first S, and then holds its
 *                           position (no timeout, a door must stay put).
 *     ?                     Prints the info line again.
 *
 *   ESP -> Host
 *     I mixer_esp32 cells=4 motors=2 servo=1  Info, sent at boot and on "?".
 *     L <raw0> <raw1> <raw2> <raw3>      Raw HX711 counts of every cell,
 *                                        sent when all cells have a new
 *                                        sample (~10 Hz).
 *     E <message>                        Error, e.g. a cell not responding.
 */

#include <Arduino.h>

#include "config.h"  // pins and settings

const int NUM_CELLS = sizeof(CELLS) / sizeof(CELLS[0]);
const int NUM_MOTORS = sizeof(MOTORS) / sizeof(MOTORS[0]);
const uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

// ---- HX711 ------------------------------------------------------------------

portMUX_TYPE hxMux = portMUX_INITIALIZER_UNLOCKED;

int32_t lastRaw[NUM_CELLS];
bool fresh[NUM_CELLS];
uint32_t lastSampleMs[NUM_CELLS];
uint32_t lastErrorMs[NUM_CELLS];

// DT goes LOW when the HX711 has a new conversion ready.
bool hxReady(const CellPins &c) { return digitalRead(c.dt) == LOW; }

// Clock out one 24-bit sample. Only call when hxReady().
int32_t hxRead(const CellPins &c) {
  uint32_t value = 0;
  // Interrupts off while clocking: if SCK stays high > 60 us the HX711
  // powers down and the sample is lost. The whole read takes ~50 us.
  portENTER_CRITICAL(&hxMux);
  for (int i = 0; i < 24; i++) {
    digitalWrite(c.sck, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(c.dt);
    digitalWrite(c.sck, LOW);
    delayMicroseconds(1);
  }
  // 25th pulse: next conversion uses channel A with gain 128.
  digitalWrite(c.sck, HIGH);
  delayMicroseconds(1);
  digitalWrite(c.sck, LOW);
  portEXIT_CRITICAL(&hxMux);

  // 24-bit two's complement -> signed 32-bit.
  if (value & 0x800000) value |= 0xFF000000;
  return (int32_t)value;
}

void pollCells() {
  uint32_t now = millis();
  for (int i = 0; i < NUM_CELLS; i++) {
    if (hxReady(CELLS[i])) {
      lastRaw[i] = hxRead(CELLS[i]);
      fresh[i] = true;
      lastSampleMs[i] = now;
    } else if (now - lastSampleMs[i] > CELL_TIMEOUT_MS &&
               now - lastErrorMs[i] > 1000) {
      Serial.printf("E cell %d (DT GPIO%d) not responding, check wiring\n", i,
                    CELLS[i].dt);
      lastErrorMs[i] = now;
    }
  }

  // Send one line once every cell has a new sample, so the host always gets
  // a consistent set of all 4 cells.
  for (int i = 0; i < NUM_CELLS; i++) {
    if (!fresh[i]) return;
  }
  Serial.print('L');
  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.print(' ');
    Serial.print(lastRaw[i]);
    fresh[i] = false;
  }
  Serial.print('\n');
}

// ---- Motors -------------------------------------------------------------------

float targetSpeed[NUM_MOTORS];
float currentSpeed[NUM_MOTORS];
uint32_t lastCmdMs = 0;

// The LEDC (PWM) API changed between Arduino-ESP32 core 2.x and 3.x.
void pwmAttach(int pin, int channel, uint32_t freq = PWM_FREQ_HZ,
               uint8_t bits = PWM_BITS) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, bits);
#else
  ledcSetup(channel, freq, bits);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(int pin, int channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// Each motor uses two PWM channels: 2*i for RPWM, 2*i+1 for LPWM.
void applyMotor(int i, float speed) {
  uint32_t duty = (uint32_t)(fabsf(speed) * PWM_MAX + 0.5f);
  // Only one side is ever driven; the other is held at 0.
  if (speed >= 0) {
    pwmWrite(MOTORS[i].lpwm, 2 * i + 1, 0);
    pwmWrite(MOTORS[i].rpwm, 2 * i, duty);
  } else {
    pwmWrite(MOTORS[i].rpwm, 2 * i, 0);
    pwmWrite(MOTORS[i].lpwm, 2 * i + 1, duty);
  }
}

void updateMotors() {
  // Safety stop when the host goes quiet.
  if (CMD_TIMEOUT_MS > 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    for (int i = 0; i < NUM_MOTORS; i++) targetSpeed[i] = 0;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (targetSpeed[i] != currentSpeed[i]) {
      currentSpeed[i] = targetSpeed[i];
      applyMotor(i, currentSpeed[i]);
    }
  }
}

// ---- Servo --------------------------------------------------------------------

// Channel 4 is the first one after the motors (0..3). On core 2.x channels
// are paired per timer, so this one doesn't share a timer with the 20 kHz
// motor PWM.
const int SERVO_CHANNEL = 4;
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t SERVO_BITS = 16;
bool servoAttached = false;

void setServoUs(float us) {
  us = constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  if (!servoAttached) {
    pwmAttach(SERVO_PIN, SERVO_CHANNEL, SERVO_FREQ_HZ, SERVO_BITS);
    servoAttached = true;
  }
  // 50 Hz -> 20000 us period.
  uint32_t duty = (uint32_t)(us / 20000.0f * ((1u << SERVO_BITS) - 1) + 0.5f);
  pwmWrite(SERVO_PIN, SERVO_CHANNEL, duty);
}

// ---- Serial commands ----------------------------------------------------------

char lineBuf[64];
size_t lineLen = 0;

void printInfo() {
  Serial.printf("I mixer_esp32 cells=%d motors=%d servo=1\n", NUM_CELLS,
                NUM_MOTORS);
}

void handleLine(char *line) {
  if (line[0] == 'M') {
    // "M <speed0> <speed1> ...": parse one float per motor.
    char *p = line + 1;
    for (int i = 0; i < NUM_MOTORS; i++) {
      char *end;
      float v = strtof(p, &end);
      if (end == p) {
        Serial.println("E bad M command, expected: M <speed0> <speed1>");
        return;
      }
      targetSpeed[i] = constrain(v, -1.0f, 1.0f);
      p = end;
    }
    lastCmdMs = millis();
  } else if (line[0] == 'S') {
    char *end;
    float us = strtof(line + 1, &end);
    if (end == line + 1) {
      Serial.println("E bad S command, expected: S <microseconds>");
      return;
    }
    setServoUs(us);
  } else if (line[0] == '?') {
    printInfo();
  } else if (line[0] != '\0') {
    Serial.printf("E unknown command: %s\n", line);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;  // line too long: drop it
    }
  }
}

// ---- Main ---------------------------------------------------------------------

void setup() {
  // Motors first, so they are held off as early as possible after boot.
  for (int i = 0; i < NUM_MOTORS; i++) {
    pwmAttach(MOTORS[i].rpwm, 2 * i);
    pwmAttach(MOTORS[i].lpwm, 2 * i + 1);
    applyMotor(i, 0);
    if (MOTORS[i].en >= 0) {
      pinMode(MOTORS[i].en, OUTPUT);
      digitalWrite(MOTORS[i].en, HIGH);
    }
  }

  for (int i = 0; i < NUM_CELLS; i++) {
    pinMode(CELLS[i].dt, INPUT);
    pinMode(CELLS[i].sck, OUTPUT);
    digitalWrite(CELLS[i].sck, LOW);  // LOW also wakes the HX711
    lastSampleMs[i] = millis();
  }

  Serial.begin(SERIAL_BAUD);
  printInfo();
}

void loop() {
  pollSerial();
  pollCells();
  updateMotors();
}
