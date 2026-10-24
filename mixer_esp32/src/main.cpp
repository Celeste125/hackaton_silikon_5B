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
 *                           Must be resent at least every CMD_TIMEOUT_MS,
 *                           otherwise the motors ramp down to 0 (safety if
 *                           the ROS node or the USB cable dies).
 *     ?                     Prints the info line again.
 *
 *   ESP -> Host
 *     I mixer_esp32 cells=4 motors=2     Info, sent at boot and on "?".
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
uint32_t lastRampUs = 0;

// The LEDC (PWM) API changed between Arduino-ESP32 core 2.x and 3.x.
void pwmAttach(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(channel, PWM_FREQ_HZ, PWM_BITS);
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
  uint32_t nowUs = micros();
  float dt = (nowUs - lastRampUs) * 1e-6f;
  lastRampUs = nowUs;

  // Safety stop when the host goes quiet.
  if (CMD_TIMEOUT_MS > 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    for (int i = 0; i < NUM_MOTORS; i++) targetSpeed[i] = 0;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    float next = targetSpeed[i];
    if (RAMP_PER_S > 0) {
      float maxStep = RAMP_PER_S * dt;
      float diff = targetSpeed[i] - currentSpeed[i];
      if (diff > maxStep) next = currentSpeed[i] + maxStep;
      if (diff < -maxStep) next = currentSpeed[i] - maxStep;
    }
    if (next != currentSpeed[i]) {
      currentSpeed[i] = next;
      applyMotor(i, next);
    }
  }
}

// ---- Serial commands ----------------------------------------------------------

char lineBuf[64];
size_t lineLen = 0;

void printInfo() {
  Serial.printf("I mixer_esp32 cells=%d motors=%d\n", NUM_CELLS, NUM_MOTORS);
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
  lastRampUs = micros();
  printInfo();
}

void loop() {
  pollSerial();
  pollCells();
  updateMotors();
}
