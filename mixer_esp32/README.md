# mixer_esp32

ESP32 firmware for the feed mixer: reads the 4 HX711 load cells and drives the
2 mixer motors (BTS7960 / IBT-2) and a door servo. A ROS 2 driver talks to it over USB serial.

All pins and settings are in [`src/config.h`](src/config.h), with a comment
on every pin. `src/main.cpp` is the firmware itself.

## Flash

PlatformIO:

```bash
cd mixer_esp32
pio run -t upload
pio device monitor        # should print: I mixer_esp32 cells=4 motors=2 servo=1
```

Arduino IDE: create a sketch folder `mixer_esp32_ino/`, copy `src/main.cpp`
into it as `mixer_esp32_ino.ino` and `src/config.h` next to it, select board
"ESP32 Dev Module", upload.

## Wiring (matches src/config.h)

| Part | Pin | ESP32 |
|---|---|---|
| Load cell 0 | DT / SCK | GPIO13 / GPIO12 |
| Load cell 1 | DT / SCK | GPIO26 / GPIO25 |
| Load cell 2 | DT / SCK | GPIO14 / GPIO27 |
| Load cell 3 | DT / SCK | GPIO33 / GPIO32 |
| Door servo | signal | GPIO17 (power from an external 5V, common GND) |
| All HX711 | VCC / GND | 3.3V / GND |
| Motor 0 | RPWM / LPWM | GPIO23 / GPIO22 |
| Motor 1 | RPWM / LPWM | GPIO18 / GPIO19 |
| All BTS7960 | R_EN + L_EN | 3.3V (required, motors don't move otherwise) |
| All BTS7960 | VCC / GND | 3.3V / GND (common ground required) |

If you change pins in `config.h`, update this table too.

HX711 to load cell: E+ red, E- black, A- green, A+ white.

## Serial protocol (115200 baud, one line per message)

| Direction | Line | Meaning |
|---|---|---|
| host -> ESP | `M 0.5 -0.25` | motor speeds -1..1. Resend at least every 500 ms or the motors ramp to 0 |
| host -> ESP | `S 1500` | door servo pulse width in us (clamped to `SERVO_MIN_US..SERVO_MAX_US`); holds position, no timeout |
| host -> ESP | `?` | print the info line |
| ESP -> host | `I mixer_esp32 cells=4 motors=2 servo=1` | info, at boot and on `?` |
| ESP -> host | `L 8123 -1204 50211 3022` | raw counts of all cells, ~10 Hz |
| ESP -> host | `E ...` | error, e.g. a load cell not responding |

Tare and kg calibration happen in the ROS driver, so recalibrating never
needs a reflash.

Testing by hand in the serial monitor: the 500 ms timeout stops the motors
almost immediately, so set `CMD_TIMEOUT_MS = 0` for bench tests, then back.
