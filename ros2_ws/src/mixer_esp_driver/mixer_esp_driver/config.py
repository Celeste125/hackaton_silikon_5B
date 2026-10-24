"""Settings for the mixer ESP32 driver. Edit here (no rebuild needed with
colcon build --symlink-install, just restart the node).

Pins are configured in the ESP firmware: mixer_esp32/src/config.h
"""

# USB serial port of the ESP32. Classic DevKit (CP2102/CH340): /dev/ttyUSB0.
# ESP32-S3/C3 with native USB: /dev/ttyACM0. A /dev/serial/by-id/... path
# never changes when other USB devices are plugged in. In Docker, pass it
# with --device.
SERIAL_PORT = '/dev/ttyUSB0'
BAUD = 115200  # must match SERIAL_BAUD in the firmware

# Must match the firmware (CELLS / MOTORS in config.h).
NUM_CELLS = 4
NUM_MOTORS = 2

# Written by `ros2 run mixer_esp_driver calibrate`. Relative to the directory
# you run from: start the driver AND calibrate from the ros2_ws folder, so
# both use ros2_ws/loadcell_calibration.json. An absolute path also works.
CALIBRATION_FILE = 'loadcell_calibration.json'

# Samples averaged for tare/calibration (HX711 runs at ~10 Hz -> 30 = 3 s).
CALIBRATION_SAMPLES = 30

# Median over the last N samples of each cell; removes read glitches.
# Bigger = smoother but slower to react.
FILTER_WINDOW = 5

# Motor speeds are resent to the ESP at this rate. The firmware stops the
# motors if nothing arrives for 500 ms, so keep this well above 2 Hz.
MOTOR_SEND_HZ = 10

# Safety: a motor stops if its topic gets no message for this long (e.g. the
# commanding node crashed). Keep publishing at a few Hz. 0 = never stop.
COMMAND_TIMEOUT_S = 1.0
