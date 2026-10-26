"""Settings for the mixer ESP32 driver. Edit here (no rebuild needed with
colcon build --symlink-install, just restart the node).

Pins are configured in the ESP firmware: mixer_esp32/src/config.h
"""

# USB serial port of the ESP32. Classic DevKit (CP2102/CH340): /dev/ttyUSB0.
# ESP32-S3/C3 with native USB: /dev/ttyACM0. A /dev/serial/by-id/... path
# never changes when other USB devices are plugged in. In Docker, pass it
# with --device.
# On the Jetson the ESP's CH340 shows up as /dev/ttyCH341USB0 (vendor CH341
# driver); /dev/ttyUSB0 is the lidar.
SERIAL_PORT = '/dev/ttyCH341USB0'
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

# The ramp and the shake pattern are computed at this rate; a changed speed is
# sent right away. 200 Hz = 5 ms timing for the shake reversals.
MOTOR_LOOP_HZ = 200

# Motors ramp to a new speed instead of jumping (protects gearbox and power
# supply). Units: full speed per second, so 2.0 = 0 -> 100% in 0.5 s.
# 0 = no ramp. The shake pattern (/mixer/shake) never ramps. The ESP itself
# applies every speed instantly.
RAMP_PER_S = 2.0

# Safety: a motor stops if its topic gets no message for this long (e.g. the
# commanding node crashed). Keep publishing at a few Hz. 0 = never stop.
COMMAND_TIMEOUT_S = 1.0

# ---- Door servo ----------------------------------------------------------------
# Door position topic: 0.0 = fully open, 1.0 = fully closed.
# The pulse widths for those two ends are stored in this file (same folder
# rules as CALIBRATION_FILE) and set with the /door/save_open and
# /door/save_closed services, see esp_driver_node.py.
SERVO_CALIBRATION_FILE = 'door_calibration.json'

# Used until the file exists. Must stay inside the firmware's
# SERVO_MIN_US..SERVO_MAX_US (config.h). Jog carefully from the middle first.
SERVO_DEFAULT_OPEN_US = 1000
SERVO_DEFAULT_CLOSED_US = 2000
