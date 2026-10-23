"""Wiring and settings for the load cells. Edit this file, then colcon build.

Each HX711 board (Joy-IT SEN-HX711-05):
  VCC -> Pi 3.3V   GND -> Pi GND   DT/SCK -> the GPIOs below
  E+ red, E- black, A- green, A+ white (B-/B+ unused)
"""

# One entry per load cell. BCM GPIO numbers (not physical pin numbers).
CELLS = [
    {'dt': 5, 'sck': 6},     # cell 0: DT -> GPIO5  (pin 29), SCK -> GPIO6  (pin 31)
    {'dt': 13, 'sck': 19},   # cell 1: DT -> GPIO13 (pin 33), SCK -> GPIO19 (pin 35)
    {'dt': 17, 'sck': 27},   # cell 2: DT -> GPIO17 (pin 11), SCK -> GPIO27 (pin 13)
    {'dt': 22, 'sck': 23},   # cell 3: DT -> GPIO22 (pin 15), SCK -> GPIO23 (pin 16)
]

# /dev/gpiochipN with the header pins. None = auto-detect (Pi 3/4/5).
GPIOCHIP = None

# Written by `ros2 run loadcell_wrapper calibrate`. /ws is the mounted
# workspace, so the file survives container rebuilds.
CALIBRATION_FILE = '/ws/loadcell_calibration.json'

# Samples averaged for tare/calibration (HX711 runs at ~10 Hz -> 30 = 3 s).
CALIBRATION_SAMPLES = 30

# Median over the last N samples of each cell; removes read glitches.
# Bigger = smoother but slower to react.
FILTER_WINDOW = 5
