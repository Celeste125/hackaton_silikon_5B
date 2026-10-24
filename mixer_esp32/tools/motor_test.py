"""Bench test the mixer motors without ROS.

Keeps resending the speeds every 100 ms (the firmware stops the motors after
500 ms of silence) and prints the ESP's messages. Type two speeds, -1..1:
  0.3 0      motor 0 at 30% forward
  0 -0.5     motor 1 at 50% reverse
  0 0        stop (also Enter on an empty line, or Ctrl+C)

  ~/.platformio/penv/bin/python tools/motor_test.py /dev/cu.wchusbserial10
"""

import sys
import threading
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.2
ser.dtr = ser.rts = False  # don't reset the ESP / hold it in the bootloader
ser.open()

speeds = [0.0, 0.0]
running = True


def sender():
    last_print = 0
    while running:
        ser.write(f'M {speeds[0]:.3f} {speeds[1]:.3f}\n'.encode())
        line = ser.read_all().decode(errors='ignore').strip().splitlines()
        # Show errors/info always, load cell lines once a second.
        for l in line:
            if not l.startswith('L ') or time.time() - last_print > 1.0:
                print(f'   esp: {l}')
                last_print = time.time() if l.startswith('L ') else last_print
        time.sleep(0.1)


threading.Thread(target=sender, daemon=True).start()
print(f'Connected to {port}. Type "<speed0> <speed1>", Enter = stop, Ctrl+C = quit.')
try:
    while True:
        text = input('> ').split()
        try:
            values = [max(-1.0, min(1.0, float(v))) for v in text] if text else [0, 0]
            speeds[:] = (values + [0.0, 0.0])[:2]
            print(f'   motor 0 = {speeds[0]:+.2f}, motor 1 = {speeds[1]:+.2f}')
        except ValueError:
            print('   type two numbers, e.g. 0.3 0')
except (KeyboardInterrupt, EOFError):
    pass
finally:
    speeds[:] = [0.0, 0.0]
    time.sleep(0.25)
    running = False
    ser.write(b'M 0 0\n')
    ser.close()
    print('\nstopped')
