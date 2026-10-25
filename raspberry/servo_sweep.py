#!/usr/bin/env python3
"""
Servo sweep test for one PCA9685 channel - find the range where a servo moves.

Slowly raises the pulse width from a very low to a very high value and prints
every step, both in microseconds and in the "angle" units the valve
controller uses (0 = 900 us, 180 = 2100 us). Watch the servo:
  - the value where it STARTS moving is the low end of its range
  - the value where it STOPS moving is the high end
Press Ctrl+C at any time; outputs are switched off on exit.

Do NOT run this while the valve controller is running (both use the PCA9685).

Usage:
  python3 servo_sweep.py <channel> [--start US] [--end US] [--step US] [--delay S] [--down]

Examples:
  python3 servo_sweep.py 15                     # 300 -> 2800 us in 20 us steps
  python3 servo_sweep.py 15 --step 10 --delay 0.5
  python3 servo_sweep.py 15 --down              # sweep up, then back down
"""

import argparse
import time

from smbus2 import SMBus

ADDR = 0x40
MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L, ALL_LED_ON_L = 0x06, 0xFA
OSC_HZ, FREQ_HZ = 25_000_000, 50.0


def us_to_angle(us: float) -> float:
    """Same scale as the valve controller: 900 us = 0, 2100 us = 180."""
    return (us - 900.0) * 180.0 / 1200.0


class PCA9685:
    def __init__(self, bus: int = 1):
        self.bus = SMBus(bus)
        self.all_off()
        self.bus.write_byte_data(ADDR, MODE2, 0x04)
        self.bus.write_byte_data(ADDR, MODE1, 0x00)
        time.sleep(0.005)
        prescale = round(OSC_HZ / (4096 * FREQ_HZ)) - 1
        old = self.bus.read_byte_data(ADDR, MODE1) & 0x7F
        self.bus.write_byte_data(ADDR, MODE1, old | 0x10)
        self.bus.write_byte_data(ADDR, PRESCALE, prescale)
        self.bus.write_byte_data(ADDR, MODE1, old)
        time.sleep(0.005)
        self.bus.write_byte_data(ADDR, MODE1, old | 0xA0)
        self.period_us = 1_000_000 / (OSC_HZ / (4096 * (prescale + 1)))

    def set_pulse_us(self, channel: int, us: float) -> None:
        ticks = max(0, min(4095, int(round(us * 4096 / self.period_us))))
        self.bus.write_i2c_block_data(ADDR, LED0_ON_L + 4 * channel,
                                      [0, 0, ticks & 0xFF, ticks >> 8])

    def all_off(self) -> None:
        for offset, value in enumerate((0, 0, 0, 0x10)):
            self.bus.write_byte_data(ADDR, ALL_LED_ON_L + offset, value)

    def sleep(self) -> None:
        self.all_off()
        mode1 = self.bus.read_byte_data(ADDR, MODE1)
        self.bus.write_byte_data(ADDR, MODE1, (mode1 & 0x7F) | 0x10)
        self.bus.close()


def sweep(pca, channel, start, end, step, delay):
    direction = 1 if end >= start else -1
    us = start
    while (us <= end) if direction > 0 else (us >= end):
        pca.set_pulse_us(channel, us)
        print(f'  {us:6.0f} us   = angle {us_to_angle(us):7.1f}', flush=True)
        time.sleep(delay)
        us += direction * step


def main():
    p = argparse.ArgumentParser(description='Sweep one PCA9685 servo channel.')
    p.add_argument('channel', type=int, help='PCA9685 channel 0-15 (valve 3 = 15)')
    p.add_argument('--start', type=float, default=300, help='start pulse in us (default 300)')
    p.add_argument('--end', type=float, default=2800, help='end pulse in us (default 2800)')
    p.add_argument('--step', type=float, default=20, help='step in us (default 20)')
    p.add_argument('--delay', type=float, default=0.3, help='seconds per step (default 0.3)')
    p.add_argument('--down', action='store_true', help='sweep back down afterwards')
    a = p.parse_args()

    if not 0 <= a.channel <= 15:
        p.error('channel must be 0-15')

    pca = PCA9685()
    print(f'Channel {a.channel}: {a.start:.0f} -> {a.end:.0f} us, step {a.step:g} us, '
          f'{a.delay:g} s per step. Watch the servo. Ctrl+C to stop.\n')
    try:
        sweep(pca, a.channel, a.start, a.end, a.step, a.delay)
        if a.down:
            print('\n  ...and back down\n')
            sweep(pca, a.channel, a.end, a.start, a.step, a.delay)
        print('\nDone.')
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        pca.sleep()
        print('Outputs off.')


if __name__ == '__main__':
    main()
