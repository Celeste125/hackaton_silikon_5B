"""Minimal HX711 driver on lgpio (works on Pi 4 and Pi 5)."""

import glob
import time
from statistics import median

import lgpio

from loadcell_wrapper import config


class HX711:
    def __init__(self, handle, dt, sck):
        self.handle = handle
        self.dt = dt
        self.sck = sck
        lgpio.gpio_claim_input(handle, dt)
        lgpio.gpio_claim_output(handle, sck, 0)

    def is_ready(self):
        # DT goes low when a new conversion is available.
        return lgpio.gpio_read(self.handle, self.dt) == 0

    def read_raw(self):
        """Read one 24-bit sample. Only call when is_ready()."""
        h, dt, sck = self.handle, self.dt, self.sck
        value = 0
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
            value = (value << 1) | lgpio.gpio_read(h, dt)
        # 25th pulse selects channel A, gain 128 for the next conversion.
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        if value & 0x800000:
            value -= 1 << 24
        return value


def open_header_chip(chip=None):
    """Open the gpiochip that carries the 40-pin header."""
    if chip is not None:
        return lgpio.gpiochip_open(chip)
    for path in sorted(glob.glob('/dev/gpiochip*')):
        handle = lgpio.gpiochip_open(int(path[len('/dev/gpiochip'):]))
        label = lgpio.gpio_get_chip_info(handle)[3]
        if any(name in label for name in ('rp1', 'bcm2711', 'bcm2835')):
            return handle
        lgpio.gpiochip_close(handle)
    raise RuntimeError(
        'No Raspberry Pi GPIO chip found. In Docker, pass --device /dev/gpiochipN.'
    )


def open_cells():
    handle = open_header_chip(config.GPIOCHIP)
    cells = [HX711(handle, c['dt'], c['sck']) for c in config.CELLS]
    return handle, cells


def read_medians(cells, n):
    """Collect n samples from every cell and return the median of each."""
    samples = [[] for _ in cells]
    deadline = time.monotonic() + n * 0.2 + 1.0
    while any(len(s) < n for s in samples):
        if time.monotonic() > deadline:
            missing = [
                f'cell {i} (DT GPIO{c.dt})'
                for i, (c, s) in enumerate(zip(cells, samples)) if len(s) < n
            ]
            raise TimeoutError(f'No data from {", ".join(missing)}, check wiring')
        for cell, s in zip(cells, samples):
            if len(s) < n and cell.is_ready():
                s.append(cell.read_raw())
        time.sleep(0.001)
    return [median(s) for s in samples]
