"""Load cell calibration math and file handling.

The 4 cells share one container, so the total weight is the SUM of the cells
(each carries part of the load, wherever it sits). One known weight gives one
scale factor for that sum. Cells wired backwards are detected and flipped.
"""

import json
import os

from mixer_esp_driver import config


def calibration_path():
    """Absolute path of the calibration file, for log messages."""
    return os.path.abspath(config.CALIBRATION_FILE)


def load_calibration(path=config.CALIBRATION_FILE):
    """Return the calibration dict, or None if the file doesn't exist."""
    if not os.path.exists(path):
        return None
    with open(path) as f:
        cal = json.load(f)
    if len(cal['offsets']) != config.NUM_CELLS:
        raise ValueError(
            f'{path} has {len(cal["offsets"])} cells but NUM_CELLS is '
            f'{config.NUM_CELLS}. Rerun calibrate.'
        )
    return cal


def save_calibration(cal, path=config.CALIBRATION_FILE):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(cal, f, indent=2)


def compute_calibration(offsets, loaded, known_kg):
    """Raw medians with the empty container and with known_kg inside -> cal.

    Also returns each cell's share of the load (0..1) for the printout.
    """
    deltas = [l - o for l, o in zip(loaded, offsets)]
    total = sum(abs(d) for d in deltas)
    if total < 1000:
        raise ValueError('Readings barely changed. Check wiring and mounting.')
    cal = {
        'offsets': list(offsets),
        'signs': [1 if d >= 0 else -1 for d in deltas],
        'counts_per_kg': total / known_kg,
    }
    shares = [abs(d) / total for d in deltas]
    return cal, shares


def cell_weights(raws, cal):
    """Raw HX711 counts -> kg carried by each cell."""
    return [
        sign * (raw - offset) / cal['counts_per_kg']
        for raw, offset, sign in zip(raws, cal['offsets'], cal['signs'])
    ]


def load_door_calibration(path=config.SERVO_CALIBRATION_FILE):
    """Return {'open_us', 'closed_us'}; the config defaults if no file yet."""
    cal = {
        'open_us': config.SERVO_DEFAULT_OPEN_US,
        'closed_us': config.SERVO_DEFAULT_CLOSED_US,
    }
    if os.path.exists(path):
        with open(path) as f:
            saved = json.load(f)
        cal['open_us'] = float(saved['open_us'])
        cal['closed_us'] = float(saved['closed_us'])
    return cal


def save_door_calibration(cal, path=config.SERVO_CALIBRATION_FILE):
    save_calibration(cal, path)


def door_position_to_us(position, cal):
    """0.0 (open) .. 1.0 (closed) -> servo pulse width in microseconds."""
    position = max(0.0, min(1.0, position))
    return cal['open_us'] + position * (cal['closed_us'] - cal['open_us'])
