"""One-time calibration: ros2 run loadcell_wrapper calibrate

The 4 cells share one container, so the total weight is the SUM of the cells
(each carries part of the load, wherever it sits). One known weight gives one
scale factor for that sum. Cells wired backwards are detected and flipped.
"""

import json
import os

import lgpio

from loadcell_wrapper import config
from loadcell_wrapper.hx711 import open_cells, read_medians


def load_calibration(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        cal = json.load(f)
    if len(cal['offsets']) != len(config.CELLS):
        raise ValueError(
            f'{path} has {len(cal["offsets"])} cells but config.py has '
            f'{len(config.CELLS)}. Rerun calibrate.'
        )
    return cal


def save_calibration(path, cal):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(cal, f, indent=2)


def cell_weights(raws, cal):
    """Raw HX711 counts -> kg carried by each cell."""
    return [
        sign * (raw - offset) / cal['counts_per_kg']
        for raw, offset, sign in zip(raws, cal['offsets'], cal['signs'])
    ]


def ask_float(prompt, default):
    while True:
        text = input(prompt).strip()
        if not text:
            return default
        try:
            return float(text)
        except ValueError:
            print('   not a number, try again')


def main():
    print(f'Calibrating {len(config.CELLS)} load cells.')
    try:
        handle, cells = open_cells()
    except lgpio.error as e:
        raise SystemExit(f'{e}. Is loadcell_node still running? Stop it first.')

    try:
        input('1) Mount the EMPTY container (nothing inside). Press Enter...')
        print('   measuring zero...')
        offsets = read_medians(cells, config.CALIBRATION_SAMPLES)

        known_kg = ask_float('2) Known weight in kg [1.0]: ', 1.0)
        input(f'3) Put {known_kg} kg in the CENTER of the container. Press Enter...')
        print('   measuring...')
        loaded = read_medians(cells, config.CALIBRATION_SAMPLES)

        deltas = [l - o for l, o in zip(loaded, offsets)]
        total = sum(abs(d) for d in deltas)
        if total < 1000:
            raise SystemExit('   Readings barely changed. Check wiring and mounting.')
        for i, d in enumerate(deltas):
            share = abs(d) / total
            note = ''
            if share < 0.05:
                note = '  <- barely loaded, check wiring/mounting'
            elif d < 0:
                note = '  (reads backwards, flipped in software)'
            print(f'   cell {i}: {share:5.1%} of the load{note}')

        cal = {
            'offsets': offsets,
            'signs': [1 if d >= 0 else -1 for d in deltas],
            'counts_per_kg': total / known_kg,
        }
        save_calibration(config.CALIBRATION_FILE, cal)
        print(f'   saved to {config.CALIBRATION_FILE}')

        print(f'4) Check: move the weight to the corners, total should stay '
              f'~{known_kg} kg. Ctrl+C to quit.')
        while True:
            kg = cell_weights(read_medians(cells, 5), cal)
            cells_text = ' '.join(f'{k:6.3f}' for k in kg)
            print(f'\r   total {sum(kg):7.3f} kg   cells {cells_text}  ',
                  end='', flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        lgpio.gpiochip_close(handle)


if __name__ == '__main__':
    main()
