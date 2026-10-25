#!/usr/bin/env python3
"""Join recorded runs into one training set for the location prior network.

Each run the mapper recorded holds a pile of input windows plus a `run.yaml`
giving where the landmarks turned out to be. This script pairs them: for every
sample it converts each landmark's map position into that sample's own
robot-centric window and stores the cell index as the label.

    ./build_landmark_prior_dataset.py --runs-dir ~/landmark_runs \
        --out landmark_prior/model/dataset.npz

Two things it does that matter more than they look:

**The train/validation split is by run, never by sample.** Samples within one
run are consecutive snapshots of the same map with the robot a few centimetres
further along -- neighbouring samples are nearly identical. Splitting those at
random would put near-copies of validation samples in the training set and
report a validation error several times better than the truth. Held out by
run, the number means something.

**The geometry comes from the robot's own module**, imported from the ROS
workspace rather than reimplemented here. The labels have to land in exactly
the cell the inference code will read back, and a half-cell disagreement
between the two would train a prior that is quietly offset for ever.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import sys

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'imdl_autonomy'))
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'perception'))

from imdl_autonomy.landmark_prior import (  # noqa: E402
    DATASET_VERSION,
    OCCUPIED_THRESHOLD,
    SAMPLE_CHANNELS,
    SAMPLE_ROBOT_POSE,
    WINDOW_CELLS,
    WINDOW_RESOLUTION_M,
    Window,
)
from perception.landmark_detection import load_landmark_specs  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--runs-dir', required=True, type=Path,
        help='Directory holding the run_* folders the mapper wrote.')
    parser.add_argument(
        '--out', required=True, type=Path,
        help='Where to write the combined dataset .npz.')
    parser.add_argument(
        '--landmark-config', type=Path,
        default=REPO / 'ros2_ws' / 'src' / 'perception' / 'config'
        / 'landmarks.yaml',
        help='Fixes the output channel order; must match the robot config.')
    parser.add_argument(
        '--val-runs', type=int, default=1,
        help='How many whole runs to hold out for validation.')
    parser.add_argument(
        '--seed', type=int, default=0,
        help='Which runs get held out.')
    parser.add_argument(
        '--max-sigma-m', type=float, default=0.25,
        help='Reject a run whose label is looser than this: a label the '
             'pipeline was unsure of teaches the network that uncertainty.')
    parser.add_argument(
        '--require-all-landmarks', action='store_true',
        help='Skip runs that did not converge on every landmark.')
    return parser.parse_args()


def load_run(run_dir: Path, classes, max_sigma_m: float, require_all: bool):
    """Read one run into (channels, targets, valid), or explain the skip."""
    manifest_path = run_dir / 'run.yaml'
    if not manifest_path.is_file():
        return None, 'no run.yaml (the mapper never converged, or crashed early)'
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8')) or {}

    version = int(manifest.get('dataset_version', 0))
    if version != DATASET_VERSION:
        return None, f'dataset_version {version}, expected {DATASET_VERSION}'
    window_spec = manifest.get('window') or {}
    if (int(window_spec.get('cells', 0)) != WINDOW_CELLS
            or abs(float(window_spec.get('resolution_m', 0.0))
                   - WINDOW_RESOLUTION_M) > 1e-9):
        return None, (
            f"window {window_spec} does not match this code's "
            f'{WINDOW_CELLS} at {WINDOW_RESOLUTION_M} m')
    # Absent in the earliest runs, where it was the same value anyway.
    recorded_threshold = int(
        window_spec.get('occupied_threshold', OCCUPIED_THRESHOLD))
    if recorded_threshold != OCCUPIED_THRESHOLD:
        return None, (
            f'recorded with occupied_threshold {recorded_threshold}, but this '
            f'code builds inputs at {OCCUPIED_THRESHOLD}; the channels would '
            'not match what a model trained here would see')

    labels = manifest.get('labels') or {}
    usable = {
        name: entry for name, entry in labels.items()
        if name in classes
        and float(entry.get('sigma_major_m', 1e9)) <= max_sigma_m
    }
    if not usable:
        return None, 'no label is tight enough to learn from'
    if require_all and len(usable) != len(classes):
        return None, f'only converged on {sorted(usable)}'

    sample_paths = sorted((run_dir / 'samples').glob('sample_*.npz'))
    if not sample_paths:
        return None, 'no samples'

    channels_out, targets_out, valid_out = [], [], []
    for path in sample_paths:
        with np.load(path) as sample:
            channels = sample[SAMPLE_CHANNELS]
            pose = sample[SAMPLE_ROBOT_POSE]
        window = Window(
            robot_x=float(pose[0]), robot_y=float(pose[1]),
            robot_yaw=float(pose[2]),
            cells=WINDOW_CELLS, resolution_m=WINDOW_RESOLUTION_M)

        target = np.zeros(len(classes), dtype=np.int64)
        valid = np.zeros(len(classes), dtype=bool)
        for index, name in enumerate(classes):
            entry = usable.get(name)
            if entry is None:
                continue
            i, j = window.map_to_index(float(entry['x']), float(entry['y']))
            i, j = int(round(i)), int(round(j))
            # A landmark outside the window cannot be pointed at, so the
            # sample is simply not a training example for that class.
            if not window.contains_index(i, j):
                continue
            target[index] = i * WINDOW_CELLS + j
            valid[index] = True

        if not valid.any():
            continue
        channels_out.append(channels.astype(np.uint8))
        targets_out.append(target)
        valid_out.append(valid)

    if not channels_out:
        return None, 'every sample had both landmarks outside its window'
    return (
        np.stack(channels_out), np.stack(targets_out), np.stack(valid_out),
    ), f'{len(channels_out)} samples, labels for {sorted(usable)}'


def main():
    args = parse_args()
    specs = load_landmark_specs(
        yaml.safe_load(args.landmark_config.read_text(encoding='utf-8')))
    # Sorted, because the network's output channels are positional and the
    # robot picks them with sorted(landmarks) too.
    classes = sorted(specs)
    print(f'classes (output channel order): {classes}')

    run_dirs = sorted(p for p in args.runs_dir.expanduser().iterdir()
                      if p.is_dir() and p.name.startswith('run_'))
    if not run_dirs:
        raise SystemExit(f'no run_* directories under {args.runs_dir}')

    loaded, skipped = {}, {}
    for run_dir in run_dirs:
        data, note = load_run(
            run_dir, classes, args.max_sigma_m, args.require_all_landmarks)
        if data is None:
            skipped[run_dir.name] = note
            print(f'  skip {run_dir.name}: {note}')
        else:
            loaded[run_dir.name] = data
            print(f'  take {run_dir.name}: {note}')

    if not loaded:
        raise SystemExit(
            'no usable runs. Every run needs to have converged on at least '
            'one landmark; check run.yaml holds a labels: block.')

    names = sorted(loaded)
    if len(names) < 2:
        print(
            '\nWARNING: one usable run. There is nothing to hold out, so the '
            'validation numbers below will be meaningless and the network '
            'will memorise this single run. Collect more before trusting it.')
        validation_names: list[str] = []
    else:
        held_out = min(args.val_runs, len(names) - 1)
        random.Random(args.seed).shuffle(names)
        validation_names = sorted(names[:held_out])
        names = sorted(names[held_out:])

    def stack(selected):
        if not selected:
            return (
                np.zeros((0, 2, WINDOW_CELLS, WINDOW_CELLS), dtype=np.uint8),
                np.zeros((0, len(classes)), dtype=np.int64),
                np.zeros((0, len(classes)), dtype=bool))
        parts = [loaded[name] for name in selected]
        return tuple(
            np.concatenate([part[index] for part in parts])
            for index in range(3))

    train = stack(sorted(names))
    validation = stack(validation_names)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        train_channels=train[0], train_target=train[1], train_valid=train[2],
        val_channels=validation[0], val_target=validation[1],
        val_valid=validation[2],
        classes=np.array(classes),
        meta=np.array(json.dumps({
            'dataset_version': DATASET_VERSION,
            'window_cells': WINDOW_CELLS,
            'window_resolution_m': WINDOW_RESOLUTION_M,
            'train_runs': sorted(names),
            'val_runs': validation_names,
            'skipped_runs': skipped,
        })))

    print(f'\ntrain: {len(train[0])} samples from {len(names)} runs')
    print(f'val:   {len(validation[0])} samples from '
          f'{len(validation_names)} runs {validation_names}')
    for index, name in enumerate(classes):
        print(f'  {name}: {int(train[2][:, index].sum())} train labels, '
              f'{int(validation[2][:, index].sum())} val labels')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
