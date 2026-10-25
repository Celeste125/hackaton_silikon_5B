#!/usr/bin/env python3
"""Look at a built prior dataset before spending GPU time on it.

    ./inspect_landmark_prior_dataset.py --dataset landmark_prior/model/dataset.npz
    ./inspect_landmark_prior_dataset.py --dataset ... --render /tmp/samples

Prints the statistics worth knowing, and with --render writes a few samples as
PNGs so the labels can be eyeballed against the walls. Both are there because
the most likely failure in this pipeline is not a training problem: it is a
bad camera extrinsic or a wrong frame putting every label in the wrong cell,
and no amount of training diagnoses that. Looking at ten pictures does.

In a rendered sample the robot is at the centre crosshair facing **down** the
image (the window's i axis is the robot's forward direction), occupied cells
are dark, unknown is grey, and each landmark's label is a coloured cross.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--render', type=Path, default=None)
    parser.add_argument('--render-count', type=int, default=8)
    parser.add_argument('--split', default='train', choices=['train', 'val'])
    return parser.parse_args()


def summarise(dataset, prefix: str, classes):
    channels = dataset[f'{prefix}_channels']
    valid = dataset[f'{prefix}_valid']
    target = dataset[f'{prefix}_target']
    print(f'\n{prefix}: {len(channels)} samples')
    if not len(channels):
        return
    print(f'  occupied cells per sample: '
          f'mean {channels[:, 0].sum(axis=(1, 2)).mean():.0f}, '
          f'min {channels[:, 0].sum(axis=(1, 2)).min():.0f}')
    print(f'  unknown fraction: {channels[:, 1].mean():.2f}')
    cells = channels.shape[-1]
    for index, name in enumerate(classes):
        mask = valid[:, index]
        print(f'  {name}: {int(mask.sum())} labels', end='')
        if mask.any():
            rows = target[mask, index] // cells
            columns = target[mask, index] % cells
            # Spread tells you whether the label is always in the same place
            # relative to the robot, which would mean the robot always stood
            # in the same spot -- little for the network to learn from.
            print(f', label spread {rows.std():.1f} x {columns.std():.1f} cells')
        else:
            print()


def render(dataset, prefix, classes, out_dir: Path, count: int):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        raise SystemExit('rendering needs pillow: pip install pillow')

    channels = dataset[f'{prefix}_channels']
    valid = dataset[f'{prefix}_valid']
    target = dataset[f'{prefix}_target']
    if not len(channels):
        raise SystemExit(f'the {prefix} split is empty')

    out_dir.mkdir(parents=True, exist_ok=True)
    cells = channels.shape[-1]
    palette = [(220, 40, 40), (40, 120, 230), (230, 160, 30), (120, 40, 200)]
    chosen = np.linspace(
        0, len(channels) - 1, min(count, len(channels))).astype(int)

    for order, index in enumerate(chosen):
        occupied = channels[index, 0] > 0
        unknown = channels[index, 1] > 0
        picture = np.full((cells, cells, 3), 245, dtype=np.uint8)
        picture[unknown] = (170, 170, 170)
        picture[occupied] = (30, 30, 30)
        image = Image.fromarray(picture).resize(
            (cells * 4, cells * 4), Image.NEAREST)
        draw = ImageDraw.Draw(image)

        middle = (cells - 1) / 2.0 * 4 + 2
        draw.line([(middle - 10, middle), (middle + 10, middle)],
                  fill=(0, 160, 0), width=2)
        draw.line([(middle, middle), (middle, middle + 24)],
                  fill=(0, 160, 0), width=2)  # forward

        for class_index, name in enumerate(classes):
            if not valid[index, class_index]:
                continue
            row = int(target[index, class_index]) // cells
            column = int(target[index, class_index]) % cells
            x, y = column * 4 + 2, row * 4 + 2
            colour = palette[class_index % len(palette)]
            draw.line([(x - 8, y - 8), (x + 8, y + 8)], fill=colour, width=3)
            draw.line([(x - 8, y + 8), (x + 8, y - 8)], fill=colour, width=3)
            draw.text((x + 10, y - 6), name, fill=colour)

        path = out_dir / f'{prefix}_{order:02d}_sample{index:05d}.png'
        image.save(path)
        print(f'  wrote {path}')


def main():
    args = parse_args()
    dataset = np.load(args.dataset, allow_pickle=False)
    meta = json.loads(str(dataset['meta']))
    classes = [str(name) for name in dataset['classes']]

    print(f'dataset {args.dataset}')
    print(f'  window: {meta["window_cells"]} cells at '
          f'{meta["window_resolution_m"]} m '
          f'({meta["window_cells"] * meta["window_resolution_m"]:.1f} m across)')
    print(f'  classes (channel order): {classes}')
    print(f'  train runs: {meta["train_runs"]}')
    print(f'  val runs:   {meta["val_runs"]}')
    if meta.get('skipped_runs'):
        print('  skipped:')
        for name, reason in meta['skipped_runs'].items():
            print(f'    {name}: {reason}')

    summarise(dataset, 'train', classes)
    summarise(dataset, 'val', classes)

    if not meta['val_runs']:
        print('\nWARNING: no held-out run, so validation numbers from '
              'training will be meaningless. Record more runs.')

    if args.render is not None:
        print(f'\nrendering {args.split} samples:')
        render(dataset, args.split, classes, args.render, args.render_count)


if __name__ == '__main__':
    main()
