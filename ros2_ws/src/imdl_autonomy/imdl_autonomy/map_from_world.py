#!/usr/bin/env python3
"""Generate Nav2 occupancy maps from the IMDL Gazebo arena worlds."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ARENA_SIZE_M = 1.63
DEFAULT_RESOLUTION = 0.02
DEFAULT_MARGIN = 0.20


def parse_pose(text):
    values = [float(item) for item in (text or '0 0 0 0 0 0').split()]
    while len(values) < 6:
        values.append(0.0)
    return values


def world_obstacles(world_path: Path, exclude_model_suffixes=()):
    root = ET.parse(world_path).getroot()
    obstacles = []
    for model in root.findall('.//model'):
        model_name = model.get('name', '')
        if any(model_name.endswith(suffix) for suffix in exclude_model_suffixes):
            continue
        model_pose = parse_pose(model.findtext('pose'))
        for collision in model.findall('.//collision'):
            pose = parse_pose(collision.findtext('pose'))
            x = model_pose[0] + pose[0]
            y = model_pose[1] + pose[1]
            box = collision.find('.//box/size')
            cylinder = collision.find('.//cylinder/radius')
            if box is not None and box.text:
                sx, sy, _ = [float(item) for item in box.text.split()[:3]]
                obstacles.append(('box', x, y, sx, sy, model_name))
            elif cylinder is not None and cylinder.text:
                radius = float(cylinder.text)
                obstacles.append(('circle', x, y, radius, radius, model_name))
    return obstacles


def occupied_by(obstacles, x, y):
    for kind, ox, oy, sx, sy, _name in obstacles:
        if kind == 'box':
            if abs(x - ox) <= sx * 0.5 and abs(y - oy) <= sy * 0.5:
                return True
        elif kind == 'circle':
            if math.hypot(x - ox, y - oy) <= sx:
                return True
    return False


def generate_map(
    world_path: Path,
    output_dir: Path,
    resolution: float,
    exclude_model_suffixes=(),
):
    map_size = ARENA_SIZE_M + 2.0 * DEFAULT_MARGIN
    width = int(math.ceil(map_size / resolution))
    height = width
    origin_x = -map_size * 0.5
    origin_y = -map_size * 0.5
    obstacles = world_obstacles(world_path, exclude_model_suffixes)

    pixels = []
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        pixel_row = []
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            pixel_row.append(0 if occupied_by(obstacles, x, y) else 254)
        pixels.append(pixel_row)

    stem = world_path.stem
    pgm_name = f'{stem}.pgm'
    yaml_name = f'{stem}.yaml'
    output_dir.mkdir(parents=True, exist_ok=True)

    pgm_path = output_dir / pgm_name
    with pgm_path.open('w', encoding='ascii') as pgm:
        pgm.write('P2\n')
        pgm.write(f'{width} {height}\n')
        pgm.write('255\n')
        for row in reversed(pixels):
            pgm.write(' '.join(str(value) for value in row))
            pgm.write('\n')

    yaml_path = output_dir / yaml_name
    with yaml_path.open('w', encoding='ascii') as yaml_file:
        yaml_file.write(f'image: {pgm_name}\n')
        yaml_file.write('mode: trinary\n')
        yaml_file.write(f'resolution: {resolution:.4f}\n')
        yaml_file.write(f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n')
        yaml_file.write('negate: 0\n')
        yaml_file.write('occupied_thresh: 0.65\n')
        yaml_file.write('free_thresh: 0.25\n')

    return yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('worlds', nargs='+', help='Gazebo world files to convert')
    parser.add_argument('--output-dir', required=True, help='Directory for maps')
    parser.add_argument(
        '--resolution',
        type=float,
        default=DEFAULT_RESOLUTION,
        help='Map resolution in meters per pixel',
    )
    parser.add_argument(
        '--exclude-dynamic-blocks',
        action='store_true',
        help='Exclude *_block models; required for unknown-object tasks.',
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    excluded = ('_block',) if args.exclude_dynamic_blocks else ()
    for world in args.worlds:
        yaml_path = generate_map(
            Path(world),
            output_dir,
            args.resolution,
            exclude_model_suffixes=excluded,
        )
        print(yaml_path)


if __name__ == '__main__':
    main()
