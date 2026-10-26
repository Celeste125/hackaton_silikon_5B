"""Named map poses taught once by driving there: the silos, start, lap corners.

Stored as YAML next to the saved map (~/maps/arena_poses.yaml by default), so
they belong to that map: remapping the arena means teaching them again.

    silo1: {x: 1.20, y: -0.40, yaw: 1.571, saved: '2026-10-26 21:10:03'}

The web panel's Setup tab writes them; lap_driver and the mission read them.
Poses are in the map frame, so they only match while the robot is localized
on the same map (navigation.launch.py with AMCL, or the live SLAM map).
"""

import math
import os
import tempfile
import time

import yaml

DEFAULT_FILE = '~/maps/arena_poses.yaml'

SILOS = ('silo1', 'silo2', 'silo3')
LAP_CORNERS = ('lap1', 'lap2', 'lap3', 'lap4')
NAMES = SILOS + ('start',) + LAP_CORNERS


def resolve(path=DEFAULT_FILE):
    return os.path.abspath(os.path.expanduser(path))


def load(path=DEFAULT_FILE):
    """{name: {'x', 'y', 'yaw', 'saved'}}; empty if the file does not exist."""
    path = resolve(path)
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    poses = {}
    for name, p in data.items():
        if name in NAMES and isinstance(p, dict):
            poses[name] = {
                'x': float(p['x']), 'y': float(p['y']), 'yaw': float(p['yaw']),
                'saved': str(p.get('saved', '')),
            }
    return poses


def _write(path, poses):
    path = resolve(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    # Write a temp file and rename it over the old one, so a crash mid-write
    # never leaves a half-written file behind.
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump({n: poses[n] for n in NAMES if n in poses}, f, sort_keys=False)
    os.replace(tmp, path)


def save_pose(name, x, y, yaw, path=DEFAULT_FILE):
    if name not in NAMES:
        raise ValueError(f'unknown pose "{name}", expected one of {", ".join(NAMES)}')
    poses = load(path)
    poses[name] = {
        'x': round(float(x), 3), 'y': round(float(y), 3),
        'yaw': round(math.atan2(math.sin(yaw), math.cos(yaw)), 4),
        'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    _write(path, poses)
    return poses[name]


def clear_pose(name, path=DEFAULT_FILE):
    poses = load(path)
    if poses.pop(name, None) is not None:
        _write(path, poses)


def lap_corners(poses):
    """The four corners in driving order, or None if any is missing."""
    if not all(c in poses for c in LAP_CORNERS):
        return None
    return [(poses[c]['x'], poses[c]['y']) for c in LAP_CORNERS]
