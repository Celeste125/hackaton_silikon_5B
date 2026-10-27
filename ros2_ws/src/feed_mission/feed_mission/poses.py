"""What the robot is taught once about the arena, stored next to the saved map.

~/maps/arena_poses.yaml by default:

    silo1: {x: 1.20, y: -0.40, yaw: 1.571, saved: '2026-10-26 21:10:03'}
    silo_entry: {...} # open space beside silo 1: Nav2 drives here, the rest is straight
    docked: {...}     # on the dock; the spot in front of it comes from this pose
    start: {...}      # only without a dock: where the mission parks at the end
    cow_box: {cx: 0.76, cy: -0.39, yaw: 1.38, length: 1.0, width: 0.5, saved: ...}
    feed: {gap: 0.1, door_open: 0.2, ...}     # last settings used for laps

All in the map frame: they only match while the robot is localized on the
same map, and remapping the arena means teaching them again. The web panel
writes them; lap_driver and the mission read them.

Each lap run also adds a line to feed_runs.yaml in the same folder (settings
and how much feed left the container), to tune the door opening against.
"""

import math
import os
import tempfile
import time

import yaml

DEFAULT_FILE = '~/maps/arena_poses.yaml'

SILOS = ('silo1', 'silo2', 'silo3')
NAMES = SILOS + ('silo_entry', 'start', 'docked')
BOX_KEY = 'cow_box'
FEED_KEY = 'feed'
MISSION_KEY = 'mission'
PREDOCK_DISTANCE = 0.6  # m in front of the dock: from there the Create 3's Dock takes over


def predock_pose(docked, distance=PREDOCK_DISTANCE):
    """The spot in front of the dock, facing it, worked out from the docked pose.

    The Create 3 docks front first, so it is straight back from the docked
    pose with the same heading; Dock then drives the last bit by itself.
    """
    return {'x': docked['x'] - distance * math.cos(docked['yaw']),
            'y': docked['y'] - distance * math.sin(docked['yaw']),
            'yaw': docked['yaw']}
MAX_RUNS = 200


def resolve(path=DEFAULT_FILE):
    return os.path.abspath(os.path.expanduser(path))


def _read(path):
    path = resolve(path)
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _write(path, data):
    path = resolve(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    # Write a temp file and rename it over the old one, so a crash mid-write
    # never leaves a half-written file behind.
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    os.chmod(tmp, 0o644)
    os.replace(tmp, path)


def _stamp():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def load(path=DEFAULT_FILE):
    """{name: {'x', 'y', 'yaw', 'saved'}} of the taught poses."""
    poses = {}
    for name, p in _read(path).items():
        if name in NAMES and isinstance(p, dict):
            poses[name] = {
                'x': float(p['x']), 'y': float(p['y']), 'yaw': float(p['yaw']),
                'saved': str(p.get('saved', '')),
            }
    return poses


def save_pose(name, x, y, yaw, path=DEFAULT_FILE):
    if name not in NAMES:
        raise ValueError(f'unknown pose "{name}", expected one of {", ".join(NAMES)}')
    data = _read(path)
    data[name] = {
        'x': round(float(x), 3), 'y': round(float(y), 3),
        'yaw': round(math.atan2(math.sin(yaw), math.cos(yaw)), 4),
        'saved': _stamp(),
    }
    _write(path, data)
    return data[name]


def clear_pose(name, path=DEFAULT_FILE):
    data = _read(path)
    if data.pop(name, None) is not None:
        _write(path, data)


def load_box(path=DEFAULT_FILE):
    """{'cx', 'cy', 'yaw', 'length', 'width', 'saved'} or None."""
    b = _read(path).get(BOX_KEY)
    if not isinstance(b, dict):
        return None
    out = {k: float(b[k]) for k in ('cx', 'cy', 'yaw', 'length', 'width')}
    out['saved'] = str(b.get('saved', ''))
    return out


def save_box(box, path=DEFAULT_FILE):
    """box: dict or anything with as_dict()."""
    d = box.as_dict() if hasattr(box, 'as_dict') else dict(box)
    data = _read(path)
    data[BOX_KEY] = {
        'cx': round(float(d['cx']), 3), 'cy': round(float(d['cy']), 3),
        'yaw': round(float(d['yaw']), 4),
        'length': round(float(d['length']), 3), 'width': round(float(d['width']), 3),
        'saved': _stamp(),
    }
    _write(path, data)
    return data[BOX_KEY]


def load_section(key, path=DEFAULT_FILE):
    """A dict of last-used settings ('feed', 'mission'); empty if none yet."""
    f = _read(path).get(key)
    return dict(f) if isinstance(f, dict) else {}


def save_section(key, settings, path=DEFAULT_FILE):
    data = _read(path)
    data[key] = dict(settings)
    _write(path, data)


def load_feed(path=DEFAULT_FILE):
    return load_section(FEED_KEY, path)


def save_feed(settings, path=DEFAULT_FILE):
    save_section(FEED_KEY, settings, path)


def runs_file(path=DEFAULT_FILE):
    return os.path.join(os.path.dirname(resolve(path)), 'feed_runs.yaml')


def load_runs(path=DEFAULT_FILE):
    f = runs_file(path)
    if not os.path.exists(f):
        return []
    with open(f) as fh:
        runs = yaml.safe_load(fh)
    return runs if isinstance(runs, list) else []


def add_run(run, path=DEFAULT_FILE):
    runs = (load_runs(path) + [dict(run, time=_stamp())])[-MAX_RUNS:]
    f = runs_file(path)
    os.makedirs(os.path.dirname(f), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(f), suffix='.tmp')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(runs, fh, sort_keys=False)
    os.chmod(tmp, 0o644)
    os.replace(tmp, f)
    return runs
