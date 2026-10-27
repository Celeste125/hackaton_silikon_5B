"""How the robot gets in and out of the silo row.

The silos stand in a row against a wall, so tight that Nav2 cannot plan into
it (its planner keeps the robot's centre 0.26 m from walls; at the silos it is
0.15-0.20 m). The mission therefore drives the last part itself, on straight
lines between poses the robot has physically stood at:

  Nav2 to the silo entry (open space beside silo 1)
  straight to silo 1, then turn on the spot to silo 1's heading
  straight along the row to silo 2, then 3 (filling at each)
  back the same way: reverse along the row to silo 1, straight out to the entry

The entry is taught in Setup ("Silo entry"); without it, it is entry_m beside
silo 1 on its open side.
"""

import math

REVERSE_ABOVE = math.radians(100)  # a target further round than this is driven to in reverse


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def default_entry(silo1, side='right', distance=0.6):
    """silo1 moved `distance` sideways (robot's right or left), facing silo 1."""
    s = -1.0 if side == 'right' else 1.0
    ang = silo1['yaw'] + s * math.pi / 2
    x = silo1['x'] + distance * math.cos(ang)
    y = silo1['y'] + distance * math.sin(ang)
    return {'x': x, 'y': y, 'yaw': wrap(ang + math.pi)}


def drive_direction(pose, target):
    """(heading to face while driving, reverse?) for a straight move to target.

    A target behind the robot is reached in reverse instead of turning round:
    in the silo row there is no room to turn between silos.
    """
    x, y, yaw = pose
    ang = math.atan2(target['y'] - y, target['x'] - x)
    if abs(wrap(ang - yaw)) > REVERSE_ABOVE:
        return wrap(ang + math.pi), True
    return ang, False


def last_silo(targets):
    """Index of the last silo with something to load, or None."""
    full = [i for i, t in enumerate(targets) if t > 0]
    return full[-1] if full else None
