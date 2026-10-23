from types import SimpleNamespace

from rclpy.time import Time

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.visualization import (
    COLLISION_SLOW_ZONE,
    COLLISION_STOP_ZONE,
    MissionVisualization,
    ROBOT_FOOTPRINT,
)


class _Publisher:
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


class _Clock:
    @staticmethod
    def now():
        return Time(seconds=1.0)


class _Node:
    def __init__(self):
        self.publisher = _Publisher()

    def create_publisher(self, *_args):
        return self.publisher

    @staticmethod
    def get_clock():
        return _Clock()


def test_active_goal_marker_can_be_published():
    node = _Node()
    mission = SimpleNamespace(
        mission_id='task1_a',
        frame_id='map',
        poses={
            'A': Pose2D(-0.5, 0.0, 0.0),
            'B': Pose2D(0.5, 0.0, 0.0),
        },
    )
    visualizer = MissionVisualization(node, mission)
    visualizer.set_active_goal(mission.poses['B'], 'FastRPP')

    visualizer.publish(0.1)

    namespaces = {marker.ns for marker in node.publisher.message.markers}
    assert 'mission_active_goal' in namespaces
    assert 'mission_status' in namespaces


def test_visualized_safety_geometry_matches_nav2_configuration():
    import yaml
    from pathlib import Path

    config_path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    controller = config['controller_server']['ros__parameters']
    collision = config['collision_monitor']['ros__parameters']

    assert controller['FastRPP']['allow_reversing'] is False
    assert collision['StopZone']['action_type'] == 'approach'
    assert COLLISION_STOP_ZONE == ROBOT_FOOTPRINT
    assert _pairs(collision['SlowZone']['points']) == COLLISION_SLOW_ZONE
    assert _footprint_pairs(
        config['local_costmap']['local_costmap']['ros__parameters']['footprint']
    ) == ROBOT_FOOTPRINT


def _pairs(flat_values):
    return list(zip(flat_values[::2], flat_values[1::2]))


def _footprint_pairs(value):
    import ast

    return [tuple(point) for point in ast.literal_eval(value)]
