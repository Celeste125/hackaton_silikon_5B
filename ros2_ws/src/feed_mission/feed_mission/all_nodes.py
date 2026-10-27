"""mission, lap_driver and relocalizer in one process, sharing one TF listener.

    ros2 run feed_mission feed_mission        (mission.launch.py starts this)

Three processes would be three DDS participants and three /tf subscriptions,
each decoding every transform (about 56 a second) in Python. One process
needs one of each; on the Jetson that is a good part of a core.
"""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener

from feed_mission.lap_driver import LapDriver
from feed_mission.mission import Mission
from feed_mission.relocalize import Relocalizer


def main(args=None):
    rclpy.init(args=args)
    tf_node = rclpy.create_node('feed_mission_tf')
    tf = Buffer()
    listener = TransformListener(tf, tf_node)  # noqa: F841 (keeps the subscription alive)
    lap, mission, reloc = LapDriver(tf=tf), Mission(tf=tf), Relocalizer(tf=tf)
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (tf_node, lap, mission, reloc):
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        mission.stop_event.set()
        if rclpy.ok():
            mission._safe_outputs()
            lap._finish('idle')
        for node in (reloc, mission, lap, tf_node):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
