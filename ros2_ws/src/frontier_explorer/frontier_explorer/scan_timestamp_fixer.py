import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Subscribes to the raw LIDAR scan topic /scan_raw, replaces its timestamp
# with the Jetson local clock, and republishes it on /scan.
# Reason: the SLAMTEC C1 clock is not synchronized with the Jetson clock,
# which can make slam_toolbox TF lookups fail.
# Backdate the timestamp by 100 ms so a matching transform is available
# in the TF buffer.
#
# It also drops returns closer than min_range: the legs of the platform above
# the lidar sit 7-13 cm from it, inside the footprint. Left in, the collision
# monitor sees them as a collision already happening and stops the robot, and
# the costmaps mark lethal cells under it. Those readings are set to 0 and
# range_min is raised, so every consumer (costmaps, collision monitor, AMCL,
# slam_toolbox) skips them. Not inf: the costmaps treat inf as "nothing up to
# max range" and would clear real obstacles behind the legs.
#
# It drops every return inside the robot's own outline (footprint_half_length
# x footprint_half_width around the lidar) for the same reason: nothing but the
# robot can be there. The door and its chute on the right side reach into the
# lidar's view when the door opens; seen as "something touching the side" they
# stopped the laps and closed the door, which cleared the view, which opened
# the door again. The outline is symmetric, so the lidar being mounted turned
# 180 degrees does not matter.

class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        # The footprint's nearest edge is 0.17 m from base_link.
        self.min_range = self.declare_parameter('min_range', 0.16).value
        # As in nav2_params local_costmap, minus 5 mm so a real touch at the edge stays.
        self.half_length = self.declare_parameter('footprint_half_length', 0.165).value
        self.half_width = self.declare_parameter('footprint_half_width', 0.255).value
        self.angles = None
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        # Use the Jetson current time minus 100 ms to avoid future TF lookups.
        msg.header.stamp = (now - Duration(nanoseconds=100_000_000)).to_msg()
        r = np.asarray(msg.ranges, dtype=np.float32)
        if self.angles is None or len(self.angles[0]) != len(r):
            a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
            self.angles = (np.abs(np.cos(a)), np.abs(np.sin(a)))
        with np.errstate(invalid='ignore'):
            inside = (r * self.angles[0] < self.half_length) & (r * self.angles[1] < self.half_width)
        if self.min_range > msg.range_min:
            msg.range_min = self.min_range
            inside |= r < self.min_range
        r[inside] = 0.0  # below range_min: every consumer skips it
        msg.ranges = r.tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
