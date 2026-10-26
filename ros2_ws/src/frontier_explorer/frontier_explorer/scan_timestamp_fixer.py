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

class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        # The footprint's nearest edge is 0.17 m from base_link.
        self.min_range = self.declare_parameter('min_range', 0.16).value
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        # Use the Jetson current time minus 100 ms to avoid future TF lookups.
        msg.header.stamp = (now - Duration(nanoseconds=100_000_000)).to_msg()
        if self.min_range > msg.range_min:
            msg.range_min = self.min_range
            msg.ranges = [0.0 if r < self.min_range else r for r in msg.ranges]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
