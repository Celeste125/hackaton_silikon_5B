import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Subscribes to the raw LIDAR scan topic /scan_raw, replaces its timestamp
# with the Jetson local clock, and republishes it on /scan.
# Reason: the SLAMTEC C1 clock is not synchronized with the Jetson clock,
# which can make slam_toolbox TF lookups fail.
# Backdate the timestamp by 100 ms so a matching transform is available
# in the TF buffer.

class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        # Use the Jetson current time minus 100 ms to avoid future TF lookups.
        from rclpy.duration import Duration
        msg.header.stamp = (now - Duration(nanoseconds=100_000_000)).to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
