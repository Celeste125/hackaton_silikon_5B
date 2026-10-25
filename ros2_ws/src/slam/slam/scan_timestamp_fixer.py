import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        # /scan is laptop-local, so reliable costs nothing here.
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        # /scan_raw is the one sensor stream that must cross the WiFi, and a
        # bare depth would make this reader RELIABLE: every lost fragment of a
        # ~5 KB scan then triggers a NACK and a retransmit, which on a
        # congested link feeds back into more loss. Best effort still matches
        # sllidar's reliable writer (a reader may request less), and a dropped
        # scan is cheaper than a retransmit storm -- the next one is 100 ms
        # away, and SLAM is already built to skip scans it cannot place.
        self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, qos_profile_sensor_data)

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
