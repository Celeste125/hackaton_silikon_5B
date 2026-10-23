#!/usr/bin/env python3
"""Extract time-spaced RGB images from a ROS 2 sqlite3 bag."""

import argparse
from pathlib import Path

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("output", type=Path, help="Directory for extracted JPEGs")
    parser.add_argument(
        "--topic",
        default="/camera/camera/color/image_raw",
        help="sensor_msgs/msg/Image topic to extract",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Minimum time between saved images in seconds",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        choices=range(1, 101),
        metavar="1-100",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Stop after this many images; 0 extracts the whole bag",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.interval <= 0:
        raise ValueError("--interval must be greater than zero")
    if not args.bag.is_dir():
        raise FileNotFoundError(f"Bag directory does not exist: {args.bag}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if args.topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise ValueError(f"Topic {args.topic!r} not found. Available: {available}")

    message_type = get_message(topic_types[args.topic])
    bridge = CvBridge()
    args.output.mkdir(parents=True, exist_ok=True)

    interval_ns = int(args.interval * 1_000_000_000)
    last_saved_ns = None
    saved = 0
    bag_name = args.bag.name

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != args.topic:
            continue
        if last_saved_ns is not None and timestamp_ns - last_saved_ns < interval_ns:
            continue

        message = deserialize_message(data, message_type)
        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        output_path = args.output / f"{bag_name}_{timestamp_ns}.jpg"
        written = cv2.imwrite(
            str(output_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
        )
        if not written:
            raise OSError(f"Failed to write image: {output_path}")

        saved += 1
        last_saved_ns = timestamp_ns
        if args.max_images and saved >= args.max_images:
            break

    print(f"Saved {saved} images from {args.bag} to {args.output}")


if __name__ == "__main__":
    main()
