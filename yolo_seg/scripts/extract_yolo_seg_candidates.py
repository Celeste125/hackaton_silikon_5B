#!/usr/bin/env python3
"""Extract a unified YOLO segmentation candidate pool from ROS 2 bags."""

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass(frozen=True)
class Region:
    region_id: str
    start_sec: float
    end_sec: float
    scene_type: str
    interval_sec: float
    is_negative_candidate: bool
    needs_review: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path, help="JSON extraction plan")
    parser.add_argument("output", type=Path, help="Candidate dataset directory")
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        choices=range(1, 101),
        metavar="1-100",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove an existing candidate image pool before extraction",
    )
    return parser.parse_args()


def load_plan(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        plan = json.load(stream)
    if not plan.get("bags"):
        raise ValueError("Extraction plan contains no bags")
    return plan


def build_regions(
    bag: dict[str, Any],
    default_interval_sec: float,
    transition_interval_sec: float,
    include_transitions: bool,
) -> list[Region]:
    duration_sec = float(bag["duration_sec"])
    segments = sorted(bag["segments"], key=lambda item: float(item["start_sec"]))
    regions: list[Region] = []
    cursor = 0.0
    transition_index = 0

    for segment_index, segment in enumerate(segments):
        start_sec = float(segment["start_sec"])
        end_sec = float(segment["end_sec"])
        if start_sec < cursor or end_sec <= start_sec or end_sec > duration_sec:
            raise ValueError(f"Invalid or overlapping segment in {bag['bag_id']}: {segment}")

        if include_transitions and start_sec > cursor:
            regions.append(
                Region(
                    region_id=f"transition_{transition_index:02d}",
                    start_sec=cursor,
                    end_sec=start_sec,
                    scene_type="transition_review",
                    interval_sec=transition_interval_sec,
                    is_negative_candidate=False,
                    needs_review=True,
                )
            )
            transition_index += 1

        scene_type = str(segment["scene_type"])
        regions.append(
            Region(
                region_id=f"segment_{segment_index:02d}",
                start_sec=start_sec,
                end_sec=end_sec,
                scene_type=scene_type,
                interval_sec=float(segment.get("interval_sec", default_interval_sec)),
                is_negative_candidate=scene_type == "pure_scene",
                needs_review=False,
            )
        )
        cursor = end_sec

    if include_transitions and cursor < duration_sec:
        regions.append(
            Region(
                region_id=f"transition_{transition_index:02d}",
                start_sec=cursor,
                end_sec=duration_sec,
                scene_type="transition_review",
                interval_sec=transition_interval_sec,
                is_negative_candidate=False,
                needs_review=True,
            )
        )

    return regions


def prepare_output(output: Path, clean: bool) -> tuple[Path, Path]:
    images_dir = output / "images"
    manifest_path = output / "manifest.csv"
    existing_images = list(images_dir.glob("*.jpg")) if images_dir.exists() else []
    if (existing_images or manifest_path.exists()) and not clean:
        raise FileExistsError(
            f"Candidate output already exists at {output}; rerun with --clean to replace it"
        )
    if clean:
        for image_path in existing_images:
            image_path.unlink()
        if manifest_path.exists():
            manifest_path.unlink()
    images_dir.mkdir(parents=True, exist_ok=True)
    return images_dir, manifest_path


def extract_bag(
    bag: dict[str, Any],
    plan_path: Path,
    topic_name: str,
    regions: list[Region],
    images_dir: Path,
    jpeg_quality: int,
) -> list[dict[str, Any]]:
    bag_path = (plan_path.parent / bag["path"]).resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory does not exist: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if topic_name not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise ValueError(f"Topic {topic_name!r} not found in {bag_path}. Available: {available}")

    message_type = get_message(topic_types[topic_name])
    bridge = CvBridge()
    origin_timestamp_ns = None
    region_index = 0
    last_saved_ns: int | None = None
    rows: list[dict[str, Any]] = []

    while reader.has_next() and region_index < len(regions):
        topic, data, timestamp_ns = reader.read_next()
        if topic != topic_name:
            continue
        if origin_timestamp_ns is None:
            origin_timestamp_ns = timestamp_ns

        time_sec = (timestamp_ns - origin_timestamp_ns) / 1_000_000_000
        while region_index < len(regions) and time_sec >= regions[region_index].end_sec:
            region_index += 1
            last_saved_ns = None
        if region_index >= len(regions):
            break

        region = regions[region_index]
        if time_sec < region.start_sec:
            continue
        interval_ns = int(region.interval_sec * 1_000_000_000)
        if last_saved_ns is not None and timestamp_ns - last_saved_ns < interval_ns:
            continue

        message = deserialize_message(data, message_type)
        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        time_ms = round(time_sec * 1000)
        filename = (
            f"{bag['bag_id']}_{time_ms:09d}_{region.scene_type}.jpg"
        )
        output_path = images_dir / filename
        written = cv2.imwrite(
            str(output_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
        )
        if not written:
            raise OSError(f"Failed to write image: {output_path}")

        rows.append(
            {
                "filename": filename,
                "source_bag": bag["bag_id"],
                "source_path": str(bag_path),
                "timestamp_ns": timestamp_ns,
                "time_sec": f"{time_sec:.3f}",
                "scene_type": region.scene_type,
                "region_id": region.region_id,
                "sample_interval_sec": f"{region.interval_sec:.1f}",
                "is_negative_candidate": str(region.is_negative_candidate).lower(),
                "needs_review": str(region.needs_review).lower(),
                "width": image.shape[1],
                "height": image.shape[0],
                "split": "",
                "label_status": "unlabeled",
            }
        )
        last_saved_ns = timestamp_ns

    print(f"{bag['bag_id']}: saved {len(rows)} candidate images")
    return rows


def write_manifest(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "filename",
        "source_bag",
        "source_path",
        "timestamp_ns",
        "time_sec",
        "scene_type",
        "region_id",
        "sample_interval_sec",
        "is_negative_candidate",
        "needs_review",
        "width",
        "height",
        "split",
        "label_status",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    plan_path = args.plan.resolve()
    plan = load_plan(plan_path)
    images_dir, manifest_path = prepare_output(args.output.resolve(), args.clean)

    default_interval_sec = float(plan["default_interval_sec"])
    transition_interval_sec = float(plan["transition_interval_sec"])
    include_transitions = bool(plan.get("include_transitions", True))
    if default_interval_sec <= 0 or (
        include_transitions and transition_interval_sec <= 0
    ):
        raise ValueError("Extraction intervals must be greater than zero")

    all_rows: list[dict[str, Any]] = []
    for bag in plan["bags"]:
        regions = build_regions(
            bag,
            default_interval_sec,
            transition_interval_sec,
            include_transitions,
        )
        all_rows.extend(
            extract_bag(
                bag=bag,
                plan_path=plan_path,
                topic_name=plan["topic"],
                regions=regions,
                images_dir=images_dir,
                jpeg_quality=args.jpeg_quality,
            )
        )

    write_manifest(manifest_path, all_rows)
    print(f"Total: saved {len(all_rows)} images")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
