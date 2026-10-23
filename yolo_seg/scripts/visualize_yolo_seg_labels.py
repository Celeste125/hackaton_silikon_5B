#!/usr/bin/env python3
"""Render deterministic YOLO segmentation label overlays for review."""

import argparse
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {
    0: "red_block",
    1: "yellow_block",
    2: "green_block",
    3: "blue_block",
    4: "blue_obstacle",
}
COLORS = {
    0: (40, 40, 230),
    1: (20, 210, 240),
    2: (50, 190, 60),
    3: (220, 90, 30),
    4: (220, 210, 30),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--smallest",
        type=int,
        default=0,
        help="Render images containing the smallest polygons instead of class samples",
    )
    return parser.parse_args()


def read_label(path: Path):
    objects = []
    for line in path.read_text(encoding="utf-8").splitlines():
        values = line.split()
        class_id = int(values[0])
        coordinates = np.array([float(value) for value in values[1:]], dtype=np.float32)
        objects.append((class_id, coordinates.reshape(-1, 2)))
    return objects


def render(image_path: Path, label_path: Path) -> np.ndarray:
    image = cv2.imread(str(image_path))
    if image is None:
        raise OSError(f"Unable to read {image_path}")
    height, width = image.shape[:2]
    overlay = image.copy()
    outlines = []
    for class_id, normalized in read_label(label_path):
        points = normalized.copy()
        points[:, 0] *= width
        points[:, 1] *= height
        points = np.rint(points).astype(np.int32)
        color = COLORS[class_id]
        cv2.fillPoly(overlay, [points], color)
        outlines.append((class_id, points, color))
    image = cv2.addWeighted(overlay, 0.35, image, 0.65, 0)
    for class_id, points, color in outlines:
        cv2.polylines(image, [points], True, color, 2, cv2.LINE_AA)
        x, y = points[0]
        cv2.putText(
            image,
            CLASS_NAMES[class_id],
            (int(x), max(18, int(y) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return image


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    selected = []
    if args.smallest:
        candidates = []
        for split in ("train", "val", "test"):
            for label_path in (dataset / "labels" / split).glob("*.txt"):
                for class_id, points in read_label(label_path):
                    x = points[:, 0]
                    y = points[:, 1]
                    area = abs(
                        np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))
                    ) / 2
                    candidates.append((float(area), split, class_id, label_path))
        used = set()
        for _, split, class_id, label_path in sorted(candidates):
            key = (split, label_path.stem)
            if key in used:
                continue
            used.add(key)
            selected.append((split, class_id, label_path))
            if len(selected) >= args.smallest:
                break
    else:
        for split in ("train", "val", "test"):
            used = set()
            labels = sorted((dataset / "labels" / split).glob("*.txt"))
            objects_by_label = {path: read_label(path) for path in labels}
            for class_id in CLASS_NAMES:
                candidates = [
                    path
                    for path, objects in objects_by_label.items()
                    if any(object_class == class_id for object_class, _ in objects)
                ]
                label_path = next(
                    (path for path in candidates if path.stem not in used), candidates[0]
                )
                used.add(label_path.stem)
                selected.append((split, class_id, label_path))

    for index, (split, class_id, label_path) in enumerate(selected):
        image_path = dataset / "images" / split / f"{label_path.stem}.jpg"
        rendered = render(image_path, label_path)
        output_path = output / (
            f"{index:02d}_{split}_{CLASS_NAMES[class_id]}_{label_path.stem}.jpg"
        )
        if not cv2.imwrite(str(output_path), rendered):
            raise OSError(f"Unable to write {output_path}")
    print(f"Rendered {len(selected)} review images to {output}")


if __name__ == "__main__":
    main()
