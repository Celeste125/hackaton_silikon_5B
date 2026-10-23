#!/usr/bin/env python3
"""Merge manual labels and reviewed pseudo-labels into the final dataset."""

import argparse
import csv
import json
import math
import re
import shutil
import zipfile
from collections import Counter
from pathlib import Path


EXPECTED_CLASSES = {
    0: "red_block",
    1: "yellow_block",
    2: "green_block",
    3: "blue_block",
    4: "blue_obstacle",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("full_dataset", type=Path)
    parser.add_argument("seed_dataset", type=Path)
    parser.add_argument("pseudo_dataset", type=Path)
    parser.add_argument("corrected_archive", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_class_map(content: str) -> dict[int, str]:
    result = {}
    for line in content.splitlines():
        match = re.match(r"\s+(\d+):\s*(\S+)\s*$", line)
        if match:
            result[int(match.group(1))] = match.group(2)
    return result


def polygon_area(coordinates: list[float]) -> float:
    points = list(zip(coordinates[0::2], coordinates[1::2]))
    return abs(
        sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
        )
    ) / 2


def validate_label(content: str, filename: str) -> Counter:
    counts = Counter()
    for line_number, line in enumerate(content.splitlines(), start=1):
        values = line.split()
        if len(values) < 7 or len(values) % 2 == 0:
            raise ValueError(f"{filename}:{line_number}: invalid segmentation polygon")
        class_id = int(values[0])
        if class_id not in EXPECTED_CLASSES:
            raise ValueError(f"{filename}:{line_number}: invalid class {class_id}")
        coordinates = [float(value) for value in values[1:]]
        if any(not math.isfinite(value) or value < 0 or value > 1 for value in coordinates):
            raise ValueError(f"{filename}:{line_number}: coordinate outside [0, 1]")
        if polygon_area(coordinates) <= 1e-6:
            raise ValueError(f"{filename}:{line_number}: polygon area is zero")
        counts[class_id] += 1
    return counts


def prepare_output(output: Path, clean: bool) -> None:
    if output.exists():
        if not clean:
            raise FileExistsError(f"Output already exists: {output}; use --clean to replace it")
        shutil.rmtree(output)
    for split in ("train", "val", "test"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)


def copy_example(
    image: Path,
    label_content: str | None,
    output: Path,
    split: str,
) -> Counter:
    shutil.copy2(image, output / "images" / split / image.name)
    if label_content is None or not label_content.strip():
        return Counter()
    counts = validate_label(label_content, image.name)
    (output / "labels" / split / f"{image.stem}.txt").write_text(
        label_content.strip() + "\n", encoding="utf-8"
    )
    return counts


def write_data_yaml(output: Path) -> None:
    content = f"""path: {output}
train: images/train
val: images/val
test: images/test

names:
  0: red_block
  1: yellow_block
  2: green_block
  3: blue_block
  4: blue_obstacle
"""
    (output / "data.yaml").write_text(content, encoding="utf-8")


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    full_dataset = args.full_dataset.resolve()
    seed_dataset = args.seed_dataset.resolve()
    pseudo_dataset = args.pseudo_dataset.resolve()
    corrected_archive = args.corrected_archive.resolve()
    output = args.output.resolve()
    prepare_output(output, args.clean)

    full_rows = read_csv(full_dataset / "split_manifest.csv")
    full_by_name = {row["filename"]: row for row in full_rows}
    review_rows = read_csv(pseudo_dataset / "cvat_review" / "review_manifest.csv")
    expected_review = {row["filename"] for row in review_rows}

    with zipfile.ZipFile(corrected_archive) as archive:
        class_map = read_class_map(archive.read("data.yaml").decode())
        if class_map != EXPECTED_CLASSES:
            raise ValueError(f"Class mapping mismatch: {class_map}")
        exported = {
            Path(line.strip()).name
            for line in archive.read("train.txt").decode().splitlines()
            if line.strip()
        }
        if not exported <= expected_review:
            raise ValueError(f"Unexpected reviewed images: {sorted(exported - expected_review)}")
        corrected_labels = {
            f"{Path(member).stem}.jpg": archive.read(member).decode()
            for member in archive.namelist()
            if member.startswith("labels/train/") and member.endswith(".txt")
        }

    excluded = (expected_review - exported) | (exported - set(corrected_labels))
    provenance_rows = []
    class_counts = Counter()
    source_counts = Counter()

    for split in ("train", "val", "test"):
        for image in sorted((seed_dataset / "images" / split).glob("*")):
            if not image.is_file():
                continue
            label = seed_dataset / "labels" / split / f"{image.stem}.txt"
            content = label.read_text(encoding="utf-8") if label.exists() else None
            class_counts.update(copy_example(image, content, output, split))
            source = "manual" if content else "manual_background"
            source_counts[source] += 1
            provenance_rows.append(
                {
                    "filename": image.name,
                    "split": split,
                    "scene_type": full_by_name[image.name]["scene_type"],
                    "label_source": source,
                }
            )

    for image in sorted((pseudo_dataset / "images" / "train").glob("*")):
        if not image.is_file() or image.name in excluded:
            continue
        pseudo_label = pseudo_dataset / "labels" / "train" / f"{image.stem}.txt"
        if image.name in corrected_labels:
            content = corrected_labels[image.name]
            source = "reviewed_pseudo"
        elif image.name in expected_review:
            raise RuntimeError(f"Reviewed image has no disposition: {image.name}")
        else:
            content = pseudo_label.read_text(encoding="utf-8")
            source = "high_confidence_pseudo"
        class_counts.update(copy_example(image, content, output, "train"))
        source_counts[source] += 1
        provenance_rows.append(
            {
                "filename": image.name,
                "split": "train",
                "scene_type": full_by_name[image.name]["scene_type"],
                "label_source": source,
            }
        )

    write_data_yaml(output)
    write_manifest(output / "provenance_manifest.csv", provenance_rows)
    report = {
        "excluded_review_images": sorted(excluded),
        "image_sources": dict(source_counts),
        "class_instances": {
            EXPECTED_CLASSES[class_id]: class_counts[class_id]
            for class_id in EXPECTED_CLASSES
        },
    }
    (output / "merge_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    for split in ("train", "val", "test"):
        images = len(list((output / "images" / split).glob("*")))
        labels = len(list((output / "labels" / split).glob("*.txt")))
        print(f"{split}: images={images}, labels={labels}")
    print("sources: " + ", ".join(f"{key}={value}" for key, value in source_counts.items()))
    print(f"excluded reviewed images: {len(excluded)}")
    print(f"output: {output}")


if __name__ == "__main__":
    main()
