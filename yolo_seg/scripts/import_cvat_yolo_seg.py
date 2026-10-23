#!/usr/bin/env python3
"""Validate and import standalone CVAT YOLO segmentation task exports."""

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
ARCHIVES = {
    "train": "cvat_train_seg.zip",
    "val": "cvat_val_seg.zip",
    "test": "cvat_test_seg.zip",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("exports", type=Path)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def read_class_map(data_yaml: str) -> dict[int, str]:
    result = {}
    for line in data_yaml.splitlines():
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
    class_counts = Counter()
    for line_number, line in enumerate(content.splitlines(), start=1):
        values = line.split()
        if len(values) < 7 or len(values) % 2 == 0:
            raise ValueError(
                f"{filename}:{line_number}: expected class plus at least 3 polygon points"
            )
        class_id = int(values[0])
        if class_id not in EXPECTED_CLASSES:
            raise ValueError(f"{filename}:{line_number}: invalid class ID {class_id}")
        coordinates = [float(value) for value in values[1:]]
        if any(not math.isfinite(value) or value < 0 or value > 1 for value in coordinates):
            raise ValueError(f"{filename}:{line_number}: coordinate outside [0, 1]")
        if polygon_area(coordinates) <= 1e-6:
            raise ValueError(f"{filename}:{line_number}: polygon area is zero")
        class_counts[class_id] += 1
    return class_counts


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    exports = args.exports.resolve()
    manifest_path = dataset / "seed_manifest.csv"
    with manifest_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    expected_by_split = {
        split: {Path(row["filename"]).stem: row for row in rows if row["split"] == split}
        for split in ARCHIVES
    }

    if args.clean:
        for split in ARCHIVES:
            for label_path in (dataset / "labels" / split).glob("*.txt"):
                label_path.unlink()
    elif any((dataset / "labels").glob("*/*.txt")):
        raise FileExistsError("Label directories are not empty; use --clean to replace them")

    imported_stems = {}
    excluded_stems = {}
    class_counts = Counter()
    report = {"splits": {}, "class_instances": {}}

    for split, archive_name in ARCHIVES.items():
        archive_path = exports / archive_name
        expected = expected_by_split[split]
        with zipfile.ZipFile(archive_path) as archive:
            class_map = read_class_map(archive.read("data.yaml").decode())
            if class_map != EXPECTED_CLASSES:
                raise ValueError(f"Class mapping mismatch in {archive_name}: {class_map}")

            listed = {
                Path(line.strip()).stem
                for line in archive.read("train.txt").decode().splitlines()
                if line.strip()
            }
            unknown = listed - set(expected)
            if unknown:
                raise ValueError(f"Unknown images in {archive_name}: {sorted(unknown)}")

            labels = {
                Path(name).stem: name
                for name in archive.namelist()
                if name.startswith("labels/train/") and name.endswith(".txt")
            }
            if set(labels) - listed:
                raise ValueError(f"Labels without listed images in {archive_name}")

            for stem in listed:
                scene = expected[stem]["scene_type"]
                has_label = stem in labels
                if scene == "pure_scene" and has_label:
                    raise ValueError(f"Pure scene unexpectedly has a label: {stem}")
                if scene != "pure_scene" and not has_label:
                    raise ValueError(f"Positive image is missing a label: {stem}")

            labels_dir = dataset / "labels" / split
            labels_dir.mkdir(parents=True, exist_ok=True)
            for stem, member in labels.items():
                content = archive.read(member).decode().strip() + "\n"
                counts = validate_label(content, member)
                class_counts.update(counts)
                (labels_dir / f"{stem}.txt").write_text(content, encoding="utf-8")

        imported_stems[split] = listed
        excluded_stems[split] = set(expected) - listed
        report["splits"][split] = {
            "images": len(listed),
            "labels": len(labels),
            "excluded": sorted(excluded_stems[split]),
        }

    backup_manifest = dataset / "seed_manifest_before_cvat_import.csv"
    if not backup_manifest.exists():
        shutil.copy2(manifest_path, backup_manifest)

    excluded_root = dataset / "excluded"
    filtered_rows = []
    for row in rows:
        split = row["split"]
        stem = Path(row["filename"]).stem
        if stem in excluded_stems[split]:
            source = dataset / "images" / split / row["filename"]
            destination = excluded_root / split / row["filename"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            if source.exists():
                shutil.move(source, destination)
            continue
        row["label_status"] = (
            "negative" if row["scene_type"] == "pure_scene" else "annotated"
        )
        filtered_rows.append(row)

    write_manifest(manifest_path, filtered_rows)
    report["class_instances"] = {
        EXPECTED_CLASSES[class_id]: class_counts[class_id]
        for class_id in EXPECTED_CLASSES
    }
    (dataset / "cvat_import_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    print(f"Imported {sum(len(value) for value in imported_stems.values())} images")
    print(f"Imported {sum(class_counts.values())} object instances")
    print(f"Excluded {sum(len(value) for value in excluded_stems.values())} images")
    for class_id, name in EXPECTED_CLASSES.items():
        print(f"{name}: {class_counts[class_id]}")


if __name__ == "__main__":
    main()
