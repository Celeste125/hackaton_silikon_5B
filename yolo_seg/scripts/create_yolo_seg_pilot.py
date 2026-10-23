#!/usr/bin/env python3
"""Create a scene-balanced pilot subset from an existing YOLO split."""

import argparse
import csv
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path


TARGETS = {"train": 200, "val": 60, "test": 30}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Full YOLO dataset directory")
    parser.add_argument("output", type=Path, help="Pilot dataset directory")
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def allocate_quotas(rows: list[dict[str, str]], target: int) -> dict[str, int]:
    counts = Counter(row["scene_type"] for row in rows)
    quotas = {scene: 0 for scene in counts}

    pure_count = counts.get("pure_scene", 0)
    if pure_count:
        quotas["pure_scene"] = pure_count

    non_pure = {scene: count for scene, count in counts.items() if scene != "pure_scene"}
    remaining_target = target - pure_count
    remaining_total = sum(non_pure.values())
    ideals = {
        scene: remaining_target * count / remaining_total
        for scene, count in non_pure.items()
    }
    for scene, ideal in ideals.items():
        quotas[scene] = min(non_pure[scene], max(1, math.floor(ideal)))

    while sum(quotas.values()) < target:
        available = [scene for scene in non_pure if quotas[scene] < non_pure[scene]]
        scene = max(available, key=lambda name: ideals[name] - quotas[name])
        quotas[scene] += 1
    while sum(quotas.values()) > target:
        available = [scene for scene in non_pure if quotas[scene] > 1]
        scene = max(available, key=lambda name: quotas[name] - ideals[name])
        quotas[scene] -= 1
    return quotas


def select_evenly(rows: list[dict[str, str]], count: int) -> list[dict[str, str]]:
    ordered = sorted(rows, key=lambda row: (row["source_bag"], float(row["time_sec"])))
    if count >= len(ordered):
        return ordered
    if count == 1:
        return [ordered[len(ordered) // 2]]
    indices = [round(index * (len(ordered) - 1) / (count - 1)) for index in range(count)]
    return [ordered[index] for index in indices]


def prepare_directories(output: Path, clean: bool) -> None:
    label_files = list((output / "labels").glob("*/*.txt"))
    if clean and label_files:
        raise RuntimeError("Refusing to rebuild Pilot because label files already exist")

    for split in TARGETS:
        images_dir = output / "images" / split
        labels_dir = output / "labels" / split
        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        existing_images = list(images_dir.glob("*.jpg"))
        if existing_images and not clean:
            raise FileExistsError(f"Pilot images already exist in {images_dir}")
        for image_path in existing_images:
            image_path.unlink()


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    fields = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


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


def main() -> None:
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    with (source / "split_manifest.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    prepare_directories(output, args.clean)
    selected: list[dict[str, str]] = []
    by_split = defaultdict(list)
    for row in rows:
        by_split[row["split"]].append(row)

    for split, target in TARGETS.items():
        quotas = allocate_quotas(by_split[split], target)
        by_scene = defaultdict(list)
        for row in by_split[split]:
            by_scene[row["scene_type"]].append(row)
        for scene, quota in quotas.items():
            selected.extend(select_evenly(by_scene[scene], quota))

    selected.sort(key=lambda row: (row["split"], row["source_bag"], float(row["time_sec"])))
    for row in selected:
        source_path = source / "images" / row["split"] / row["filename"]
        destination = output / "images" / row["split"] / row["filename"]
        shutil.copy2(source_path, destination)

    write_manifest(output / "pilot_manifest.csv", selected)
    write_data_yaml(output)
    for split, target in TARGETS.items():
        count = sum(row["split"] == split for row in selected)
        print(f"{split}: {count}/{target}")


if __name__ == "__main__":
    main()
