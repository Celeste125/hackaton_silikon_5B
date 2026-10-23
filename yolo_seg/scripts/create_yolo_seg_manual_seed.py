#!/usr/bin/env python3
"""Create the manually annotated seed set from the YOLO pilot dataset."""

import argparse
import csv
import shutil
from collections import Counter, defaultdict
from pathlib import Path


TRAIN_QUOTAS = {
    "red_block": 25,
    "yellow_block": 20,
    "green_block": 15,
    "blue_block": 20,
    "blue_obstacle": 20,
    "blue_block_blue_obstacle": 30,
    "multiple_blocks": 14,
    "pure_scene": 6,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Pilot dataset directory")
    parser.add_argument("output", type=Path, help="Manual seed dataset directory")
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def select_evenly(rows: list[dict[str, str]], count: int) -> list[dict[str, str]]:
    ordered = sorted(rows, key=lambda row: (row["source_bag"], float(row["time_sec"])))
    if count > len(ordered):
        raise ValueError(f"Requested {count} images from a group containing {len(ordered)}")
    if count == len(ordered):
        return ordered
    if count == 1:
        return [ordered[len(ordered) // 2]]
    indices = [round(index * (len(ordered) - 1) / (count - 1)) for index in range(count)]
    return [ordered[index] for index in indices]


def prepare_output(output: Path, clean: bool) -> None:
    labels = list((output / "labels").glob("*/*.txt"))
    if clean and labels:
        raise RuntimeError("Refusing to rebuild seed set because annotations exist")
    for split in ("train", "val", "test"):
        images_dir = output / "images" / split
        labels_dir = output / "labels" / split
        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        existing = list(images_dir.glob("*.jpg"))
        if existing and not clean:
            raise FileExistsError(f"Seed images already exist in {images_dir}")
        for image_path in existing:
            image_path.unlink()


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
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
    with (source / "pilot_manifest.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    prepare_output(output, args.clean)
    train_by_scene = defaultdict(list)
    for row in rows:
        if row["split"] == "train":
            train_by_scene[row["scene_type"]].append(row)

    selected = []
    for scene, quota in TRAIN_QUOTAS.items():
        selected.extend(select_evenly(train_by_scene[scene], quota))
    selected.extend(row for row in rows if row["split"] in {"val", "test"})
    selected.sort(key=lambda row: (row["split"], row["source_bag"], float(row["time_sec"])))

    for row in selected:
        source_path = source / "images" / row["split"] / row["filename"]
        destination = output / "images" / row["split"] / row["filename"]
        shutil.copy2(source_path, destination)

    write_manifest(output / "seed_manifest.csv", selected)
    write_data_yaml(output)
    counts = Counter(row["split"] for row in selected)
    print(f"train: {counts['train']}")
    print(f"val: {counts['val']}")
    print(f"test: {counts['test']}")


if __name__ == "__main__":
    main()
