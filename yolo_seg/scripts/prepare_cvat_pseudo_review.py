#!/usr/bin/env python3
"""Package selected pseudo-labels for a CVAT review task."""

import argparse
import csv
import shutil
import zipfile
from collections import Counter
from pathlib import Path


DATA_YAML = """names:
  0: red_block
  1: yellow_block
  2: green_block
  3: blue_block
  4: blue_obstacle
path: .
train: train.txt
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("pseudo_dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--auto-samples-per-scene", type=int, default=5)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def select_rows(rows: list[dict[str, str]], samples_per_scene: int) -> list[dict[str, str]]:
    selected = []
    sampled = Counter()
    for row in rows:
        status = row["review_status"]
        if status in {"review_empty", "review_low_confidence"}:
            row["review_group"] = status
            selected.append(row)
        elif status == "auto_accept_candidate" and sampled[row["scene_type"]] < samples_per_scene:
            sampled[row["scene_type"]] += 1
            row["review_group"] = "auto_accept_sample"
            selected.append(row)
    return selected


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    pseudo_dataset = args.pseudo_dataset.resolve()
    output = args.output.resolve()
    if output.exists():
        if not args.clean:
            raise FileExistsError(f"Output already exists: {output}; use --clean to replace it")
        shutil.rmtree(output)
    output.mkdir(parents=True)

    with (pseudo_dataset / "pseudo_label_manifest.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    selected = select_rows(rows, args.auto_samples_per_scene)

    image_archive = output / "review_images.zip"
    annotation_archive = output / "review_annotations.zip"
    train_lines = []
    with zipfile.ZipFile(image_archive, "w", zipfile.ZIP_DEFLATED) as archive:
        for row in selected:
            filename = row["filename"]
            source = pseudo_dataset / "images" / "train" / filename
            member = f"data/images/train/{filename}"
            archive.write(source, member)
            train_lines.append(member)

    with zipfile.ZipFile(annotation_archive, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("data.yaml", DATA_YAML)
        archive.writestr("train.txt", "\n".join(train_lines) + "\n")
        for row in selected:
            filename = row["filename"]
            label = pseudo_dataset / "labels" / "train" / f"{Path(filename).stem}.txt"
            content = label.read_text(encoding="utf-8")
            if content.strip():
                archive.writestr(f"labels/train/{label.name}", content)

    write_manifest(output / "review_manifest.csv", selected)
    counts = Counter(row["review_group"] for row in selected)
    print(f"images: {len(selected)}")
    for group in ("review_empty", "review_low_confidence", "auto_accept_sample"):
        print(f"{group}: {counts[group]}")
    print(f"images archive: {image_archive}")
    print(f"annotations archive: {annotation_archive}")


if __name__ == "__main__":
    main()
