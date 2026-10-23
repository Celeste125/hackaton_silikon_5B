#!/usr/bin/env python3
"""Reconcile curated candidates and create a grouped YOLO dataset split."""

import argparse
import csv
import hashlib
import re
import shutil
from pathlib import Path


FILENAME_PATTERN = re.compile(r"^(bag\d+)_(\d{9})_(.+)\.jpg$")
SPLITS = ("train", "val", "test")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidates", type=Path)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--chunk-seconds", type=int, default=20)
    parser.add_argument("--seed", type=int, default=50310)
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def split_for_chunk(chunk_id: str, seed: int) -> str:
    digest = hashlib.sha256(f"{seed}:{chunk_id}".encode()).digest()
    value = int.from_bytes(digest[:8], "big") / 2**64
    if value < 0.7:
        return "train"
    if value < 0.9:
        return "val"
    return "test"


def load_reconciled_rows(candidates: Path, chunk_seconds: int, seed: int):
    manifest_path = candidates / "manifest.csv"
    with manifest_path.open(newline="", encoding="utf-8") as stream:
        original_rows = list(csv.DictReader(stream))

    rows_by_key = {
        (row["source_bag"], round(float(row["time_sec"]) * 1000)): row
        for row in original_rows
    }
    reconciled = []
    for image_path in sorted((candidates / "images").glob("*.jpg")):
        match = FILENAME_PATTERN.match(image_path.name)
        if not match:
            raise ValueError(f"Unexpected candidate filename: {image_path.name}")
        bag_id, time_ms_text, scene_type = match.groups()
        key = (bag_id, int(time_ms_text))
        if key not in rows_by_key:
            raise ValueError(f"No manifest row matches candidate image: {image_path.name}")

        row = dict(rows_by_key[key])
        time_sec = float(row["time_sec"])
        chunk_id = f"{bag_id}_t{int(time_sec // chunk_seconds):03d}"
        row.update(
            {
                "filename": image_path.name,
                "source_bag": bag_id,
                "scene_type": scene_type,
                "is_negative_candidate": str(scene_type == "pure_scene").lower(),
                "needs_review": "false",
                "chunk_id": chunk_id,
                "split": split_for_chunk(chunk_id, seed),
                "label_status": "unlabeled",
            }
        )
        reconciled.append(row)
    return original_rows, reconciled


def prepare_dataset_directories(dataset: Path, clean: bool) -> None:
    label_files = list((dataset / "labels").glob("*/*.txt"))
    if clean and label_files:
        raise RuntimeError(
            "Refusing to rebuild image splits because label files already exist"
        )

    for split in SPLITS:
        images_dir = dataset / "images" / split
        labels_dir = dataset / "labels" / split
        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        existing_images = list(images_dir.glob("*.jpg"))
        if existing_images and not clean:
            raise FileExistsError(
                f"Images already exist in {images_dir}; rerun with --clean"
            )
        for image_path in existing_images:
            image_path.unlink()


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    preferred_fields = [
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
        "chunk_id",
        "split",
        "label_status",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=preferred_fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    candidates = args.candidates.resolve()
    dataset = args.dataset.resolve()
    if args.chunk_seconds <= 0:
        raise ValueError("--chunk-seconds must be greater than zero")

    original_rows, rows = load_reconciled_rows(
        candidates, args.chunk_seconds, args.seed
    )
    if not rows:
        raise RuntimeError("No curated candidate images were found")

    extracted_manifest = candidates / "manifest_extracted.csv"
    if not extracted_manifest.exists():
        write_manifest(extracted_manifest, original_rows)

    prepare_dataset_directories(dataset, args.clean)
    for row in rows:
        source = candidates / "images" / row["filename"]
        destination = dataset / "images" / row["split"] / row["filename"]
        shutil.copy2(source, destination)

    write_manifest(candidates / "manifest.csv", rows)
    write_manifest(dataset / "split_manifest.csv", rows)
    print(f"Prepared {len(rows)} curated images")
    for split in SPLITS:
        count = sum(row["split"] == split for row in rows)
        print(f"{split}: {count}")


if __name__ == "__main__":
    main()
