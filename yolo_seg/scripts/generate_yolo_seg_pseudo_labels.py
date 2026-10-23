#!/usr/bin/env python3
"""Generate reviewable YOLO segmentation pseudo-labels for unlabelled train images."""

import argparse
import csv
import shutil
from collections import Counter
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("full_dataset", type=Path)
    parser.add_argument("seed_dataset", type=Path)
    parser.add_argument("model", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--auto-accept-confidence", type=float, default=0.70)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return {row["filename"]: row for row in csv.DictReader(stream)}


def prepare_output(output: Path, clean: bool) -> tuple[Path, Path, Path]:
    if output.exists():
        if not clean:
            raise FileExistsError(f"Output already exists: {output}; use --clean to replace it")
        shutil.rmtree(output)
    images = output / "images" / "train"
    labels = output / "labels" / "train"
    overlays = output / "review_overlays"
    for directory in (images, labels, overlays):
        directory.mkdir(parents=True, exist_ok=True)
    return images, labels, overlays


def format_polygon(class_id: int, polygon) -> str:
    coordinates = " ".join(f"{float(value):.6f}" for point in polygon for value in point)
    return f"{class_id} {coordinates}"


def write_manifest(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    full_dataset = args.full_dataset.resolve()
    seed_dataset = args.seed_dataset.resolve()
    model_path = args.model.resolve()
    output = args.output.resolve()

    images_dir, labels_dir, overlays_dir = prepare_output(output, args.clean)
    full_manifest = load_manifest(full_dataset / "split_manifest.csv")

    seed_names = {
        path.name for path in (seed_dataset / "images" / "train").glob("*") if path.is_file()
    }
    excluded_names = {
        path.name for path in (seed_dataset / "excluded" / "train").glob("*") if path.is_file()
    }
    source_images = sorted(
        path
        for path in (full_dataset / "images" / "train").glob("*")
        if path.is_file() and path.name not in seed_names | excluded_names
    )
    if not source_images:
        raise RuntimeError("No unlabelled train images found")

    model = YOLO(model_path)
    predictions = model.predict(
        source=[str(path) for path in source_images],
        conf=args.conf,
        imgsz=args.imgsz,
        device=args.device,
        stream=True,
        verbose=False,
    )

    rows = []
    status_counts = Counter()
    class_counts = Counter()
    for source_path, result in zip(source_images, predictions, strict=True):
        shutil.copy2(source_path, images_dir / source_path.name)

        classes = []
        confidences = []
        label_lines = []
        boxes = result.boxes
        masks = result.masks
        if boxes is not None and masks is not None:
            class_ids = boxes.cls.int().cpu().tolist()
            scores = boxes.conf.cpu().tolist()
            polygons = masks.xyn
            if not (len(class_ids) == len(scores) == len(polygons)):
                raise RuntimeError(f"Prediction output mismatch for {source_path.name}")
            for class_id, confidence, polygon in zip(class_ids, scores, polygons, strict=True):
                if len(polygon) < 3:
                    continue
                label_lines.append(format_polygon(class_id, polygon))
                classes.append(str(class_id))
                confidences.append(float(confidence))
                class_counts[class_id] += 1

        label_path = labels_dir / f"{source_path.stem}.txt"
        label_path.write_text(
            "\n".join(label_lines) + ("\n" if label_lines else ""), encoding="utf-8"
        )
        cv2.imwrite(str(overlays_dir / source_path.name), result.plot())

        if not confidences:
            status = "review_empty"
        elif min(confidences) < args.auto_accept_confidence:
            status = "review_low_confidence"
        else:
            status = "auto_accept_candidate"
        status_counts[status] += 1

        source_row = full_manifest[source_path.name]
        rows.append(
            {
                "filename": source_path.name,
                "scene_type": source_row["scene_type"],
                "source_bag": source_row["source_bag"],
                "time_sec": source_row["time_sec"],
                "detection_count": str(len(label_lines)),
                "class_ids": ";".join(classes),
                "confidences": ";".join(f"{value:.4f}" for value in confidences),
                "min_confidence": f"{min(confidences):.4f}" if confidences else "",
                "review_status": status,
            }
        )

    write_manifest(output / "pseudo_label_manifest.csv", rows)
    shutil.copy2(full_dataset / "data.yaml", output / "source_data.yaml")
    print(f"images: {len(rows)}")
    for status in ("auto_accept_candidate", "review_low_confidence", "review_empty"):
        print(f"{status}: {status_counts[status]}")
    print("instances: " + ", ".join(f"class_{key}={class_counts[key]}" for key in sorted(class_counts)))
    print(f"output: {output}")


if __name__ == "__main__":
    main()
