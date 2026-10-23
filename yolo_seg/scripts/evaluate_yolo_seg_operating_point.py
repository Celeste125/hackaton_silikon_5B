#!/usr/bin/env python3
"""Evaluate segmentation models at deployment-style confidence thresholds."""

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


CLASS_NAMES = {
    0: "red_block",
    1: "yellow_block",
    2: "green_block",
    3: "blue_block",
    4: "blue_obstacle",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--model",
        action="append",
        required=True,
        metavar="NAME=PATH",
        help="Model name and weights; may be repeated",
    )
    parser.add_argument("--thresholds", type=float, nargs="+", default=[0.25, 0.35, 0.5, 0.7])
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    return parser.parse_args()


def rasterize(points: np.ndarray, width: int, height: int, normalized: bool) -> np.ndarray:
    polygon = points.astype(np.float64).copy()
    if normalized:
        polygon[:, 0] *= width
        polygon[:, 1] *= height
    polygon[:, 0] = np.clip(np.rint(polygon[:, 0]), 0, width - 1)
    polygon[:, 1] = np.clip(np.rint(polygon[:, 1]), 0, height - 1)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon.astype(np.int32)], 1)
    return mask.astype(bool)


def read_ground_truth(label_path: Path, width: int, height: int) -> list[dict]:
    if not label_path.exists():
        return []
    objects = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        values = line.split()
        class_id = int(values[0])
        points = np.asarray([float(value) for value in values[1:]], dtype=np.float64).reshape(-1, 2)
        objects.append(
            {"class_id": class_id, "mask": rasterize(points, width, height, normalized=True)}
        )
    return objects


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    union = np.logical_or(first, second).sum()
    return float(np.logical_and(first, second).sum() / union) if union else 0.0


def match_instances(ground_truth: list[dict], predictions: list[dict], threshold: float):
    candidates = []
    for gt_index, gt in enumerate(ground_truth):
        for pred_index, prediction in enumerate(predictions):
            if gt["class_id"] != prediction["class_id"]:
                continue
            iou = mask_iou(gt["mask"], prediction["mask"])
            if iou >= threshold:
                candidates.append((iou, gt_index, pred_index))
    matched_gt = set()
    matched_predictions = set()
    matches = []
    for iou, gt_index, pred_index in sorted(candidates, reverse=True):
        if gt_index in matched_gt or pred_index in matched_predictions:
            continue
        matched_gt.add(gt_index)
        matched_predictions.add(pred_index)
        matches.append((gt_index, pred_index, iou))
    return matches, matched_gt, matched_predictions


def summarize_threshold(images: list[dict], confidence: float, iou_threshold: float) -> dict:
    tp = Counter()
    fp = Counter()
    fn = Counter()
    matched_ious = Counter()
    matched_counts = Counter()
    issues = []
    negative_false_positives = []
    class_confusions = Counter()

    for image in images:
        predictions = [item for item in image["predictions"] if item["confidence"] >= confidence]
        ground_truth = image["ground_truth"]
        matches, matched_gt, matched_predictions = match_instances(
            ground_truth, predictions, iou_threshold
        )
        for gt_index, _, iou in matches:
            class_id = ground_truth[gt_index]["class_id"]
            tp[class_id] += 1
            matched_ious[class_id] += iou
            matched_counts[class_id] += 1
        for index, prediction in enumerate(predictions):
            if index not in matched_predictions:
                fp[prediction["class_id"]] += 1
        for index, gt in enumerate(ground_truth):
            if index not in matched_gt:
                fn[gt["class_id"]] += 1

        unmatched_gt = [index for index in range(len(ground_truth)) if index not in matched_gt]
        unmatched_pred = [index for index in range(len(predictions)) if index not in matched_predictions]
        for pred_index in unmatched_pred:
            prediction = predictions[pred_index]
            best = None
            for gt_index in unmatched_gt:
                gt = ground_truth[gt_index]
                iou = mask_iou(gt["mask"], prediction["mask"])
                if best is None or iou > best[0]:
                    best = (iou, gt["class_id"])
            if best and best[0] >= iou_threshold and best[1] != prediction["class_id"]:
                class_confusions[(best[1], prediction["class_id"])] += 1

        false_classes = [CLASS_NAMES[predictions[index]["class_id"]] for index in unmatched_pred]
        missed_classes = [CLASS_NAMES[ground_truth[index]["class_id"]] for index in unmatched_gt]
        if false_classes or missed_classes:
            issues.append(
                {
                    "filename": image["filename"],
                    "false_positive_classes": false_classes,
                    "missed_classes": missed_classes,
                }
            )
        if not ground_truth and predictions:
            negative_false_positives.append(image["filename"])

    per_class = {}
    for class_id, name in CLASS_NAMES.items():
        precision = tp[class_id] / (tp[class_id] + fp[class_id]) if tp[class_id] + fp[class_id] else 0.0
        recall = tp[class_id] / (tp[class_id] + fn[class_id]) if tp[class_id] + fn[class_id] else 0.0
        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
        per_class[name] = {
            "tp": tp[class_id],
            "fp": fp[class_id],
            "fn": fn[class_id],
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1": round(f1, 4),
            "mean_matched_mask_iou": round(
                matched_ious[class_id] / matched_counts[class_id], 4
            )
            if matched_counts[class_id]
            else None,
        }
    total_tp = sum(tp.values())
    total_fp = sum(fp.values())
    total_fn = sum(fn.values())
    return {
        "confidence": confidence,
        "mask_iou_threshold": iou_threshold,
        "overall": {
            "tp": total_tp,
            "fp": total_fp,
            "fn": total_fn,
            "precision": round(total_tp / (total_tp + total_fp), 4) if total_tp + total_fp else 0.0,
            "recall": round(total_tp / (total_tp + total_fn), 4) if total_tp + total_fn else 0.0,
        },
        "per_class": per_class,
        "class_confusions": {
            f"{CLASS_NAMES[true_id]}->{CLASS_NAMES[predicted_id]}": count
            for (true_id, predicted_id), count in class_confusions.items()
        },
        "negative_false_positive_images": negative_false_positives,
        "issues": issues,
    }


def evaluate_model(model_path: Path, dataset: Path, thresholds: list[float], args) -> dict:
    image_paths = sorted((dataset / "images" / "test").glob("*"))
    model = YOLO(model_path)
    results = model.predict(
        source=[str(path) for path in image_paths],
        conf=min(thresholds),
        imgsz=args.imgsz,
        device=args.device,
        retina_masks=True,
        stream=True,
        verbose=False,
    )
    images = []
    for image_path, result in zip(image_paths, results, strict=True):
        height, width = result.orig_shape
        ground_truth = read_ground_truth(
            dataset / "labels" / "test" / f"{image_path.stem}.txt", width, height
        )
        predictions = []
        if result.boxes is not None and result.masks is not None:
            for class_id, confidence, polygon in zip(
                result.boxes.cls.int().cpu().tolist(),
                result.boxes.conf.cpu().tolist(),
                result.masks.xy,
                strict=True,
            ):
                if len(polygon) < 3:
                    continue
                predictions.append(
                    {
                        "class_id": class_id,
                        "confidence": float(confidence),
                        "mask": rasterize(np.asarray(polygon), width, height, normalized=False),
                    }
                )
        images.append(
            {"filename": image_path.name, "ground_truth": ground_truth, "predictions": predictions}
        )
    return {
        "weights": str(model_path),
        "test_images": len(images),
        "thresholds": [summarize_threshold(images, value, args.iou) for value in thresholds],
    }


def main() -> None:
    args = parse_args()
    dataset = args.dataset.resolve()
    models = []
    for specification in args.model:
        if "=" not in specification:
            raise ValueError("--model must use NAME=PATH")
        name, path = specification.split("=", 1)
        models.append((name, Path(path).resolve()))
    report = {
        "dataset": str(dataset),
        "models": {
            name: evaluate_model(path, dataset, args.thresholds, args) for name, path in models
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
