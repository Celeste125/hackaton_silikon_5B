#!/usr/bin/env python3
"""Run the frozen segmentation model on extracted RGB images using CPU only."""

import argparse
import csv
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO


EXPECTED_CLASSES = {
    0: 'red_block',
    1: 'yellow_block',
    2: 'green_block',
    3: 'blue_block',
    4: 'blue_obstacle',
}
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png'}


def parse_args():
    parser = argparse.ArgumentParser(
        description='CPU-only replay of the frozen IMDL YOLO segmentation model.'
    )
    parser.add_argument('--input', required=True, type=Path, help='JPEG/PNG directory')
    parser.add_argument('--output', required=True, type=Path, help='New evidence directory')
    parser.add_argument(
        '--model',
        type=Path,
        default=Path('ros2_ws/src/perception/models/yolo11n_seg_imdl.pt'),
    )
    parser.add_argument('--confidence', type=float, default=0.35)
    parser.add_argument('--iou', type=float, default=0.70)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--limit', type=int, default=0, help='0 processes every image')
    parser.add_argument(
        '--device',
        default='cpu',
        choices=('cpu',),
        help='Fixed to CPU so offline evidence never uses CUDA.',
    )
    return parser.parse_args()


def model_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def image_paths(input_dir, limit):
    paths = sorted(
        path for path in input_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    return paths[:limit] if limit else paths


def class_names(names):
    normalized = {int(index): str(name) for index, name in dict(names).items()}
    if normalized != EXPECTED_CLASSES:
        raise RuntimeError(
            f'Unexpected model classes: {normalized}; expected {EXPECTED_CLASSES}'
        )
    return normalized


def polygon_points(result, index):
    if result.masks is None or index >= len(result.masks.xy):
        return []
    return [[round(float(x), 3), round(float(y), 3)] for x, y in result.masks.xy[index]]


def serialise_detections(result, names):
    if result.boxes is None:
        return []
    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    detections = []
    for index, (box, confidence, class_id) in enumerate(
        zip(boxes, confidences, class_ids)
    ):
        x1, y1, x2, y2 = (float(value) for value in box)
        detections.append({
            'class_id': int(class_id),
            'class_name': names[int(class_id)],
            'confidence': round(float(confidence), 6),
            'bbox_xyxy_px': [round(value, 3) for value in (x1, y1, x2, y2)],
            'center_px': {
                'u': round((x1 + x2) / 2.0, 3),
                'v': round((y1 + y2) / 2.0, 3),
            },
            'mask_polygon_px': polygon_points(result, index),
        })
    return detections


def annotate(image, detections):
    canvas = image.copy()
    overlay = image.copy()
    for index, detection in enumerate(detections):
        colour = colour_for_class(detection['class_id'])
        polygon = np.asarray(detection['mask_polygon_px'], dtype=np.int32)
        if len(polygon) >= 3:
            cv2.fillPoly(overlay, [polygon], colour)
    canvas = cv2.addWeighted(overlay, 0.30, canvas, 0.70, 0.0)

    for detection in detections:
        colour = colour_for_class(detection['class_id'])
        x1, y1, x2, y2 = (int(round(value)) for value in detection['bbox_xyxy_px'])
        center = detection['center_px']
        cv2.rectangle(canvas, (x1, y1), (x2, y2), colour, 2)
        cv2.circle(
            canvas, (int(round(center['u'])), int(round(center['v']))), 3, colour, -1
        )
        label = f"{detection['class_name']} {detection['confidence']:.2f}"
        cv2.putText(
            canvas, label, (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, colour, 1, cv2.LINE_AA,
        )
    return canvas


def colour_for_class(class_id):
    return {
        0: (0, 0, 255),
        1: (0, 255, 255),
        2: (0, 180, 0),
        3: (255, 0, 0),
        4: (255, 180, 0),
    }[class_id]


def write_contact_sheet(annotation_paths, output_path, columns=4, max_images=64):
    selected = annotation_paths[:max_images]
    if not selected:
        return
    tiles = []
    for path in selected:
        image = cv2.imread(str(path))
        if image is None:
            continue
        scale = 320.0 / image.shape[1]
        tiles.append(cv2.resize(image, (320, int(round(image.shape[0] * scale)))))
    if not tiles:
        return
    tile_height = max(tile.shape[0] for tile in tiles)
    padded = [
        cv2.copyMakeBorder(
            tile, 0, tile_height - tile.shape[0], 0, 0,
            cv2.BORDER_CONSTANT, value=(30, 30, 30),
        )
        for tile in tiles
    ]
    blank = np.full_like(padded[0], 30)
    rows = []
    for start in range(0, len(padded), columns):
        row = padded[start:start + columns]
        row += [blank] * (columns - len(row))
        rows.append(np.hstack(row))
    cv2.imwrite(str(output_path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])


def main():
    args = parse_args()
    if not args.input.is_dir():
        raise FileNotFoundError(f'Input directory does not exist: {args.input}')
    if not args.model.is_file():
        raise FileNotFoundError(f'Model does not exist: {args.model}')
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f'Output directory is not empty: {args.output}')
    if not 0.0 <= args.confidence <= 1.0 or not 0.0 <= args.iou <= 1.0:
        raise ValueError('confidence and iou must be within [0, 1]')
    if args.imgsz < 32 or args.limit < 0:
        raise ValueError('imgsz must be >= 32 and limit must be >= 0')

    images = image_paths(args.input, args.limit)
    if not images:
        raise ValueError(f'No JPEG/PNG files found in {args.input}')
    args.output.mkdir(parents=True, exist_ok=True)
    json_dir = args.output / 'json'
    annotated_dir = args.output / 'annotated'
    json_dir.mkdir()
    annotated_dir.mkdir()

    model = YOLO(str(args.model))
    names = class_names(model.names)
    model_device = next(model.model.parameters()).device
    if model_device.type != 'cpu':
        raise RuntimeError(f'Model is not on CPU: {model_device}')
    manifest = {
        'input_directory': str(args.input.resolve()),
        'model': str(args.model.resolve()),
        'model_sha256': model_sha256(args.model),
        'device_requested': args.device,
        'model_device_verified': str(model_device),
        'torch_cuda_available': torch.cuda.is_available(),
        'python': sys.version,
        'platform': platform.platform(),
        'torch': torch.__version__,
        'ultralytics': ultralytics.__version__,
        'opencv': cv2.__version__,
        'confidence_threshold': args.confidence,
        'iou_threshold': args.iou,
        'image_size': args.imgsz,
        'class_names': names,
        'image_count': len(images),
        'limitations': [
            'RGB-only replay does not validate depth, TF, base/map coordinates, or grasp accuracy.',
            'CPU replay is functional evidence and not Jetson latency evidence.',
        ],
    }
    (args.output / 'replay_manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )

    annotated_paths = []
    review_rows = []
    with (args.output / 'summary.csv').open('w', newline='', encoding='utf-8') as summary_file:
        writer = csv.DictWriter(
            summary_file,
            fieldnames=['image', 'width_px', 'height_px', 'detection_count', 'classes', 'inference_ms'],
        )
        writer.writeheader()
        for image_path in images:
            started = time.perf_counter()
            result = model.predict(
                source=str(image_path), device='cpu', conf=args.confidence,
                iou=args.iou, imgsz=args.imgsz, verbose=False,
            )[0]
            if result.boxes is not None and result.boxes.data.device.type != 'cpu':
                raise RuntimeError(
                    f'Inference returned a non-CPU tensor: {result.boxes.data.device}'
                )
            inference_ms = (time.perf_counter() - started) * 1000.0
            detections = serialise_detections(result, names)
            source = cv2.imread(str(image_path))
            if source is None:
                raise OSError(f'Cannot read extracted image: {image_path}')
            annotation_path = annotated_dir / image_path.name
            if not cv2.imwrite(str(annotation_path), annotate(source, detections)):
                raise OSError(f'Cannot write annotation: {annotation_path}')
            annotated_paths.append(annotation_path)
            frame_payload = {
                'image': image_path.name,
                'width_px': int(source.shape[1]),
                'height_px': int(source.shape[0]),
                'inference_ms': round(inference_ms, 3),
                'detections': detections,
            }
            (json_dir / f'{image_path.stem}.json').write_text(
                json.dumps(frame_payload, indent=2, ensure_ascii=False) + '\n',
                encoding='utf-8',
            )
            writer.writerow({
                'image': image_path.name,
                'width_px': source.shape[1],
                'height_px': source.shape[0],
                'detection_count': len(detections),
                'classes': '|'.join(detection['class_name'] for detection in detections),
                'inference_ms': f'{inference_ms:.3f}',
            })
            for detection in detections:
                review_rows.append({
                    'image': image_path.name,
                    'predicted_class': detection['class_name'],
                    'confidence': detection['confidence'],
                    'manual_ground_truth': '',
                    'mask_reasonable': '',
                    'bbox_center_reasonable': '',
                    'outcome_tp_fp_fn': '',
                    'notes': '',
                })

    with (args.output / 'manual_review.csv').open('w', newline='', encoding='utf-8') as review_file:
        writer = csv.DictWriter(
            review_file,
            fieldnames=[
                'image', 'predicted_class', 'confidence', 'manual_ground_truth',
                'mask_reasonable', 'bbox_center_reasonable', 'outcome_tp_fp_fn', 'notes',
            ],
        )
        writer.writeheader()
        writer.writerows(review_rows)
    write_contact_sheet(annotated_paths, args.output / 'contact_sheet.jpg')
    print(f'CPU replay completed: {len(images)} images -> {args.output}')


if __name__ == '__main__':
    main()
