#!/usr/bin/env python3
"""Interactive OpenCV reviewer for CPU YOLO replay evidence."""

import argparse
import csv
import json
import os
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {
    0: 'red_block',
    1: 'yellow_block',
    2: 'green_block',
    3: 'blue_block',
    4: 'blue_obstacle',
}
REVIEW_FIELDS = [
    'image', 'predicted_class', 'confidence', 'manual_ground_truth',
    'mask_reasonable', 'bbox_center_reasonable', 'outcome_tp_fp_fn', 'notes',
]
FRAME_FIELDS = ['image', 'detection_count', 'frame_reviewed', 'notes']
FALSE_NEGATIVE_FIELDS = [
    'image', 'actual_class', 'reason', 'source_predicted_class', 'notes',
]
COLOURS = {
    0: (0, 0, 255),
    1: (0, 255, 255),
    2: (0, 180, 0),
    3: (255, 0, 0),
    4: (255, 180, 0),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Review one replay result directory through an OpenCV GUI.'
    )
    parser.add_argument('--images', required=True, type=Path)
    parser.add_argument('--results', required=True, type=Path)
    parser.add_argument(
        '--mode', choices=('detections', 'frames'), default='detections',
        help='detections reviews each prediction; frames covers empty images and FNs.',
    )
    parser.add_argument('--start-unreviewed', action='store_true')
    return parser.parse_args()


def read_csv(path, fields):
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if rows and list(rows[0]) != fields:
        raise ValueError(f'Unexpected CSV columns in {path}: {list(rows[0])}')
    return rows


def atomic_write_csv(path, rows, fields):
    temporary = path.with_name(f'.{path.name}.tmp')
    with temporary.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def load_frames(results_dir):
    frames = []
    for path in sorted((results_dir / 'json').glob('*.json')):
        payload = json.loads(path.read_text(encoding='utf-8'))
        required = {'image', 'width_px', 'height_px', 'detections'}
        if not required <= set(payload):
            raise ValueError(f'Invalid replay JSON: {path}')
        frames.append(payload)
    if not frames:
        raise ValueError(f'No per-frame JSON files found in {results_dir / "json"}')
    return frames


def attach_detection_reviews(frames, review_rows):
    rows_by_image = defaultdict(deque)
    for row in review_rows:
        rows_by_image[row['image']].append(row)
    records = []
    for frame in frames:
        rows = rows_by_image[frame['image']]
        if len(rows) != len(frame['detections']):
            raise ValueError(
                f"Review row count mismatch for {frame['image']}: "
                f'{len(rows)} rows, {len(frame["detections"])} predictions'
            )
        for detection in frame['detections']:
            row = rows.popleft()
            if row['predicted_class'] != detection['class_name']:
                raise ValueError(
                    f"Review class mismatch for {frame['image']}: "
                    f"{row['predicted_class']} != {detection['class_name']}"
                )
            records.append({'frame': frame, 'detection': detection, 'row': row})
    return records


def colour_for(detection):
    return COLOURS[int(detection['class_id'])]


def draw_detection(image, detection, colour, thickness=2, fill=False):
    polygon = np.asarray(detection['mask_polygon_px'], dtype=np.int32)
    if len(polygon) >= 3 and fill:
        overlay = image.copy()
        cv2.fillPoly(overlay, [polygon], colour)
        image[:] = cv2.addWeighted(overlay, 0.28, image, 0.72, 0.0)
    if len(polygon) >= 3:
        cv2.polylines(image, [polygon], True, colour, thickness, cv2.LINE_AA)
    x1, y1, x2, y2 = (int(round(value)) for value in detection['bbox_xyxy_px'])
    cv2.rectangle(image, (x1, y1), (x2, y2), colour, thickness)
    center = detection['center_px']
    cv2.drawMarker(
        image, (int(round(center['u'])), int(round(center['v']))), colour,
        cv2.MARKER_CROSS, 12, thickness,
    )


def draw_all_predictions(image, detections, selected_index=None):
    for index, detection in enumerate(detections):
        draw_detection(image, detection, colour_for(detection), 1, fill=True)
        if index == selected_index:
            draw_detection(image, detection, (255, 255, 255), 3, fill=False)
        x1, y1, _, _ = (int(round(value)) for value in detection['bbox_xyxy_px'])
        label = f"{detection['class_name']} {detection['confidence']:.2f}"
        cv2.putText(
            image, label, (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, colour_for(detection), 1, cv2.LINE_AA,
        )


def fit_for_display(image, max_width=1500, max_height=780):
    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale == 1.0:
        return image
    return cv2.resize(image, (int(width * scale), int(height * scale)))


def add_status_panel(image, lines):
    panel_height = 128
    canvas = cv2.copyMakeBorder(
        image, 0, panel_height, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25)
    )
    for line_number, line in enumerate(lines):
        cv2.putText(
            canvas, line, (12, image.shape[0] + 24 + 20 * line_number),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA,
        )
    return canvas


def image_for_frame(images_dir, frame):
    path = images_dir / frame['image']
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f'Cannot load source image: {path}')
    return image


def is_reviewed(row):
    return bool(row['outcome_tp_fp_fn'])


def append_false_negative(rows, image, class_name, reason, source_class=''):
    rows.append({
        'image': image,
        'actual_class': class_name,
        'reason': reason,
        'source_predicted_class': source_class,
        'notes': '',
    })


def next_index(records, current, predicate):
    for offset in range(1, len(records) + 1):
        candidate = (current + offset) % len(records)
        if predicate(records[candidate]):
            return candidate
    return current


def detection_mode(images_dir, results_dir, frames, review_rows, false_negative_rows, start_unreviewed):
    records = attach_detection_reviews(frames, review_rows)
    index = 0
    if start_unreviewed:
        index = next_index(records, -1, lambda record: not is_reviewed(record['row']))
    selected_truth = None
    window = 'IMDL YOLO detection review'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    while True:
        record = records[index]
        frame = record['frame']
        detection = record['detection']
        row = record['row']
        original = image_for_frame(images_dir, frame)
        left = original.copy()
        right = original.copy()
        selected_index = frame['detections'].index(detection)
        draw_all_predictions(right, frame['detections'], selected_index)
        paired = np.hstack((left, right))
        truth = selected_truth or row['manual_ground_truth'] or 'not selected'
        status = row['outcome_tp_fp_fn'] or 'unreviewed'
        lines = [
            f"{index + 1}/{len(records)}  image={frame['image']}  prediction={detection['class_name']} {detection['confidence']:.2f}",
            f"current: truth={truth}, mask={row['mask_reasonable'] or '-'}, center={row['bbox_center_reasonable'] or '-'}, outcome={status}",
            'A accept TP | M mask bad | C center bad | X false positive | 0-4 select actual class',
            'W wrong class (adds FN for selected class) | F missed object (adds FN) | ,/. prev/next | U next unreviewed | Q quit',
        ]
        cv2.imshow(window, add_status_panel(fit_for_display(paired), lines))
        key = cv2.waitKeyEx(0)
        if key in (27, ord('q')):
            break
        if ord('0') <= key <= ord('4'):
            selected_truth = CLASS_NAMES[key - ord('0')]
            continue
        if key == ord(','):
            index = (index - 1) % len(records)
            continue
        if key == ord('.'):
            index = (index + 1) % len(records)
            continue
        if key == ord('u'):
            index = next_index(records, index, lambda item: not is_reviewed(item['row']))
            continue
        if key == ord('a'):
            row.update({
                'manual_ground_truth': detection['class_name'],
                'mask_reasonable': 'yes',
                'bbox_center_reasonable': 'yes',
                'outcome_tp_fp_fn': 'TP',
            })
        elif key == ord('m'):
            row.update({
                'manual_ground_truth': detection['class_name'],
                'mask_reasonable': 'no',
                'bbox_center_reasonable': 'yes',
                'outcome_tp_fp_fn': 'TP',
            })
        elif key == ord('c'):
            row.update({
                'manual_ground_truth': detection['class_name'],
                'mask_reasonable': 'yes',
                'bbox_center_reasonable': 'no',
                'outcome_tp_fp_fn': 'TP',
            })
        elif key == ord('x'):
            row.update({
                'manual_ground_truth': '',
                'mask_reasonable': 'no',
                'bbox_center_reasonable': 'no',
                'outcome_tp_fp_fn': 'FP',
            })
        elif key == ord('w'):
            if selected_truth is None or selected_truth == detection['class_name']:
                continue
            row.update({
                'manual_ground_truth': selected_truth,
                'mask_reasonable': 'yes',
                'bbox_center_reasonable': 'yes',
                'outcome_tp_fp_fn': 'FP',
            })
            append_false_negative(
                false_negative_rows, frame['image'], selected_truth, 'wrong_class',
                detection['class_name'],
            )
        elif key == ord('f'):
            if selected_truth is None:
                continue
            append_false_negative(
                false_negative_rows, frame['image'], selected_truth, 'missed_object'
            )
            atomic_write_csv(
                results_dir / 'manual_false_negatives.csv',
                false_negative_rows, FALSE_NEGATIVE_FIELDS,
            )
            continue
        else:
            continue
        atomic_write_csv(results_dir / 'manual_review.csv', review_rows, REVIEW_FIELDS)
        atomic_write_csv(
            results_dir / 'manual_false_negatives.csv',
            false_negative_rows, FALSE_NEGATIVE_FIELDS,
        )
        selected_truth = None
        index = (index + 1) % len(records)

    cv2.destroyAllWindows()


def frame_mode(images_dir, results_dir, frames, false_negative_rows):
    review_path = results_dir / 'frame_review.csv'
    existing = {row['image']: row for row in read_csv(review_path, FRAME_FIELDS)}
    rows = [existing.get(frame['image'], {
        'image': frame['image'],
        'detection_count': str(len(frame['detections'])),
        'frame_reviewed': '',
        'notes': '',
    }) for frame in frames]
    index = 0
    selected_truth = None
    window = 'IMDL YOLO frame review'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    while True:
        frame = frames[index]
        original = image_for_frame(images_dir, frame)
        annotated = original.copy()
        draw_all_predictions(annotated, frame['detections'])
        paired = np.hstack((original, annotated))
        lines = [
            f"frame {index + 1}/{len(frames)}  image={frame['image']}  predictions={len(frame['detections'])}",
            f"frame reviewed={rows[index]['frame_reviewed'] or 'no'}  selected true class={selected_truth or 'not selected'}",
            'V mark frame reviewed | 0-4 select actual class | F add missed object/FN | ,/. prev/next | Q quit',
            'Use this mode for empty frames and any real object the detector missed.',
        ]
        cv2.imshow(window, add_status_panel(fit_for_display(paired), lines))
        key = cv2.waitKeyEx(0)
        if key in (27, ord('q')):
            break
        if ord('0') <= key <= ord('4'):
            selected_truth = CLASS_NAMES[key - ord('0')]
            continue
        if key == ord(','):
            index = (index - 1) % len(frames)
            continue
        if key == ord('.'):
            index = (index + 1) % len(frames)
            continue
        if key == ord('v'):
            rows[index]['frame_reviewed'] = 'yes'
            atomic_write_csv(review_path, rows, FRAME_FIELDS)
            index = (index + 1) % len(frames)
            continue
        if key == ord('f') and selected_truth is not None:
            append_false_negative(
                false_negative_rows, frame['image'], selected_truth, 'missed_object'
            )
            atomic_write_csv(
                results_dir / 'manual_false_negatives.csv',
                false_negative_rows, FALSE_NEGATIVE_FIELDS,
            )
            selected_truth = None

    cv2.destroyAllWindows()


def main():
    args = parse_args()
    images_dir = args.images.resolve()
    results_dir = args.results.resolve()
    if not images_dir.is_dir() or not results_dir.is_dir():
        raise FileNotFoundError('--images and --results must both be directories')
    frames = load_frames(results_dir)
    review_path = results_dir / 'manual_review.csv'
    review_rows = read_csv(review_path, REVIEW_FIELDS)
    if not review_rows:
        raise ValueError(f'No detection review rows found in {review_path}')
    false_negative_path = results_dir / 'manual_false_negatives.csv'
    false_negative_rows = read_csv(false_negative_path, FALSE_NEGATIVE_FIELDS)

    if args.mode == 'detections':
        detection_mode(
            images_dir, results_dir, frames, review_rows, false_negative_rows,
            args.start_unreviewed,
        )
    else:
        frame_mode(images_dir, results_dir, frames, false_negative_rows)


if __name__ == '__main__':
    main()
