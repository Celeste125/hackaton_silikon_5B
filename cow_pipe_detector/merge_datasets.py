"""Merge the cow dataset and the Roboflow pipe dataset into one YOLO dataset.

Classes in the output:  0 cow_head   1 cow_body   2 pipe

Works with any of these as input (zip or folder):
  - cow_dataset.zip            (images/train, labels/train, ...)
  - a CVAT "Ultralytics YOLO" export of the cows (same layout)
  - the Roboflow pipe export   (train/images, train/labels, valid/..., test/...)

Usage:
  python merge_datasets.py --cows cow_dataset.zip --pipes Pipe_Detection.v1i.yolov8.zip --out combined_dataset
"""
import argparse, os, shutil, tempfile, zipfile, re
from pathlib import Path

NAMES = ['cow_head', 'cow_body', 'pipe']
IMG_EXT = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
SPLIT_ALIASES = {'train': 'train', 'val': 'val', 'valid': 'val', 'validation': 'val', 'test': 'test'}


def unpack(src: Path) -> Path:
    if src.is_dir():
        return src
    tmp = Path(tempfile.mkdtemp())
    with zipfile.ZipFile(src) as z:
        z.extractall(tmp)
    return tmp


def find_pairs(root: Path):
    """Yield (split, image_path, label_path_or_None) for every image under root."""
    for img in root.rglob('*'):
        if img.suffix.lower() not in IMG_EXT or 'previews' in img.parts:
            continue
        parts = list(img.parts)
        if 'images' not in parts:
            continue
        i = len(parts) - 1 - parts[::-1].index('images')
        split = next((SPLIT_ALIASES[p.lower()] for p in parts if p.lower() in SPLIT_ALIASES), 'train')
        lab_parts = parts[:i] + ['labels'] + parts[i + 1:]
        lab = Path(*lab_parts).with_suffix('.txt')
        yield split, img, (lab if lab.exists() else None)


def to_box(values):
    """A YOLO line's coordinates -> (cx, cy, w, h). Polygons become their bounding box."""
    if len(values) == 4:
        return values
    xs, ys = values[0::2], values[1::2]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    return [(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0]


def add(root, class_map, prefix, out, stats):
    for split, img, lab in find_pairs(root):
        name = prefix + re.sub(r'[^\w.-]', '_', img.stem)
        (out / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / split).mkdir(parents=True, exist_ok=True)
        shutil.copy(img, out / 'images' / split / (name + img.suffix.lower()))
        lines = []
        if lab:
            for raw in lab.read_text().split('\n'):
                p = raw.split()
                if len(p) < 5:
                    continue
                cls = class_map[int(p[0])]
                cx, cy, w, h = (min(max(v, 0.0), 1.0) for v in to_box([float(v) for v in p[1:]]))
                if w <= 0 or h <= 0:
                    continue
                lines.append(f'{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}')
                stats[(split, NAMES[cls])] = stats.get((split, NAMES[cls]), 0) + 1
        (out / 'labels' / split / (name + '.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''))
        stats[(split, 'images')] = stats.get((split, 'images'), 0) + 1
        if not lines:
            stats[(split, 'empty')] = stats.get((split, 'empty'), 0) + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cows', required=True)
    ap.add_argument('--pipes', required=True)
    ap.add_argument('--out', default='combined_dataset')
    a = ap.parse_args()
    out = Path(a.out)
    shutil.rmtree(out, ignore_errors=True)
    stats = {}
    add(unpack(Path(a.cows)), {0: 0, 1: 1}, 'cow_', out, stats)    # cow_head, cow_body
    add(unpack(Path(a.pipes)), {0: 2}, 'pipe_', out, stats)        # pipe -> 2
    splits = sorted({s for s, _ in stats})
    yaml = [f'path: {out.resolve().as_posix()}'] + [f'{s}: images/{s}' for s in splits]
    yaml += ['', 'names:'] + [f'  {i}: {n}' for i, n in enumerate(NAMES)]
    (out / 'data.yaml').write_text('\n'.join(yaml) + '\n')
    for s in splits:
        row = {k: v for (sp, k), v in stats.items() if sp == s}
        print(f'{s:5s}', row)
    print('wrote', out / 'data.yaml')


if __name__ == '__main__':
    main()
