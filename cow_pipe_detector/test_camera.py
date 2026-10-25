"""Live test of the cow + pipe model on a camera (or a video file).

Run from anywhere; by default it loads the model shipped in the repo
(ros2_ws/src/perception/models/cows_pipes_yolov8.pt).

    python test_camera.py --list                 # which camera numbers exist?
    python test_camera.py                        # camera 0 (usually the laptop webcam)
    python test_camera.py --camera 1             # an external USB camera / C920
    python test_camera.py --source clip.mp4      # a video file instead
    python test_camera.py --record               # also save the annotated video

Keys in the window:
    q / Esc   quit
    s         save a snapshot (annotated + raw) into snapshots/
    + / -     raise / lower the confidence threshold by 0.05
    space     pause / resume
"""
import argparse, sys, time
from collections import Counter
from pathlib import Path

import cv2
from ultralytics import YOLO

DEFAULT_WEIGHTS = (Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'perception'
                   / 'models' / 'cows_pipes_yolov8.pt')
COLORS = {'cow_head': (0, 0, 255), 'cow_body': (0, 200, 0), 'pipe': (255, 0, 255)}


def open_capture(index):
    # On Windows, DirectShow opens USB cameras much faster than the default backend.
    if sys.platform.startswith('win'):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            return cap
    return cv2.VideoCapture(index)


def list_cameras(max_index=6):
    found = []
    for i in range(max_index):
        cap = open_capture(i)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        if ok:
            found.append(i)
            print(f'camera {i}: {frame.shape[1]}x{frame.shape[0]}')
        cap.release()
    if not found:
        print('no camera found - is it plugged in / used by another app (Teams, Zoom, browser)?')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default=str(DEFAULT_WEIGHTS))
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--source', help='video file instead of a camera')
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--device', default=None, help="'0' for GPU, 'cpu' to force CPU")
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        list_cameras()
        return

    model = YOLO(a.weights)
    names = model.names
    print('model classes:', names)

    cap = cv2.VideoCapture(a.source) if a.source else open_capture(a.camera)
    if not a.source:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        sys.exit(f'could not open {a.source or "camera " + str(a.camera)} - try --list')

    snaps = Path(__file__).resolve().parent / 'snapshots'; snaps.mkdir(exist_ok=True)
    writer, conf, paused, fps, last = None, a.conf, False, 0.0, time.time()
    frame = shown = None

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                break
            r = model.predict(frame, conf=conf, imgsz=a.imgsz, device=a.device, verbose=False)[0]
            shown = frame.copy()
            counts = Counter()
            for box, cls, score in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
                name = names[int(cls)]
                counts[name] += 1
                x0, y0, x1, y1 = map(int, box)
                c = COLORS.get(name, (255, 255, 0))
                cv2.rectangle(shown, (x0, y0), (x1, y1), c, 2)
                cv2.putText(shown, f'{name} {score:.2f}', (x0, max(18, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

            now = time.time(); fps = 0.9 * fps + 0.1 / max(now - last, 1e-6); last = now
            cow = counts['cow_head'] + counts['cow_body'] > 0
            status = (f'COW: {"YES" if cow else "no"}   pipes: {counts["pipe"]}   '
                      f'head {counts["cow_head"]}  body {counts["cow_body"]}   '
                      f'conf {conf:.2f}   {fps:.1f} fps')
            cv2.rectangle(shown, (0, 0), (shown.shape[1], 32), (0, 0, 0), -1)
            cv2.putText(shown, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if cow else (255, 255, 255), 2)

            if a.record and writer is None:
                h, w = shown.shape[:2]
                writer = cv2.VideoWriter(str(snaps / f'recording_{int(time.time())}.mp4'),
                                         cv2.VideoWriter_fourcc(*'mp4v'), 15, (w, h))
            if writer is not None:
                writer.write(shown)

        cv2.imshow('cow + pipe detector', shown)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            stamp = time.strftime('%Y%m%d_%H%M%S')
            cv2.imwrite(str(snaps / f'{stamp}_boxes.jpg'), shown)
            cv2.imwrite(str(snaps / f'{stamp}_raw.jpg'), frame)  # raw = new training data
            print('saved', stamp)
        if key in (ord('+'), ord('=')):
            conf = min(0.95, conf + 0.05)
        if key in (ord('-'), ord('_')):
            conf = max(0.05, conf - 0.05)
        if key == ord(' '):
            paused = not paused

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
