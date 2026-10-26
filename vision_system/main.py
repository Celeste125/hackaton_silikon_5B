import cv2
import numpy as np
import time

COLORS = {'pea': (0, 255, 0), 'rapeseed': (255, 0, 0), 'wheat': (0, 0, 255)}


def crop(img):
    h, w = img.shape[:2]
    img = img[200: h - 200, 200: w - 450]
    roi = np.zeros(img.shape[:2], np.uint8)
    params = dict(radius=350, color=255, thickness=-1)
    cv2.circle(roi, center=(350, 350), **params)
    cv2.circle(roi, center=(1000, 350), **params)
    return cv2.bitwise_and(img, img, mask=roi), roi


def background_filter(img, roi):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerb=(0, 0, 50), upperb=(180, 70, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize=(7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.bitwise_and(mask, roi)


def pea_filter(img, roi):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray,
        cv2.HOUGH_GRADIENT, dp=1, minDist=5, param1=20, param2=20, minRadius=5, maxRadius=19
    )
    mask = np.zeros(img.shape[:2], np.uint8)
    if circles is not None:
        for x, y, radius in np.round(circles[0]).astype(int):
            cv2.circle(mask, center=(x, y), radius=radius, color=255, thickness=-1)
    return cv2.bitwise_and(mask, roi)


def rapeseed_filter(img, roi):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh=60, maxval=255, type=cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize=(3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.bitwise_and(mask, roi)


def wheat_filter(background, pea, rapeseed, roi):
    not_wheat = cv2.bitwise_or(cv2.bitwise_or(pea, rapeseed), background)
    return cv2.bitwise_and(cv2.bitwise_not(not_wheat), roi)


def measure_area(**masks):
    areas = {grain_type: cv2.countNonZero(mask) for grain_type, mask in masks.items()}
    total_area = max(sum(areas.values()), 1)
    return {grain_type: 100 * area / total_area for grain_type, area in areas.items()}


def measure_homogeneity(roi, cells_inline=50, **masks):
    h, w = roi.shape
    size = (w // cells_inline, h // cells_inline)
    cells = np.stack([
        cv2.resize(mask.astype(np.float32) / 255, size, interpolation=cv2.INTER_AREA)
        for mask in masks.values()
    ])
    total = cells.sum(axis=0)
    valid = total > 0.5
    weights = total[valid]
    cell_content = cells[:, valid] / weights
    mean = (cell_content * weights).sum(axis=1) / weights.sum()
    variance = (weights * (cell_content - mean[:, None]) ** 2).sum(axis=1) / weights.sum()
    segregated = (mean * (1 - mean)).sum()
    return 1 - variance.sum() / max(segregated, 1e-9)


def apply_overlay(img, pea, rapeseed, wheat, alpha=0.3):
    overlay = img.copy()
    overlay[pea > 0] = COLORS['pea']
    overlay[rapeseed > 0] = COLORS['rapeseed']
    overlay[wheat > 0] = COLORS['wheat']
    return cv2.addWeighted(overlay, alpha, img, 1 - alpha, gamma=0)


def add_legend(img, fps, areas, mix_idx):
    params = dict(
        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(255, 255, 255), thickness=1
    )
    cv2.putText(img, text=f'FPS: {fps:.1f}', org=(10, 20), **params)
    cv2.putText(img, text=f'homogeneity: {mix_idx:.2f}/1', org=(10, 40), **params)
    for i, (grain_type, area) in enumerate(areas.items()):
        y = 60 + 20 * i
        cv2.rectangle(img, (10, y - 14), (24, y), color=COLORS[grain_type], thickness=-1)
        cv2.putText(img, text=f'{grain_type}: {area:.0f}%', org=(32, y), **params)


def main():
    cap  = cv2.VideoCapture('videos/clip_hd.mp4')
    _, frame = cap.read()
    frame, _ = crop(frame)
    h, w = frame.shape[:2]
    writer = cv2.VideoWriter('results/mixing.mp4',
        fourcc=cv2.VideoWriter_fourcc(*'mp4v'), fps=12, frameSize=(w, h)
    )
    start = time.perf_counter()
    while cap.isOpened():
        _, frame = cap.read()
        frame, roi = crop(frame)
        background = background_filter(frame, roi)

        pea = pea_filter(frame, roi=cv2.bitwise_and(roi, cv2.bitwise_not(background)))
        rapeseed = rapeseed_filter(frame, roi)
        wheat = wheat_filter(background, pea, rapeseed, roi=cv2.bitwise_and(roi, cv2.bitwise_not(background)))

        masks = dict(pea=pea, rapeseed=rapeseed, wheat=wheat)
        areas = measure_area(**masks)
        mix_idx  = measure_homogeneity(roi, **masks)

        output_frame = apply_overlay(frame, pea, rapeseed, wheat)

        end = time.perf_counter()
        fps = 1 / max(end - start, 1e-6)
        start = end
        add_legend(output_frame, fps, areas, mix_idx)
        writer.write(output_frame)
        cv2.imshow('', output_frame)
        if cv2.waitKey(50) & 0xFF == 27:
            break

    cap.release()
    writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
