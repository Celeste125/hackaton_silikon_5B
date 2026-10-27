"""Grain detection and mixing analysis. Pure OpenCV/NumPy, no ROS dependency."""
from dataclasses import dataclass, field

import cv2
import numpy as np

COLORS = {'pea': (0, 255, 0), 'rapeseed': (255, 0, 0), 'wheat': (0, 0, 255)}
GRAIN_TYPES = tuple(COLORS)


@dataclass
class PipelineParams:
    # crop window: pixels removed from each side of the input frame
    crop_top: int = 200
    crop_bottom: int = 200
    crop_left: int = 200
    crop_right: int = 450
    # circular mixer views: x1, y1, x2, y2 in the cropped frame
    circle_centers: list = field(default_factory=lambda: [350, 350, 1000, 350])
    circle_radius: int = 350
    # machine parts (HSV)
    bg_lower: list = field(default_factory=lambda: [0, 0, 50])
    bg_upper: list = field(default_factory=lambda: [180, 70, 255])
    # peas
    hough_param1: float = 15.0
    hough_param2: float = 15.0
    pea_min_radius: int = 5
    pea_max_radius: int = 18
    pea_max_gradient: float = 10.0
    # rapeseed
    rapeseed_threshold: int = 60
    # homogeneity grid
    cell_size: int = 50
    # visualisation
    overlay_alpha: float = 0.3
    output_scale: float = 0.5


@dataclass
class PipelineResult:
    raw: np.ndarray          # cropped input frame (scaled by output_scale)
    overlay: np.ndarray      # frame with colour overlays (scaled by output_scale)
    total_area: float        # fraction of roi covered by grains, 0-1
    areas: dict              # share of each grain type, percent
    homogeneity: float       # 1 = uniform mix, 0 = fully segregated


class GrainPipeline:
    def __init__(self, params=None):
        self.p = params or PipelineParams()
        self.kernel7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    # ---- filters ---------------------------------------------------------
    def crop(self, img):
        p = self.p
        h, w = img.shape[:2]
        img = img[p.crop_top: h - p.crop_bottom, p.crop_left: w - p.crop_right]
        if img.size == 0:
            raise ValueError('crop window is larger than the input frame')
        roi = np.zeros(img.shape[:2], np.uint8)
        centers = list(p.circle_centers)
        for cx, cy in zip(centers[0::2], centers[1::2]):
            cv2.circle(roi, (int(cx), int(cy)), int(p.circle_radius), 255, thickness=-1)
        return cv2.bitwise_and(img, img, mask=roi), roi

    def background_filter(self, img, roi):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = tuple(int(v) for v in self.p.bg_lower)
        upper = tuple(int(v) for v in self.p.bg_upper)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel7)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel7)
        return cv2.bitwise_and(mask, roi)

    def pea_filter(self, img, region):
        p = self.p
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1, minDist=5,
            param1=p.hough_param1, param2=p.hough_param2,
            minRadius=int(p.pea_min_radius), maxRadius=int(p.pea_max_radius),
        )
        mask = np.zeros(img.shape[:2], np.uint8)
        if circles is None:
            return cv2.bitwise_and(mask, region)

        # reject circles whose inside is not smooth (wheat has a groove):
        # more than 10% steep pixels in a box of the same area as the inner disk.
        # An integral image gives every box sum at once, with no per-circle loop.
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        gradient = np.hypot(cv2.Sobel(blur, cv2.CV_32F, 1, 0),
                            cv2.Sobel(blur, cv2.CV_32F, 0, 1)) / 8
        integral = cv2.integral((gradient > p.pea_max_gradient).astype(np.uint8))

        circles = np.round(circles[0]).astype(int)
        x, y, radius = circles.T
        half = np.maximum((radius * 0.7 * 0.886).astype(int), 1)
        h, w = gray.shape
        x0, x1 = np.clip(x - half, 0, w), np.clip(x + half + 1, 0, w)
        y0, y1 = np.clip(y - half, 0, h), np.clip(y + half + 1, 0, h)
        steep = integral[y1, x1] - integral[y0, x1] - integral[y1, x0] + integral[y0, x0]
        smooth = steep <= 0.1 * (x1 - x0) * (y1 - y0)
        for cx, cy, r in circles[smooth]:
            cv2.circle(mask, (int(cx), int(cy)), int(r), 255, thickness=-1)
        return cv2.bitwise_and(mask, region)

    def rapeseed_filter(self, img, roi):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, int(self.p.rapeseed_threshold), 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel3)
        return cv2.bitwise_and(mask, roi)

    @staticmethod
    def wheat_filter(background, pea, rapeseed, region):
        not_wheat = cv2.bitwise_or(cv2.bitwise_or(pea, rapeseed), background)
        return cv2.bitwise_and(cv2.bitwise_not(not_wheat), region)

    # ---- measurements ----------------------------------------------------
    @staticmethod
    def measure_total_area(roi, **masks):
        grains = np.zeros_like(roi)
        for mask in masks.values():
            grains = cv2.bitwise_or(grains, mask)
        return cv2.countNonZero(grains) / max(cv2.countNonZero(roi), 1)

    @staticmethod
    def measure_each_area(**masks):
        areas = {name: cv2.countNonZero(mask) for name, mask in masks.items()}
        total = max(sum(areas.values()), 1)
        return {name: 100 * area / total for name, area in areas.items()}

    def measure_homogeneity(self, roi, **masks):
        h, w = roi.shape
        cell = max(int(self.p.cell_size), 1)
        size = (w // cell, h // cell)
        if min(size) < 1:
            return float('nan')
        cells = np.stack([
            cv2.resize(mask.astype(np.float32) / 255, size, interpolation=cv2.INTER_AREA)
            for mask in masks.values()
        ])
        total = cells.sum(axis=0)
        valid = total > 0.5
        if not valid.any():
            return float('nan')
        weights = total[valid]
        cell_content = cells[:, valid] / weights
        mean = (cell_content * weights).sum(axis=1) / weights.sum()
        variance = (weights * (cell_content - mean[:, None]) ** 2).sum(axis=1) / weights.sum()
        segregated = (mean * (1 - mean)).sum()
        return float(1 - variance.sum() / max(segregated, 1e-9))

    # ---- visualisation ---------------------------------------------------
    def apply_overlay(self, img, pea, rapeseed, wheat):
        overlay = img.copy()
        overlay[pea > 0] = COLORS['pea']
        overlay[rapeseed > 0] = COLORS['rapeseed']
        overlay[wheat > 0] = COLORS['wheat']
        alpha = self.p.overlay_alpha
        return cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    # ---- full pipeline ---------------------------------------------------
    def process(self, bgr):
        frame, roi = self.crop(bgr)
        background = self.background_filter(frame, roi)
        grain_region = cv2.bitwise_and(roi, cv2.bitwise_not(background))

        pea = self.pea_filter(frame, grain_region)
        rapeseed = self.rapeseed_filter(frame, roi)
        wheat = self.wheat_filter(background, pea, rapeseed, grain_region)

        masks = dict(pea=pea, rapeseed=rapeseed, wheat=wheat)
        total_area = self.measure_total_area(roi, **masks)
        areas = self.measure_each_area(**masks)
        homogeneity = self.measure_homogeneity(roi, **masks)

        overlay = self.apply_overlay(frame, pea, rapeseed, wheat)
        s = self.p.output_scale
        if s != 1.0:
            kw = dict(dsize=(0, 0), fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
            frame, overlay = cv2.resize(frame, **kw), cv2.resize(overlay, **kw)
        return PipelineResult(frame, overlay, total_area, areas, homogeneity)


def add_legend(img, fps, total_area, areas, homogeneity):
    params = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6,
                  color=(255, 255, 255), thickness=1)
    cv2.putText(img, f'FPS: {fps:.1f}', (10, 20), **params)
    cv2.putText(img, f'coverage: {total_area:.2f}/1', (10, 40), **params)
    cv2.putText(img, f'homogeneity: {homogeneity:.2f}/1', (10, 60), **params)
    for i, (grain_type, area) in enumerate(areas.items()):
        y = 80 + 20 * i
        cv2.rectangle(img, (10, y - 14), (24, y), COLORS[grain_type], thickness=-1)
        cv2.putText(img, f'{grain_type}: {area:.0f}%', (32, y), **params)
