"""Grain detection and mixing analysis. Pure OpenCV/NumPy, no ROS dependency.

Everything runs at `scale` x the cropped frame (0.5 = 4x fewer pixels). All
size-dependent values follow `scale`; scale = 1.0 gives full-resolution behaviour.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

COLORS = {'pea': (0, 255, 0), 'rapeseed': (255, 0, 0), 'wheat': (0, 0, 255)}
GRAIN_TYPES = tuple(COLORS)


def _odd(n):
    return max(3, round(n) | 1)


def _ellipse(n):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))


@dataclass
class PipelineParams:
    scale: float = 0.5
    # crop window: pixels removed from each side of the input frame
    crop_top: int = 200
    crop_bottom: int = 200
    crop_left: int = 200
    crop_right: int = 450
    # circular mixer views: x1, y1, x2, y2 in the cropped frame, at scale 1.0
    circle_centers: list = field(default_factory=lambda: [350, 350, 1000, 350])
    circle_radius: int = 350
    # machine parts (HSV, OpenCV scale)
    bg_lower: list = field(default_factory=lambda: [0, 0, 50])
    bg_upper: list = field(default_factory=lambda: [180, 70, 255])
    # peas (values at scale 1.0; param2 is reduced automatically for smaller scales)
    hough_param1: float = 20.0
    hough_param2: float = 20.0
    pea_min_radius: int = 5
    pea_max_radius: int = 19
    # rapeseed
    rapeseed_threshold: int = 60
    # homogeneity grid cell size, at scale 1.0
    cell_size: int = 100
    # visualisation
    overlay_alpha: float = 0.3


@dataclass
class PipelineResult:
    raw: np.ndarray          # cropped, scaled, masked input frame
    overlay: np.ndarray      # same frame with colour overlays
    total_area: float        # fraction of roi covered by grains, 0-1
    areas: dict              # share of each grain type, percent
    homogeneity: float       # 1 = uniform mix, 0 = fully segregated


class GrainPipeline:
    def __init__(self, params=None):
        self.p = params or PipelineParams()
        s = self.p.scale
        self.kernel_bg = _ellipse(_odd(7 * s))
        self.kernel_rapeseed = _ellipse(3)
        self.hough = dict(
            dp=1,
            minDist=max(1, round(5 * s)),
            param1=float(self.p.hough_param1),
            param2=float(self.p.hough_param2) * s ** 0.5,   # fewer votes on smaller circles
            minRadius=max(1, round(int(self.p.pea_min_radius) * s)),
            maxRadius=max(2, round(int(self.p.pea_max_radius) * s)),
        )
        self.cell_size = max(2, round(int(self.p.cell_size) * s))
        self.roi = None            # built once, reused while the frame size stays the same
        self._solid = {}

    # ---- preprocessing ---------------------------------------------------
    def preprocess(self, img):
        """Crop, downscale and mask a frame. Returns (frame, roi)."""
        p, s = self.p, self.p.scale
        h, w = img.shape[:2]
        img = img[p.crop_top: h - p.crop_bottom, p.crop_left: w - p.crop_right]
        if img.size == 0:
            raise ValueError('crop window is larger than the input frame')
        if s != 1.0:
            img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        h, w = img.shape[0] // 2 * 2, img.shape[1] // 2 * 2      # even size for video encoders
        img = np.ascontiguousarray(img[:h, :w])
        if self.roi is None or self.roi.shape != (h, w):
            self.roi = np.zeros((h, w), np.uint8)
            centers = list(p.circle_centers)
            for cx, cy in zip(centers[0::2], centers[1::2]):
                cv2.circle(self.roi, (round(cx * s), round(cy * s)),
                           round(p.circle_radius * s), 255, thickness=-1)
        return cv2.bitwise_and(img, img, mask=self.roi), self.roi

    # ---- filters ---------------------------------------------------------
    def background_filter(self, img, roi):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = tuple(int(v) for v in self.p.bg_lower)
        upper = tuple(int(v) for v in self.p.bg_upper)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_bg)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_bg)
        return cv2.bitwise_and(mask, roi)

    def pea_filter(self, gray, region):
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, **self.hough)
        mask = np.zeros(gray.shape, np.uint8)
        if circles is not None:
            for x, y, radius in np.round(circles[0]).astype(int):
                cv2.circle(mask, (x, y), radius, 255, thickness=-1)
        return cv2.bitwise_and(mask, region)

    def rapeseed_filter(self, gray, roi):
        _, mask = cv2.threshold(gray, int(self.p.rapeseed_threshold), 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_rapeseed)
        return cv2.bitwise_and(mask, roi)

    @staticmethod
    def wheat_filter(region, pea, rapeseed):
        # region already excludes the machine, so wheat = region - pea - rapeseed
        return cv2.subtract(region, cv2.bitwise_or(pea, rapeseed))

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
        size = (w // self.cell_size, h // self.cell_size)
        if min(size) < 1:
            return float('nan')
        cells = np.stack([
            cv2.resize(mask, size, interpolation=cv2.INTER_AREA) for mask in masks.values()
        ]).astype(np.float32) / 255
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
    def _solid_image(self, color, shape):
        key = (color, shape)
        if key not in self._solid:
            self._solid[key] = np.full(shape, color, np.uint8)
        return self._solid[key]

    def apply_overlay(self, img, pea, rapeseed, wheat):
        overlay = img.copy()
        for mask, grain_type in ((pea, 'pea'), (rapeseed, 'rapeseed'), (wheat, 'wheat')):
            cv2.copyTo(self._solid_image(COLORS[grain_type], img.shape), mask, overlay)
        alpha = self.p.overlay_alpha
        return cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    # ---- full pipeline ---------------------------------------------------
    def process(self, bgr):
        frame, roi = self.preprocess(bgr)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)           # shared by two filters
        background = self.background_filter(frame, roi)
        region = cv2.subtract(roi, background)                   # grain area: inside circles, no machine

        pea = self.pea_filter(gray, region)
        rapeseed = self.rapeseed_filter(gray, roi)
        wheat = self.wheat_filter(region, pea, rapeseed)

        masks = dict(pea=pea, rapeseed=rapeseed, wheat=wheat)
        total_area = self.measure_total_area(roi, **masks)
        areas = self.measure_each_area(**masks)
        homogeneity = self.measure_homogeneity(roi, **masks)
        overlay = self.apply_overlay(frame, pea, rapeseed, wheat)
        return PipelineResult(frame, overlay, total_area, areas, homogeneity)


def add_legend(img, fps, total_area, areas, homogeneity):
    """Draws the legend in place."""
    params = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6,
                  color=(255, 255, 255), thickness=1)
    cv2.putText(img, f'FPS: {fps:.1f}', (10, 20), **params)
    cv2.putText(img, f'coverage: {total_area:.2f}/1', (10, 40), **params)
    cv2.putText(img, f'homogeneity: {homogeneity:.2f}/1', (10, 60), **params)
    for i, (grain_type, area) in enumerate(areas.items()):
        y = 80 + 20 * i
        cv2.rectangle(img, (10, y - 14), (24, y), COLORS[grain_type], thickness=-1)
        cv2.putText(img, f'{grain_type}: {area:.0f}%', (32, y), **params)
