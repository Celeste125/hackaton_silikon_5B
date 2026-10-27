# Grain Mixing Analyzer

Computer vision system for analyzing a video of a grain mixer filmed from above. For every frame the program:

1. detects three grain types: **peas**, **rapeseed** and **wheat**,
2. measures the share of the visible grain area covered by each type,
3. computes a **homogeneity index** that says how well the grains are mixed,
4. draws semi-transparent color overlays and a legend on the frame and saves the result as a video.

| Grain | Overlay colour | Detected by |
|---|---|---|
| Pea | green | Hough circle transform (round, pale grains) |
| Rapeseed | blue | dark-pixel threshold (small, dark grains) |
| Wheat | red | everything that is left over in the grain area |

The gray machine parts (the discs) are detected separately and excluded from all grain masks.

## Requirements

- Python 3.8+
- `opencv-python`
- `numpy`

```bash
pip install opencv-python numpy
```

## Usage

1. Put the input video at `videos/clip_hd.mp4`.
2. Create the output folder: `mkdir results`. OpenCV does not create it, and `VideoWriter` fails silently if it is missing.
3. Run:

```bash
python main.py
```

A preview window shows the processed frames. Press **Esc** to stop early. The annotated video is written to `results/mixing.mp4`.

## How it works

### Pipeline

```
frame ──► crop() ──► frame + roi
                        │
          background_filter ──► machine parts
                        │
          pea_filter ───┤        (Hough circles, restricted to grain area)
          rapeseed_filter ──┤    (dark pixels)
          wheat_filter ─┘        (leftover: roi − peas − rapeseed − machine)
                        │
          measure_area ─────► % of grain area per type
          measure_homogeneity ► mixing index (0–1)
                        │
          apply_overlay + add_legend ──► output frame ──► video / window
```

### Functions

| Function | What it does |
|---|---|
| `crop(img)` | Cuts the frame to the working area and builds `roi`, a mask of the two circular mixer views (radius 350 px). Everything outside the circles is blacked out. |
| `background_filter(img, roi)` | Finds the grey machine parts with an HSV range (low saturation, not too dark), then closes and opens the mask with a 7×7 elliptical kernel. |
| `pea_filter(img, roi)` | Runs `cv2.HoughCircles` (radius 5–19 px) on the grayscale image and draws the circles filled. Called with the grain area (`roi` minus machine), so circles are clipped at the machine boundary. |
| `rapeseed_filter(img, roi)` | Thresholds dark pixels (grey value < 60) and removes specks with a 3×3 opening. |
| `wheat_filter(...)` | Wheat is whatever is inside `roi` and is neither pea, rapeseed nor machine. |
| `measure_area(**masks)` | Pixel count of each mask as a percentage of the summed pixel count. |
| `measure_homogeneity(roi, cells_inline, **masks)` | Mixing index, described below. |
| `apply_overlay(...)` | Blends the colour masks over the frame (`alpha=0.3`). |
| `add_legend(...)` | Draws FPS, homogeneity and the area shares with colour swatches. |

### Homogeneity index

The frame is divided into a grid of square cells (`cells_inline` = 50 px by default). For each cell with enough grain in it, the program computes the share of each grain type, then compares how much those shares vary between cells with the variance of a completely segregated mixture:

```
homogeneity = 1 − (variance of cell compositions) / (variance if fully segregated)
```

- **1** means every cell has the same composition as the whole, so the mix is uniform.
- **0** means every cell contains a single grain type, so the grains are fully segregated.

Cells are weighted by how much grain they contain, and cells that are mostly machine or outside the circles are skipped. The index is based on area, not grain count, so large grains such as peas count for more than small ones.

A perfectly random mixture scores below 1, because small cells hold only a few grains and differ by chance. How far below depends on the cell size, so only compare values computed with the same `cells_inline`.
