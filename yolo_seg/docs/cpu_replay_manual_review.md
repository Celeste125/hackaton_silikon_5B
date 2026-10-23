# CPU Replay Manual Review

Run the reviewer separately for each bag after the CPU replay has produced its
`json/`, `manual_review.csv`, and annotated-image output.

```bash
conda run -n imdl python yolo_seg/scripts/review_yolo_seg_results.py \
  --images yolo_seg/experiment/replay_cpu/2026-07-17/formal/extracted/bag01 \
  --results yolo_seg/experiment/replay_cpu/2026-07-17/formal/results/bag01 \
  --mode detections --start-unreviewed
```

The window shows the original image on the left. The right image contains all
model predictions, while the selected prediction has a white outline.

| Key | Action |
| --- | --- |
| `A` | Accept prediction: class, mask, and center are correct (`TP`). |
| `M` | Class is correct but mask is not reasonable. |
| `C` | Class is correct but bbox/center is not reasonable. |
| `X` | Mark the selected prediction as a false positive (`FP`). |
| `0`–`4` | Select the real class (`red`, `yellow`, `green`, `blue block`, `blue obstacle`). |
| `W` | Mark selected prediction as a wrong class; records FP plus a false negative for the selected real class. |
| `F` | Record a missed real object of the selected class. Press once per missed instance. |
| `,` / `.` | Previous / next record or frame. |
| `U` | Jump to the next unreviewed prediction. |
| `Q` or `Esc` | Save completed work and exit. |

All detection decisions are written immediately to `manual_review.csv`.
Missed objects and wrong-class false negatives are appended to
`manual_false_negatives.csv`.

After reviewing predictions, inspect every frame, including frames with no
prediction, in frame mode:

```bash
conda run -n imdl python yolo_seg/scripts/review_yolo_seg_results.py \
  --images yolo_seg/experiment/replay_cpu/2026-07-17/formal/extracted/bag01 \
  --results yolo_seg/experiment/replay_cpu/2026-07-17/formal/results/bag01 \
  --mode frames
```

Use `V` to mark a frame reviewed, and select a true class then press `F` for
each real object the model missed. This GUI produces human-review evidence; a
precise mask-IoU report still requires polygon ground-truth labels.
