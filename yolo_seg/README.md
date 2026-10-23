# YOLO Segmentation Assets

This directory is the single parent for all YOLO segmentation data and
experiment outputs. It separates reproducible release inputs from raw capture
and intermediate annotation work.

## Layout

- `data/releases/final_v1`: canonical dataset used for the final YOLO11n-seg
  training and held-out evaluation.
- `data/raw_bags`: original RealSense RGB rosbag recordings.
- `data/cvat_exports`: annotation exports retained as source evidence.
- `data/archive`: pilot, candidate, pseudo-label, and manual-seed datasets.
- `experiment/training`: training outputs and weights.
- `experiment/evaluation`: test metrics, prediction images, and validation
  outputs.
- `experiment/archive`: one-off smoke experiments.
- `registry/yolo11n_seg_imdl_v1`: provenance for the frozen deployment model.
- `model`: local generic pre-trained weights used to start experiments. They are
  not runtime assets and are intentionally ignored by Git.
- `scripts`: data extraction, CVAT import, training-set assembly, evaluation,
  visualization, and Jetson deployment helpers.

## Deployment Model

The Jetson runtime model remains deliberately outside this directory at:

`ros2_ws/src/perception/models/yolo11n_seg_imdl.pt`

Do not replace or move that file as part of dataset organisation. Its SHA256
matches the final training checkpoint recorded in the registry manifest.

## CPU-Only Offline Replay

Use the ROS Distrobox to extract RGB images from a rosbag, then use the Conda
`imdl` environment for CPU-only YOLO inference. The bags are RGB-only evidence:
their replay can validate classes, masks, bounding boxes, and image positions,
but cannot validate depth, TF, base/map coordinates, or grasp accuracy.

```bash
# ROS extraction environment
distrobox enter imdl
source /opt/ros/humble/setup.bash
python3 yolo_seg/scripts/extract_rosbag_images.py BAG_DIRECTORY OUTPUT_DIRECTORY \
  --interval 5

# CPU inference environment
conda run -n imdl python yolo_seg/scripts/replay_yolo_seg_cpu.py \
  --input OUTPUT_DIRECTORY --output REPLAY_OUTPUT_DIRECTORY \
  --model ros2_ws/src/perception/models/yolo11n_seg_imdl.pt \
  --device cpu --confidence 0.35 --iou 0.70 --imgsz 640
```

The replay script writes per-image JSON, annotated images, a contact sheet,
timing summary, environment/model manifest, and a manual-review CSV template.
Use `docs/cpu_replay_manual_review.md` to review those results interactively.
