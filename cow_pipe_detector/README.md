# Cow + pipe detector

One YOLOv8 detection model (bounding boxes) with three classes:

| id | class    | what is labeled                                   |
|----|----------|---------------------------------------------------|
| 0  | cow_head | head of a cow cutout, incl. ears/horns            |
| 1  | cow_body | visible torso + legs, head excluded               |
| 2  | pipe     | the vertical pipes hanging from the wooden frame  |

The deployable weights live with the ROS package:
`ros2_ws/src/perception/models/cows_pipes_yolov8.pt` (manifest: `cows_pipes_yolov8.yaml`).

## Test it with a camera (laptop, no ROS needed)

```bash
pip install -r cow_pipe_detector/requirements.txt
python cow_pipe_detector/test_camera.py --list        # which camera numbers exist
python cow_pipe_detector/test_camera.py --camera 0    # live view
python cow_pipe_detector/test_camera.py --source clip.mp4 --record
```

Keys: `q` quit, `s` snapshot (annotated + raw, into `snapshots/`), `+`/`-` confidence, space pause.
Raw snapshots of frames the model gets wrong are the best new training data.

## Test it on the robot (ROS)

```bash
ros2 launch perception landmark_detector.launch.py backend:=ultralytics \
    model_path:=<repo>/ros2_ws/src/perception/models/cows_pipes_yolov8.pt \
    publish_debug_image:=true
ros2 run rqt_image_view rqt_image_view /landmarks/debug_image/compressed
```

Only `cow_head` and `cow_body` are claimed in `perception/config/landmarks.yaml`
(as `cow_enclosure`). `pipe` detections are ignored by the landmark pipeline,
with one log line, until a pipe landmark is added there.

## Retrain

1. Open `train_cows_pipes_yolov8.ipynb` in Google Colab (T4 GPU).
2. Upload the cow dataset (CVAT "Ultralytics YOLO Detection 1.0" export) and the
   Roboflow pipe export; the notebook merges them with `merge_datasets.py`
   (pipe class 0 -> 2, polygons -> boxes).
3. Train, download `best.pt`, and replace `models/cows_pipes_yolov8.pt`.

Locally: `python merge_datasets.py --cows cows.zip --pipes pipes.zip --out combined_dataset`
then `yolo detect train data=combined_dataset/data.yaml model=yolov8s.pt imgsz=640 epochs=120`.

## Data

- Cows: 170 frames (2 fps) from 6 indoor videos, 42 empty negatives, reviewed in CVAT.
- Pipes: 134 images from Roboflow (`pipe-detection-rdfwn` v1).
- Combined: 230 train / 61 val / 13 test images.
- Val frames come from the same recordings as training frames, so val scores are
  optimistic. Test on new footage.
