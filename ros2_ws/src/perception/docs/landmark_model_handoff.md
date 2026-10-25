# Landmark localisation: model handoff

Putting the cow enclosure and the food dispenser on the map, with a model that
runs on the robot. No API key, no cloud, and no images on the network.

This document is the contract between whoever trains the model and the code
that is already written and tested. **The pipeline works today without a
model** (`backend:=stub`), so the geometry, the extrinsics and the fusion can
be verified before the weights arrive, and swapping the stub for the real
model is one launch argument.

## What the model must deliver

| | |
|---|---|
| Task | Object **detection** (boxes). A segmentation model is fine -- its boxes are used and its masks ignored. |
| Framework | Ultralytics YOLO (v8/v11 or newer). |
| Classes | `food_dispenser` and `cow_enclosure`. Other spellings are fine; see below. |
| Format | `.pt`. Optionally also `.onnx`, and a `.engine` exported **on the Jetson itself**. |
| Image size | Whatever it was trained at; pass it as `image_size:=`. 640 is the default. |

Class names are matched case- and separator-insensitively, so `Food
Dispenser`, `food-dispenser` and `food_dispenser` all work. If the training
set ended up with different names entirely, do not retrain -- add them to
`model_classes` in [`config/landmarks.yaml`](../config/landmarks.yaml):

```yaml
landmarks:
  food_dispenser:
    model_classes: [food_dispenser, dispenser, feeder, hopper]
```

A class the model emits that no landmark claims is ignored with one log line,
so extra classes in the model cost nothing.

### What matters for accuracy, and what does not

The fusion turns each box into a **bearing**, so the only thing that matters
about a box is where its centre is. Consequences worth passing to whoever is
labelling:

* **Label the whole object, consistently.** The model's idea of the object's
  centre is what gets triangulated. A tight box on the dispenser body in every
  frame is worth more than a very accurate box that sometimes includes the
  stand and sometimes does not: inconsistency is bearing noise.
* **Recall beats precision, within reason.** A missed frame costs one view out
  of many. A false positive on the wrong object drags the estimate -- though
  the fusion gates outliers once it has four or more views, and logs them as
  `views_rejected`.
* **Boxes that run off the frame are fine.** They are detected and
  automatically trusted less, because a clipped object's visible centre is not
  its real centre.
* **Do not bother with distance.** Nothing asks the model for range. Range
  comes from geometry (below), which is why a plain webcam is enough.

## Plugging it in

Nothing but the launch arguments change.

```bash
# No weights yet: a fixed box, to check the camera mounting and the fusion.
ros2 launch perception landmark_detector.launch.py backend:=stub \
    stub_detections:='[{"class":"food_dispenser","box":[0.40,0.40,0.60,0.70]}]'

# The trained weights.
ros2 launch perception landmark_detector.launch.py \
    backend:=ultralytics model_path:=/home/jetson/models/imdl_landmarks.pt

# Faster, once the .pt is known to work. Export on the Jetson, never copy an
# engine built elsewhere.
yolo export model=imdl_landmarks.pt format=engine half=True device=0
ros2 launch perception landmark_detector.launch.py \
    backend:=ultralytics model_path:=/home/jetson/models/imdl_landmarks.engine

# If torch/Ultralytics will not install on this Jetson: ONNX instead.
# An ONNX graph carries no labels, so give it the dataset YAML.
ros2 launch perception landmark_detector.launch.py backend:=onnx \
    model_path:=/home/jetson/models/imdl_landmarks.onnx \
    class_names_path:=/home/jetson/models/imdl_landmarks.yaml
```

`backend:=claude` also exists, and is the only backend that sends frames off
the robot. It is a stand-in for before the model exists, not a deployment.

Adding a fifth backend is one class and one `register_backend` call in
[`landmark_backends.py`](../perception/landmark_backends.py); nothing else in
the pipeline knows which backend is running.

See [`jetson_yolo_runtime.md`](jetson_yolo_runtime.md) for getting a
JetPack-matched torch and Ultralytics onto the Jetson in the first place.

## How it runs

Two nodes, and the split between them is deliberate.

```
  Jetson (camera)                        laptop (SLAM, Nav2, TF)
  ---------------                        -----------------------
  webcam / RealSense
        |
        v
  landmark_detector  --- /landmarks/detections --->  landmark_mapper
  - runs the model       (JSON, <1 kB/frame)         - TF at the image stamp
  - resolves intrinsics                              - fuses all views
  - samples depth locally                            - markers, poses, YAML
```

`landmark_detector` publishes boxes, the resolved intrinsics, and -- with a
depth camera -- the range already sampled out of the depth image. The mapper
therefore needs no camera topics at all. Two things follow:

* **No frames cross the WiFi.** The arena link is congested and a VGA stream
  would fight the lidar and Nav2 for it. Under a kilobyte a frame does not.
* **The TF chain stays on one machine.** The fusion does TF lookups at each
  image's own timestamp, and this project has already learned that TF cannot
  be split across the WiFi. So the mapper runs on the laptop with SLAM, and
  the camera's static extrinsic transform is published *there too*:

```bash
# jetson
ros2 launch perception webcam.launch.py publish_camera_tf:=false
ros2 launch perception landmark_detector.launch.py backend:=ultralytics \
    model_path:=~/models/imdl_landmarks.pt

# laptop
ros2 launch imdl_autonomy slam_navigation.launch.py run_slam:=true
ros2 launch imdl_autonomy landmark_mapping.launch.py   # publishes the camera TF
ros2 launch imdl_autonomy frontier_explore.launch.py   # once Nav2 is active
```

Nothing needs triggering. The detector runs on every frame it can and the
mapper fuses continuously while the explorer drives.

## How the range is obtained

A bearing alone does not locate anything -- it is a line. There are three ways
to get the missing distance, and the mapper uses whichever is available:

| Source | Needs | Accuracy | Notes |
|---|---|---|---|
| `depth` | RGB-D camera, depth aligned to colour | best, ~5-10 cm | One view is enough. Sampled on the Jetson from the middle of the box. |
| `triangulate` | the robot to have moved between views | good, improves with baseline | The path for a plain C920. Two views from the same spot do **not** work. |
| `wall` | `/map` from SLAM | poor, biased | Casts the bearing at the wall behind the object. Reads the object as being *at* the wall, so it is wrong by the standoff. Used only to bootstrap, then dropped automatically. |

Which were used appears in `/landmarks/state` as `sources`, and a run that
only ever managed wall casts is flagged `wall_cast_bias: true` -- that bias is
systematic and no number of views removes it.

## Why the estimate improves with more angles

Each view contributes the constraint that the landmark lies on its sight line,
weighted by `range * bearing_sigma`. Those constraints are accumulated in
information form and solved for a position **and a covariance**. The full
reasoning is in
[`landmark_estimation.py`](../../imdl_autonomy/imdl_autonomy/landmark_estimation.py);
what it means in practice:

* Views from **new angles** shrink the estimate fast. Roughly `1/sqrt(views)`.
* Views from the **same angle** barely help, and the published covariance says
  so rather than looking confident. This is the main thing the old averaging
  approach got wrong: averaging two deprojected points from nearly the same
  place produces a tight-looking answer that can be metres off.
* Two views from **exactly opposite sides** do not triangulate either -- they
  are the same sight line. The reported baseline is an angle between lines,
  not a distance between camera positions, for exactly this reason.
* `/landmarks/<name>/next_view` publishes a pose to drive to that would shrink
  the current estimate fastest: a standpoint across the long axis of the error
  ellipse, facing the landmark. It is a `PoseStamped`, so it can be handed
  straight to Nav2 if the exploration should start caring about landmark
  quality.

A landmark counts as localised once its worst axis is below `max_sigma_m`
(default 0.25 m), at which point one log line is printed.

**The one error this cannot fix is the camera mounting.** An extrinsic error
is the same on every view, so it does not average out, and the covariance will
happily report a tight estimate that is systematically displaced. Measure
`camera_x/y/z/roll/pitch/yaw` in `landmark_mapping.launch.py` on the real
robot. There is a test pinning this distinction
(`test_bearing_noise_averages_out_but_an_extrinsic_bias_does_not`).

## Checking it

```bash
# Is the detector alive and seeing anything?
ros2 topic echo /landmarks/detector_status

# The fused estimates, with their error bars.
ros2 topic echo /landmarks/state

# What the model actually sees -- costs bandwidth, so for debugging only.
ros2 launch perception landmark_detector.launch.py ... publish_debug_image:=true
ros2 run rqt_image_view rqt_image_view /landmarks/debug_image/compressed
```

In RViz, the navigation profile (`imdl_simulation/rviz/real_create3.rviz`)
already carries a **Landmarks** display on `/landmarks/markers`. Each landmark
shows a sphere, a translucent 2-sigma ellipse and a label with its uncertainty,
view count and baseline. Watching the ellipse start as a long streak and round
off as the robot drives around is the feature working.

Pass `output_path:=~/maps/arena_landmarks.yaml` to the mapper to have the
landmarks written beside the map as they settle, so a restart keeps them.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `backend ... did not load` | Bad `model_path`, or torch/Ultralytics/onnxruntime missing. The node keeps running and keeps reporting the camera, so the run is not lost. |
| `none of the model classes ... appear in the landmark config` | The model's labels differ from `model_classes`. Fix the YAML, not the model. |
| Detector publishes nothing, no errors | Confidence below the per-landmark `min_confidence`, or the model genuinely sees nothing. Turn on `publish_debug_image` and look. |
| `no transform ... at the image stamp` | The camera frame is not in the TF tree, or detections arrive later than `tf_timeout_sec`. Check `publish_camera_tf` is on exactly one machine, and that `camera_frame` matches the driver's `frame_id`. |
| Estimate stays unconverged with a long thin ellipse | Every view came from the same direction. Drive to `/landmarks/<name>/next_view`. |
| `camera_info is uncalibrated` / `FOV fallback` | `v4l2_camera` publishes a zeroed K until given a calibration. Good to a couple of degrees; run `camera_calibration` for better bearings. |
| Landmark lands consistently off by the same amount | Camera extrinsics. No amount of views fixes this. |
| `depth is WxH but colour is WxH` | The depth topic must be the aligned-to-colour one, not raw depth. |

## The second model: a learned location prior

Everything above is the **detector** -- model 1. There is a second, optional
model that predicts where a landmark probably is from the partial SLAM map,
before the camera has seen it. It enters the estimator as a prior, so it can
never override a real measurement, and it is off by default.

Collecting its training data and training it is
[`../../../../landmark_prior/README.md`](../../../../landmark_prior/README.md).
It needs this detector working first, because its labels are generated from
runs that localised successfully.

## Tests

None of them need a robot, a camera or a model:

```bash
colcon test --packages-select perception imdl_autonomy
# or directly:
cd ros2_ws/src/perception     && python3 -m pytest test/test_landmark_detection.py
cd ros2_ws/src/imdl_autonomy  && python3 -m pytest test/test_landmark_estimation.py \
                                                  test/test_landmark_pipeline.py \
                                                  test/test_landmark_prior.py \
                                                  test/test_landmark_wiring.py
```

`test_landmark_pipeline.py` is the one worth reading: it places a landmark,
projects it into a camera at several poses, pushes the result through the real
wire format and the real fusion, and checks both that the answer is right and
that the reported uncertainty actually bounds the true error.
