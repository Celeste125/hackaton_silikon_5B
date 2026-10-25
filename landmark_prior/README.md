# The two landmark models: collecting data and training

Two models, with different jobs. They are independent — model 1 is useful on
its own, and model 2 is useless without it.

| | Model 1 — detector | Model 2 — location prior |
|---|---|---|
| Question | "Is the dispenser in this image, and where?" | "Given the map so far, where is the dispenser likely to be?" |
| Input | one camera frame | the partial SLAM occupancy grid, robot-centric |
| Output | bounding boxes | a probability map per landmark |
| Called | **object detection** | **semantic map completion** (not behavioural cloning — that would output an action) |
| Labels | hand-drawn boxes | generated automatically from past runs |
| Where it runs | Jetson, on every frame | laptop, every 2 s |
| Size | YOLO11n, ~2.6M params | small U-Net, ~121k params |
| Without it | nothing works | geometry still works; you lose the head start |

Model 2 enters the estimator as a **prior**, never as an answer. Real camera
bearings outvote it in proportion to their own information, and it is never
reported as `converged`. That is deliberate: see the `Prior` docstring in
`ros2_ws/src/imdl_autonomy/imdl_autonomy/landmark_estimation.py`.

---

## Before you start: does a prior make sense for your arena?

Answer this first, because it decides whether model 2 is worth any effort:

**Are the landmarks in the same place from run to run?**

- **Yes** → a prior is meaningful. Proceed.
- **No, they are moved between attempts** → stop. There is nothing to
  predict, and training will produce a model that confidently points at where
  things used to be. Use the geometry alone; it is already good to ~5 cm.

If they stay put, what you vary between runs is **the robot's start pose and
its path**, never the landmarks. The network's input window is robot-centric,
so twenty runs that all start from the same spot and drive the same loop look
like one run to it.

---

## Model 1: the detector

Already built and pluggable; your teammate owns the training. The contract —
classes, format, export, what matters for accuracy — is
[`../ros2_ws/src/perception/docs/landmark_model_handoff.md`](../ros2_ws/src/perception/docs/landmark_model_handoff.md).
Short version:

```bash
# On the GPU box, with a labelled dataset in YOLO format:
yolo detect train data=imdl_landmarks.yaml model=yolo11n.pt imgsz=640 epochs=100

# Copy best.pt to the Jetson, then:
ros2 launch perception landmark_detector.launch.py \
    backend:=ultralytics model_path:=~/models/imdl_landmarks.pt
```

There is also a **zero-training shortcut** worth knowing about if the labelled
data is late: the existing pipeline can auto-label for you. Once a landmark
has converged geometrically, you know where it is, so you can project it back
into every recorded frame — including the ones the detector missed. That is
the highest-value use of run data, and it improves recall, which is what
actually limits you.

---

## Model 2: the location prior

### Step 1 — Prerequisites

Model 2 learns from runs that **succeeded**, so model 1 must work first and
the camera mounting must be measured. Check both:

```bash
# The detector is seeing things
ros2 topic echo /landmarks/detector_status

# And landmarks are converging, not just being detected
ros2 topic echo /landmarks/state
```

You need `"converged": true` with a `sigma_major_m` under ~0.15 m. Unconverged
runs are discarded by the dataset builder — a label the pipeline was unsure of
would teach the network this pipeline's own errors.

> If `sigma_major_m` is small but the position is consistently wrong, that is
> the camera extrinsics, and no amount of training data fixes it. Fix the
> mounting numbers in `landmark_mapping.launch.py` first.

### Step 2 — Record runs

Add `dataset_dir:=` to the mapper. Everything else is a normal exploration run:

```bash
# laptop
ros2 launch imdl_autonomy slam_navigation.launch.py run_slam:=true
ros2 launch imdl_autonomy landmark_mapping.launch.py \
    dataset_dir:=~/landmark_runs \
    output_path:=~/maps/arena_landmarks.yaml

# jetson
ros2 launch perception webcam.launch.py publish_camera_tf:=false
ros2 launch perception landmark_detector.launch.py \
    backend:=ultralytics model_path:=~/models/imdl_landmarks.pt

# laptop, once Nav2 reports "Managed nodes are active"
ros2 launch imdl_autonomy frontier_explore.launch.py
```

Note there is no `prior_source:=` above, and that is deliberate: **record
training data with the prior switched off.** A converged position is
influenced by whatever prior was active, so recording with a prior enabled
would train the next network partly on the previous one's output, which drifts
over generations. Labels should come from the camera and the geometry alone.

Each run writes `~/landmark_runs/run_<timestamp>/` holding `samples/*.npz` (a
window every 2 s) and `run.yaml` (the labels). `run.yaml` is rewritten on
every publish, not at shutdown, so Ctrl-C or a flat battery still leaves
usable data.

**Between runs, change the start pose.** Move the robot a metre, turn it
ninety degrees, start SLAM from a different corner. This is the single most
important thing you do for model 2.

Check what you got:

```bash
ls ~/landmark_runs/*/samples | wc -l          # samples
grep -A3 labels: ~/landmark_runs/run_*/run.yaml   # did it converge?
```

**How many runs?** Honestly:

| Runs | What to expect |
|---|---|
| 1–3 | Not enough to hold anything out. The validation number will be meaningless. |
| 5–10 | A usable prior for a fixed arena. Expect ~0.3–0.6 m. |
| 15+ | Diminishing returns in one arena — the limit is the number of distinct *layouts*, not runs. |

Each run gives ~40–150 samples, but samples within a run are near-duplicates,
so six runs is six data points' worth of independent information, not four
hundred.

### Step 3 — Copy the runs off the robot

```bash
rsync -av --progress frl@10.23.60.77:~/landmark_runs/ ~/landmark_runs/
```

A run is a few hundred kilobytes (the windows are stored as uint8 masks and
compressed), so this is quick even over the arena WiFi.

### Step 4 — Build the dataset

Runs on any machine with numpy; no GPU and no ROS needed.

```bash
cd ~/github/hackaton_silikon_5B
./landmark_prior/scripts/build_landmark_prior_dataset.py \
    --runs-dir ~/landmark_runs \
    --out landmark_prior/model/dataset.npz \
    --val-runs 2
```

It prints which runs it took and which it skipped, with reasons. **The
train/validation split is by whole run, never by sample** — consecutive
samples are nearly identical, so a random split would report a validation
error several times better than the truth.

Then look at it before spending GPU time:

```bash
./landmark_prior/scripts/inspect_landmark_prior_dataset.py \
    --dataset landmark_prior/model/dataset.npz \
    --render /tmp/prior_samples
```

Open a few PNGs. The robot is the green crosshair at the centre facing **down**
the image; dark cells are walls, grey is unexplored, and each coloured cross is
a landmark label. **Check the crosses sit where the landmarks really are
relative to the walls.** This catches a wrong frame or a bad extrinsic in
thirty seconds — a failure that training cannot diagnose and will happily
learn.

Also check `label spread`: if it is under ~3 cells, the robot stood in the
same place every run and there is little to learn.

### Step 5 — Train on the cloud GPU

```bash
# On the GPU box
git clone <your repo> && cd hackaton_silikon_5B
python3 -m venv .venv && source .venv/bin/activate
# onnxscript is needed by torch's ONNX exporter -- without it the training
# run completes and then fails on the very last step, at the export.
pip install torch onnx onnxscript numpy pyyaml   # CUDA wheels are the default

# Copy the dataset up (it is a single file, a few MB)
scp landmark_prior/model/dataset.npz gpubox:~/hackaton_silikon_5B/landmark_prior/model/

./landmark_prior/scripts/train_landmark_prior.py \
    --dataset landmark_prior/model/dataset.npz \
    --out-dir landmark_prior/model \
    --epochs 60 \
    --name landmark_prior_v1
```

This is a 121k-parameter network on a few hundred 128×128 samples: **a couple
of minutes on any GPU**, and it fits in well under 1 GB of VRAM. You do not
need a large instance — the cheapest one with any CUDA device is plenty. (It
trains on a CPU too, just slowly: about 10 s per epoch on a laptop.)

Mostly leave the knobs alone. The two worth knowing:

- `--augment-flip` mirrors samples left/right, **doubling your data, but only
  valid if a mirrored arena is as plausible as the real one.** Check your
  layout first.
- `--base-width` is deliberately small. Raising it makes overfitting worse,
  not better, until you have runs from several different layouts.

The saved epoch is chosen by validation **loss**, not by the mode error,
because the prior is built from the heatmap's mean *and covariance*. A network
goes overconfident — peak still in the right cell, spread far too tight — well
before its mode degrades, and a too-tight prior is the one that fights the
geometry. `--select-on mode` overrides this if you only care about the peak.

Reading the output, the column that matters is `mode_err`, the distance from
the network's most likely cell to the true one, in metres:

| val `mode_err` | Verdict |
|---|---|
| under ~0.3 m | Genuinely useful. Switch it on. |
| 0.3–0.8 m | Weak but harmless; it gets outvoted quickly. Still worth it. |
| over ~1 m, or val much worse than train | It has memorised the training runs. **More runs, not more epochs.** |

Expect the last row on your first attempt with a handful of runs. That is the
nature of one arena, not a bug.

It writes three files:

- `landmark_prior_v1.pt` — checkpoint, for resuming or fine-tuning
- `landmark_prior_v1.onnx` — **the only one the robot needs**, a single
  self-contained file (the script fails loudly rather than letting the
  exporter split the weights into a sidecar you would forget to copy)
- `landmark_prior_v1.json` — the manifest: classes, window geometry, score,
  which runs were held out. Keep it. A model whose provenance is unknown can
  only be retrained, not safely deployed.

`--export-only` re-exports an existing checkpoint without training, for when
you upgrade onnxruntime or want to deploy a checkpoint trained elsewhere.

### Step 6 — Deploy and switch it on

```bash
scp gpubox:~/hackaton_silikon_5B/landmark_prior/model/landmark_prior_v1.onnx \
    ~/models/

ros2 launch imdl_autonomy landmark_mapping.launch.py \
    prior_source:=network \
    prior_network_path:=~/models/landmark_prior_v1.onnx
```

The robot runs it through onnxruntime on the CPU — no torch on the robot, which
sidesteps the JetPack version-matching problem entirely. It takes well under a
millisecond.

The mapper refuses to start the prior if the ONNX output channel count does not
match the number of configured landmarks, because the channels are positional
and a mismatch would silently swap the dispenser and the enclosure.

### Step 7 — Check it is helping

```bash
ros2 topic echo /landmarks/state
```

Each landmark now carries `"prior": "network"`. Before the camera has seen
anything, the position comes from the prior alone and `reason` says so. In
RViz the marker starts faint (unconverged) at the predicted spot and firms up
as real views arrive.

What you should see, in order:

1. At startup, a faint marker somewhere plausible — the prior.
2. As the robot drives, the marker **moving to the truth** and the ellipse
   shrinking — the geometry taking over.
3. `"converged": true`, with `prior` still listed so you know it was involved.

If step 2 does not happen — the marker stays at the prior's guess even after
several good views — the prior is too confident. Raise `prior_inflate` (try
3.0) or `prior_min_sigma_m` (try 0.3).

---

## The shortcut: no training at all

If you only want "start where it was last time", you do not need model 2:

```bash
ros2 launch imdl_autonomy landmark_mapping.launch.py \
    prior_source:=file \
    prior_file:=~/maps/arena_landmarks.yaml \
    prior_file_sigma_m:=0.5
```

That reads the file the mapper already writes with `output_path:=`. It takes
no training, no GPU and no data collection, and for a fixed arena it is most
of the benefit.

**But it has one hard requirement the network does not:** the stored positions
are in map coordinates, so this run's map frame must match the recording run's.
That means localising with AMCL against a saved map:

```bash
ros2 launch slam localization.launch.py     # not mapping.launch.py
```

Start fresh SLAM instead and the map origin moves to wherever the robot
happens to begin, making the stored coordinates meaningless. The network has
no such requirement, because its window is robot-centric — which is most of
why it is worth the trouble.

**Try the file prior first.** If it is enough, you have saved yourself the
whole data-collection exercise.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `no usable runs` from the builder | No run converged. Check `/landmarks/state` during a run — this is a detector or extrinsics problem, not a data problem. |
| `only converged on ['cow_enclosure']` | The dispenser was never localised in that run. The run is still used, for the enclosure only. |
| `window ... does not match this code's` | Recorded with a different `WINDOW_CELLS`/resolution. Those constants invalidate old samples by design; re-record or use the older checkout. |
| Labels in the rendered PNGs sit in the wrong place | Camera extrinsics, or `map_frame` mismatch. Fix before training. |
| `label spread` under ~3 cells | Every run started from the same pose. Vary it. |
| val error ≫ train error | Memorising. More runs. |
| `no location prior (...)` at startup | Bad path or missing onnxruntime. The run continues without a prior, which is safe. |
| Prior does not get overridden by observations | Too confident: raise `prior_inflate` / `prior_min_sigma_m`. |
| `has N output channels but M landmarks were configured` | The network was trained with a different `landmarks.yaml`. Retrain or restore the config. |

## Tests

The window geometry, the heatmap-to-prior conversion and the prior's
interaction with the estimator are all pinned, and none of it needs a robot:

```bash
cd ros2_ws/src/imdl_autonomy
python3 -m pytest test/test_landmark_prior.py test/test_landmark_estimation.py
```
