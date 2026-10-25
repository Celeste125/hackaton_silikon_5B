# Trained location priors

Weights and datasets live here and are **not** in Git, matching
`../../yolo_seg/model/`. What is kept is the provenance: each model's
`*.json` manifest, which records the classes, the window geometry, the
validation score and which runs were held out.

Typical contents after a training round:

| File | In Git | What it is |
|---|---|---|
| `dataset.npz` | no | Built by `../scripts/build_landmark_prior_dataset.py` |
| `landmark_prior_v1.pt` | no | Torch checkpoint, for resuming or re-exporting |
| `landmark_prior_v1.onnx` | no | **The file the robot loads.** Copy this to the laptop |
| `landmark_prior_v1.json` | yes | The manifest. Keep it with the model |

A model without its manifest cannot be safely deployed: the output channels
are positional, so there is no way to tell from the file alone which channel
is the dispenser and which is the enclosure, nor what window geometry it was
trained for. If you have lost a manifest, retrain rather than guess.

See [`../README.md`](../README.md) for collecting data and training.
