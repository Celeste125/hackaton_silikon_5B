# Local Pre-trained Weights

This directory contains optional local generic weights used to initialise or
compare training experiments, such as `yolo11n-seg.pt` and `yolo26n.pt`.

They are not part of the ROS runtime and are intentionally ignored by Git.
The single deployable perception model is:

`../../ros2_ws/src/perception/models/yolo11n_seg_imdl.pt`

Its provenance and SHA256 are recorded in
`../registry/yolo11n_seg_imdl_v1/manifest.md`.
