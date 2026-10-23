# yolo11n_seg_imdl_v1 Provenance

| Item | Value |
| --- | --- |
| Dataset release | `../../data/releases/final_v1` |
| Training output | `../../experiment/training/final_yolo11n_seg` |
| Held-out evaluation | `../../experiment/evaluation/final_yolo11n_test` |
| Deployment model | `ros2_ws/src/perception/models/yolo11n_seg_imdl.pt` |
| Model SHA256 | `63481d80ed1d4f9b2a70e6c540cc484c28631308d189d6c4b565b22f14de086e` |
| Dataset YAML SHA256 before consolidation | `f28eb547639fd2b575f299dd1a368c80b68186ce09b0b60ae8daede13fbda55c` |
| Dataset YAML SHA256 after consolidation | `2321da2f1c101a0e794a3c0ad8778d14b5f88500282a5f5c45d1b9646c53db5c` |
| Class order | `red_block`, `yellow_block`, `green_block`, `blue_block`, `blue_obstacle` |

The deployment weight and final training `best.pt` have identical SHA256
digests. Dataset path edits made during consolidation do not alter images or
labels; the YAML hash above is retained as pre-move provenance.

## Layout Validation

On 2026-07-15, `yolo segment val` successfully loaded the relocated release
dataset and final checkpoint. The test split contained 28 images and 46
instances; the resulting mask mAP50 was 0.984 and mask mAP50-95 was 0.865.
