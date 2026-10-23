# Jetson YOLO Runtime Preparation

The perception package intentionally does not install PyTorch or Ultralytics
through rosdep. NVIDIA Jetson requires a PyTorch build compatible with its
JetPack release, CUDA, Python version, and ARM64 architecture.

Before installing anything, record the Jetson runtime:

```bash
cat /etc/nv_tegra_release
python3 --version
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

Install the NVIDIA-supported Jetson PyTorch build first. Verify CUDA is
available before installing Ultralytics. Pin the training-compatible version:

```bash
python3 -m pip install --no-deps ultralytics==8.4.87
```

`--no-deps` prevents pip from replacing the Jetson-specific PyTorch package.
Install any missing non-PyTorch dependencies separately after inspecting the
error. Do not copy the laptop Conda environment to the Jetson.

Synchronize the required source packages from the laptop. The deployment script
copies both `imdl_msgs` and `perception`, because the typed output depends on
`imdl_msgs`. Run without `--apply` first to inspect the transfer, then use
`--apply` to copy and build remotely:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory
./yolo_seg/scripts/deploy_perception_to_jetson.sh --host=jetson@jetson-desktop.local
./yolo_seg/scripts/deploy_perception_to_jetson.sh --host=jetson@jetson-desktop.local --apply
```

Verify the frozen model after synchronization:

```bash
cd ~/Desktop/ros2_ws/src/perception/models
sha256sum -c yolo11n_seg_imdl.sha256
```

Build and launch the full pipeline only after the runtime imports succeed:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs perception --symlink-install
source install/setup.bash
ros2 launch perception camera.launch.py
```

The formal launch uses `config/perception.yaml`. Use deployment mode by default;
use `runtime_mode:=debug` only when `/yolo/debug_image` is needed. The standalone
`yolo_segmentation.launch.py` remains available for focused diagnosis.

Export TensorRT on the Jetson itself after `.pt` smoke testing. Do not copy a
TensorRT engine built on the laptop. Keep the `.pt` model as the fallback.
