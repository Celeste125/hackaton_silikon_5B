# grain_mixing

ROS 2 (Humble) package that analyses a top-down view of a grain mixer. It detects peas, rapeseed and wheat, and publishes the area share of each grain type and a homogeneity index (1 = uniform mix, 0 = fully segregated).

The detection is classical computer vision (OpenCV + NumPy), with no neural network and no GPU. See `grain_mixing/pipeline.py`.

## Build

```bash
cd ~/hackaton_silikon_5B/ros2_ws
colcon build --packages-select grain_mixing
source install/setup.bash
```

If the build fails with `Permission denied` on `log/`, the `build`, `install` and `log` folders were created by the Docker container (root). Take ownership of them first:

```bash
sudo chown -R $USER:$USER build install log
```

A `not found: .../imdl_simulation/local_setup.bash` message when sourcing is a harmless warning from a stale entry in `install/`.

## Quick start

Open a terminal and run the launch file with a video:

```bash
cd ~/hackaton_silikon_5B/ros2_ws
source install/setup.bash
ros2 launch grain_mixing grain_mixing.launch.py \
  video_path:=/home/ce/hackaton_silikon_5B/vision_system/videos/clip_hd.mp4 \
  rate:=1.0
```

You should see `grain_analyzer reading ... at 1.0 frames/s`. Then open a second terminal (run `source install/setup.bash` again in every new terminal) and read the results:

```bash
ros2 topic echo /grain_analyzer/homogeneity
```

A new number appears about once per second. Stop the launch with `Ctrl+C` (a long Python error message at that point is harmless).

What the launch does: it starts one node, `grain_analyzer`, with the parameters of `config/params.yaml`, plus the two arguments you pass:

- `video_path:=...` is the video file (or camera index) the analyzer reads. Leave it out to make the analyzer wait for images on `/image_raw` instead (see "From the `image_raw` topic" below).
- `rate:=...` is how many frames per second are analysed.

The test video is not on this branch. Use any video of the mixer filmed from above, at 1920 x 1080, or adjust the crop (see Tuning).

## Run

The analyzer gets its frames from one of two places. Use the launch file in both cases.

### 1. From a video file or a camera (recommended for testing)

The analyzer opens the source itself, so no images travel between nodes.

```bash
ros2 launch grain_mixing grain_mixing.launch.py \
  video_path:=/home/ce/hackaton_silikon_5B/vision_system/videos/clip_hd.mp4 \
  rate:=1.0
```

`video_path` is a video file or a camera index (`video_path:=0`). Use an absolute path.

### 2. From the `image_raw` topic (for example a camera driver on the Jetson)

Leave `video_path` out. The analyzer subscribes to `/image_raw` (encoding `bgr8` or `rgb8`):

```bash
ros2 launch grain_mixing grain_mixing.launch.py rate:=1.0
```

To test this mode without a camera, play the video as a fake camera in another terminal:

```bash
ros2 run grain_mixing video_source --ros-args \
  -p video_path:=/home/ce/hackaton_silikon_5B/vision_system/videos/clip_hd.mp4 \
  -p rate:=10.0
```

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `video_path` | empty | Video file or camera index. Empty: subscribe to `image_raw`. |
| `rate` | `2.0` | Frames analysed per second. The other frames are dropped. |

## Topics

| Topic | Type | Content |
|---|---|---|
| `/grain_analyzer/homogeneity` | Float32 | Mixing index, 0-1 |
| `/grain_analyzer/coverage` | Float32 | Fraction of the mixer views covered by grain, 0-1 |
| `/grain_analyzer/area/pea` | Float32 | Percent of the grain area that is peas |
| `/grain_analyzer/area/rapeseed` | Float32 | Percent that is rapeseed |
| `/grain_analyzer/area/wheat` | Float32 | Percent that is wheat |
| `/grain_analyzer/overlay` | Image | Frame with colour overlay and legend |
| `/grain_analyzer/stacked` | Image | Raw frame above the overlay |
| `/grain_analyzer/overlay/compressed` | CompressedImage | The overlay as JPEG (about 50 kB). The web panel shows it as the "Grain mixing" camera. |

`/image_raw` is only used in mode 2 (published by your camera, or by `video_source`). The two image outputs are only encoded while something is subscribed to them.

## Subscribe

```bash
ros2 topic list
ros2 topic echo /grain_analyzer/homogeneity
ros2 topic hz /grain_analyzer/homogeneity
ros2 run rqt_image_view rqt_image_view /grain_analyzer/stacked
```

In Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Listener(Node):
    def __init__(self):
        super().__init__('homogeneity_listener')
        self.create_subscription(Float32, '/grain_analyzer/homogeneity', self.on_msg, 10)

    def on_msg(self, msg):
        self.get_logger().info(f'homogeneity: {msg.data:.2f}')


rclpy.init()
rclpy.spin(Listener())
```

## Low CPU use (Jetson)

- `max_rate` (the `rate` launch argument): frames analysed per second. A frame takes about 60 ms on one laptop core, so 1 frame per second uses roughly 20% of one core there. The Jetson will need more.
- `cv_threads` (default `1`): OpenCV threads, so the analyzer does not compete with other tasks.

Change them in `config/params.yaml` or at runtime:

```bash
ros2 param set /grain_analyzer max_rate 1.0
```

## Frame loss on `image_raw`

A 1920 x 1080 frame is 6 MB. Sent best-effort over the default network settings, most frames are lost (a test got 44 of 133). Reliable delivery received all of them, so:

- The analyzer subscribes **reliable** by default (`reliable_image: true`) and `video_source` publishes reliable.
- A reliable subscriber does not connect to a best-effort camera driver, and nothing is reported. Check the driver with `ros2 topic info /image_raw -v`. If it is `BEST_EFFORT`, set `-p reliable_image:=false` on the analyzer (expect lost frames) or make the camera reliable.
- Raising the kernel limits also helps a little (resets at reboot):

```bash
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.core.rmem_max=2147483647
```

`ros2 topic hz /image_raw` can show a low number even when the analyzer is fine. Check `/grain_analyzer/homogeneity` instead.

## Tuning

The crop window and the two circles in `config/params.yaml` are tuned for the test video (1920 x 1080). With another camera or framing, adjust them. Every parameter can be changed while the node runs:

```bash
ros2 param set /grain_analyzer rapeseed_threshold 70
ros2 param set /grain_analyzer circle_radius 300
```

Float parameters need a decimal point (`15.0`), integer parameters must not have one. Copy values you want to keep into `config/params.yaml` and rebuild.
