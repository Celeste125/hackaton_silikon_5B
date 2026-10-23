# RGB-D Perception Visualisation

`perception_visualizer` is a diagnostic and demonstration node. It does not
change classifications, depth estimates, target selection, or navigation
commands.

`camera.launch.py` starts it automatically. In `runtime_mode:=debug` it
publishes a JPEG stream at `/perception/fused_debug/compressed`:

- left: YOLO segmentation/debug image with each valid object's optical depth
  (`Z`) and base-frame planar range (`R`);
- right: colourised aligned depth. Warm pixels are near, cool pixels are far,
  and black pixels are invalid or outside the configured depth range.

The node also publishes `visualization_msgs/msg/MarkerArray` at
`/perception/object_markers`. It includes only objects with
`map_pose_valid=true` and a finite `position_map_m`. Block classes use cube
markers and `blue_obstacle` uses a cylinder marker. Marker dimensions and
height are display aids only; they are not measurements of object size.

## Laptop Viewer

Use the direct compressed-image viewer rather than selecting a transport
suffix in `rqt_image_view`:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 yolo_seg/scripts/view_yolo_debug.py --ros-args \
  -p topic:=/perception/fused_debug/compressed
```

## RViz

Set `Fixed Frame` to `map`, add `Map`, `LaserScan`, and `TF`, then add a
`MarkerArray` display with topic `/perception/object_markers`. The markers
should remain approximately fixed on the map while the robot moves around a
stationary block.
