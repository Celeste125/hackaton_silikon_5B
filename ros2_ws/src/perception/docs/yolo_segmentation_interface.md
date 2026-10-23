# YOLO Segmentation ROS 2 Interface Contract

Status: frozen for schema version 1.

This contract defines the boundary between the YOLO segmentation node and the
geometry backend. Schema version 1 intentionally uses `std_msgs/msg/String`
JSON to preserve compatibility with the existing perception package. A future
typed message must use a new schema version.

## Frozen Model

- Model version: `imdl_yolo_seg_v1`
- Artifact: `models/yolo11n_seg_imdl.pt`
- Architecture: `YOLO11n-seg`
- Input size: 640 pixels
- SHA256: `63481d80ed1d4f9b2a70e6c540cc484c28631308d189d6c4b565b22f14de086e`
- Default confidence threshold: 0.35
- Default NMS IoU threshold: 0.70

The class IDs are immutable for schema version 1:

| ID | Name | `is_block` |
|---:|---|---|
| 0 | `red_block` | `true` |
| 1 | `yellow_block` | `true` |
| 2 | `green_block` | `true` |
| 3 | `blue_block` | `true` |
| 4 | `blue_obstacle` | `false` |

`blue_obstacle` is a perception result but must never be selected as a block
target. Consumers must use `class_id` or `is_block`, not color substring
matching.

## Input

Default topic: `/camera/camera/color/image_raw`

Type: `sensor_msgs/msg/Image`

QoS: ROS sensor-data QoS (best effort, volatile, small queue). The node must
process the newest available image instead of building an inference backlog.
The input image header is authoritative for timestamp and frame ID.

## Detection Output

Default topic: `/yolo/detections`

Type: `std_msgs/msg/String`

QoS: reliable, volatile, depth 10.

The string contains one JSON object per processed frame:

```json
{
  "schema_version": 1,
  "model_version": "imdl_yolo_seg_v1",
  "timestamp": {
    "sec": 1780000000,
    "nanosec": 123456789
  },
  "frame_id": "camera_color_optical_frame",
  "image_size": {
    "width": 640,
    "height": 480
  },
  "inference_ms": 8.4,
  "detections": [
    {
      "class_id": 3,
      "class_name": "blue_block",
      "is_block": true,
      "confidence": 0.98,
      "bbox_xyxy_px": [100.0, 150.0, 180.0, 240.0],
      "center_px": {
        "u": 140.0,
        "v": 195.0
      },
      "mask_polygon_px": [
        [105.0, 155.0],
        [176.0, 153.0],
        [180.0, 235.0],
        [103.0, 238.0]
      ]
    }
  ]
}
```

Detections are ordered by descending confidence. Consumers must not use array
order as a target-selection policy.

## Pixel Coordinate Rules

- The image origin is the top-left corner.
- `u`/`x` increases to the right; `v`/`y` increases downward.
- All geometry is expressed in source-image pixels, not model-letterbox pixels.
- `bbox_xyxy_px` is `[x_min, y_min, x_max, y_max]` using a half-open maximum.
- `center_px` is the mask centroid; the bbox center is the fallback.
- `mask_polygon_px` is the outer contour with at least three points.
- Coordinates must be finite and clamped to the image bounds.

The geometry backend must rasterize `mask_polygon_px` and sample aligned depth
inside that mask. It must not estimate range from bbox size or known block size.

## Empty and Invalid Results

The node publishes a fresh message with `"detections": []` when no object is
detected. It must not repeat detections from an older frame.

- A model-load failure is fatal and must stop node startup.
- An image conversion or inference failure drops only that frame and emits a
  rate-limited warning.
- An invalid polygon drops only that instance.
- Timestamp freshness is evaluated by downstream consumers.

## Debug Output

Raw topic: `/yolo/debug_image`

Type: `sensor_msgs/msg/Image`

QoS: ROS sensor-data QoS.

The debug image uses the input header and displays masks, bounding boxes, class
names, confidence values, and mask centers. Debug publication can be disabled
without changing detection output.

Compressed topic: `/yolo/debug_image/compressed`

Type: `sensor_msgs/msg/CompressedImage`

This is a JPEG encoding of the same annotated image. Use it for remote laptop
visualization over Wi-Fi; it avoids sending the full raw BGR image over DDS.

## Frozen Parameter Names and Defaults

| Parameter | Default |
|---|---|
| `model_path` | package model `yolo11n_seg_imdl.pt` |
| `image_topic` | `/camera/camera/color/image_raw` |
| `detections_topic` | `/yolo/detections` |
| `debug_image_topic` | `/yolo/debug_image` |
| `debug_compressed_image_topic` | `/yolo/debug_image/compressed` |
| `debug_jpeg_quality` | `75` |
| `confidence_threshold` | `0.35` |
| `iou_threshold` | `0.70` |
| `image_size` | `640` |
| `device` | `0` |
| `max_inference_rate_hz` | `10.0` |
| `publish_debug_image` | `true` |

The old HSV `/detected_color` topic is not part of this pipeline.
`target_pose_estimator` consumes `/yolo/detections`; see
`object_geometry_interface.md` for the 3D and target-selection interfaces.
