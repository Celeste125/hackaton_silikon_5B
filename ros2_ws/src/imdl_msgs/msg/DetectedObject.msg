# The header is copied from the source RGB image. Image coordinates use its
# frame, while position_base_m and position_map_m use the configured fixed
# base and map frames respectively.
std_msgs/Header header

# Frozen IMDL class information. blue_obstacle must always set is_block=false.
uint8 class_id
string class_name
bool is_block
float32 confidence

# Image-plane information in pixels. bbox_xyxy_px is [x_min, y_min, x_max, y_max].
float32 center_u_px
float32 center_v_px
float32[4] bbox_xyxy_px
geometry_msgs/Point32[] mask_polygon_px

# Metric geometry. Positions are in metres; bearing_rad is relative to base.
float32 depth_m
geometry_msgs/Point position_camera_m
geometry_msgs/Point position_base_m
geometry_msgs/Point position_map_m
float32 range_m
float32 bearing_rad

# Validity and diagnostic information. Position values are meaningful only when
# their matching validity flag is true. valid_depth_ratio/depth_mad_m are
# meaningful only when depth_valid is true.
bool depth_valid
bool base_pose_valid
bool map_pose_valid
bool is_stale
string transform_status
uint32 mask_area_px
uint32 valid_depth_pixels
float32 valid_depth_ratio
float32 depth_mad_m
