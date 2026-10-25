"""Checks on the landmark detection interface.

Three things here would be expensive to debug on the robot and cost nothing to
pin: the class mapping (a mismatch silently discards every detection), the
unit handling in the depth and image decoders (a millimetre/metre slip puts a
landmark a kilometre away and looks like a TF fault), and the wire format the
fusion node on the other machine depends on.
"""

import json
import math

import numpy as np
import pytest

from perception.landmark_backends import available_backends, create_backend
from perception.landmark_detection import (
    LandmarkDetection,
    build_payload,
    canonical_class,
    class_lookup,
    color_image_to_bgr,
    depth_image_to_metres,
    depth_in_box,
    load_landmark_specs,
    normalise_box,
    parse_payload,
    resolve_intrinsics,
    touches_image_edge,
)

WIDTH, HEIGHT = 640, 480

CONFIG = {
    'landmarks': {
        'food_dispenser': {
            'model_classes': ['food_dispenser', 'Feeder'],
            'min_confidence': 0.45,
            'bearing_sigma_deg': 3.0,
        },
        'cow_enclosure': {
            'model_classes': ['cow-enclosure'],
            'min_confidence': 0.5,
            'bearing_sigma_deg': 7.0,
        },
    },
}


# ------------------------------------------------------------ class mapping


def test_spellings_of_a_class_name_all_match():
    """A teammate's labels may not match the config's capitalisation."""
    for spelling in ('Food Dispenser', 'food-dispenser', 'FOOD_DISPENSER  '):
        assert canonical_class(spelling) == 'food_dispenser'


def test_the_lookup_covers_every_alias():
    lookup = class_lookup(load_landmark_specs(CONFIG))
    assert lookup['feeder'].name == 'food_dispenser'
    assert lookup['cow_enclosure'].name == 'cow_enclosure'


def test_a_class_claimed_twice_is_refused():
    """Otherwise which landmark a detection feeds depends on dict order."""
    with pytest.raises(ValueError, match='more than one landmark'):
        load_landmark_specs({'landmarks': {
            'a': {'model_classes': ['shared']},
            'b': {'model_classes': ['shared']},
        }})


def test_a_malformed_spec_is_refused_not_defaulted():
    for entry in (
        {'model_classes': ['x'], 'min_confidence': 1.5},
        {'model_classes': ['x'], 'bearing_sigma_deg': 0.0},
        {'model_classes': [''], },
        {'model_classes': ['x'], 'color': [1.0, 0.0]},
    ):
        with pytest.raises(ValueError):
            load_landmark_specs({'landmarks': {'a': entry}})


def test_an_empty_config_is_refused():
    with pytest.raises(ValueError):
        load_landmark_specs({})


def test_per_landmark_confidence_is_applied():
    specs = load_landmark_specs(CONFIG)
    assert specs['food_dispenser'].accepts(0.45)
    assert not specs['food_dispenser'].accepts(0.44)
    assert not specs['cow_enclosure'].accepts(0.45)


def test_a_wide_landmark_is_trusted_less_per_view():
    """An enclosure's box centroid wanders; a dispenser's does not."""
    specs = load_landmark_specs(CONFIG)
    assert (specs['cow_enclosure'].bearing_sigma_rad
            > specs['food_dispenser'].bearing_sigma_rad)


# -------------------------------------------------------------- intrinsics


def test_a_zeroed_k_falls_back_to_the_field_of_view():
    """v4l2_camera publishes a zeroed K until it is given a calibration."""
    intrinsics = resolve_intrinsics(WIDTH, HEIGHT, [0.0] * 9, 70.4)
    assert intrinsics.source == 'fov_fallback'
    expected = (WIDTH / 2) / math.tan(math.radians(70.4) / 2)
    assert intrinsics.fx == pytest.approx(expected)
    assert (intrinsics.cx, intrinsics.cy) == pytest.approx((320.0, 240.0))


def test_a_calibration_is_preferred():
    intrinsics = resolve_intrinsics(
        WIDTH, HEIGHT, [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1], 70.4)
    assert intrinsics.source == 'camera_info'
    assert (intrinsics.fx, intrinsics.cx) == pytest.approx((600.0, 320.0))


def test_a_missing_camera_info_still_yields_intrinsics():
    assert resolve_intrinsics(WIDTH, HEIGHT, None, 70.4).fx > 0.0


def test_the_principal_point_looks_straight_ahead():
    intrinsics = resolve_intrinsics(
        WIDTH, HEIGHT, [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1], 70.4)
    assert intrinsics.bearing(320.0, 240.0) == pytest.approx((0.0, 0.0, 1.0))


def test_a_pixel_right_of_centre_bears_right_and_stays_a_unit_vector():
    intrinsics = resolve_intrinsics(
        WIDTH, HEIGHT, [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1], 70.4)
    x, y, z = intrinsics.bearing(420.0, 240.0)
    assert x > 0.0 and y == pytest.approx(0.0) and z > 0.0
    assert math.hypot(x, math.hypot(y, z)) == pytest.approx(1.0)
    # 100 px at fx 600 is atan(1/6) off axis.
    assert math.atan2(x, z) == pytest.approx(math.atan2(100.0, 600.0))


# -------------------------------------------------------------------- boxes


def test_a_pixel_box_normalises():
    assert normalise_box([64, 48, 128, 96], WIDTH, HEIGHT) == pytest.approx(
        (0.1, 0.1, 0.2, 0.2))


def test_an_inverted_box_is_ordered_not_refused():
    assert normalise_box([128, 96, 64, 48], WIDTH, HEIGHT) == pytest.approx(
        (0.1, 0.1, 0.2, 0.2))


def test_a_box_outside_the_image_clamps():
    assert normalise_box([-20, -20, WIDTH + 50, HEIGHT + 50],
                         WIDTH, HEIGHT) == pytest.approx((0.0, 0.0, 1.0, 1.0))


def test_a_zero_area_box_is_refused():
    """A widened box would produce a confident bearing out of nothing."""
    with pytest.raises(ValueError, match='zero area'):
        normalise_box([100, 100, 100, 200], WIDTH, HEIGHT)
    with pytest.raises(ValueError):
        normalise_box([float('nan'), 0, 10, 10], WIDTH, HEIGHT)


def test_a_clipped_box_is_detected():
    """Its centroid is pulled inward, so its bearing is biased."""
    assert touches_image_edge((0.0, 0.2, 0.5, 0.8))
    assert touches_image_edge((0.2, 0.2, 1.0, 0.8))
    assert not touches_image_edge((0.2, 0.2, 0.8, 0.8))


def test_the_box_centre_is_in_pixels():
    detection = LandmarkDetection(
        landmark='food_dispenser', model_class='food_dispenser',
        confidence=0.9, box_norm=(0.25, 0.25, 0.75, 0.75))
    assert detection.centre_px(WIDTH, HEIGHT) == pytest.approx((320.0, 240.0))


# -------------------------------------------------------------------- depth


def test_millimetre_depth_becomes_metres():
    grid = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    decoded = depth_image_to_metres(
        grid.tobytes(), HEIGHT, WIDTH, WIDTH * 2, '16UC1')
    assert decoded[240, 320] == pytest.approx(2.0)


def test_float_depth_is_already_metres():
    grid = np.full((HEIGHT, WIDTH), 1.25, dtype=np.float32)
    decoded = depth_image_to_metres(
        grid.tobytes(), HEIGHT, WIDTH, WIDTH * 4, '32FC1')
    assert decoded[240, 320] == pytest.approx(1.25)


def test_a_padded_row_stride_is_honoured():
    """A driver may pad rows; ignoring the stride shears the whole image."""
    padded = np.zeros((HEIGHT, WIDTH + 8), dtype=np.uint16)
    padded[:, :WIDTH] = 1500
    decoded = depth_image_to_metres(
        padded.tobytes(), HEIGHT, WIDTH, (WIDTH + 8) * 2, '16UC1')
    assert decoded.shape == (HEIGHT, WIDTH)
    assert decoded[0, WIDTH - 1] == pytest.approx(1.5)


def test_an_unknown_depth_encoding_is_refused():
    with pytest.raises(ValueError, match='unsupported depth encoding'):
        depth_image_to_metres(b'', 1, 1, 1, '8UC1')


def test_only_the_middle_of_the_box_is_sampled():
    """The box edges straddle whatever is behind the object.

    The object here occupies only the middle quarter of the box, the rest of
    which sees a wall 3 m back. Sampling the whole box would return the wall,
    because the wall is most of the box.
    """
    box = (240 / WIDTH, 180 / HEIGHT, 400 / WIDTH, 300 / HEIGHT)
    grid = np.full((HEIGHT, WIDTH), 3.0, dtype=np.float32)
    # The shrunk patch of that box, at the default shrink of 0.5.
    grid[210:270, 280:360] = 1.0
    range_m, _, _ = depth_in_box(grid, box)
    assert range_m == pytest.approx(1.0)
    # And with no shrink, the wall wins, which is the behaviour being avoided.
    wall, _, _ = depth_in_box(grid, box, shrink=1.0)
    assert wall == pytest.approx(3.0)


def test_depth_outside_the_window_is_rejected():
    grid = np.full((HEIGHT, WIDTH), 9.0, dtype=np.float32)
    range_m, _, pixels = depth_in_box(grid, (0.4, 0.4, 0.6, 0.6))
    assert range_m is None and pixels == 0


def test_a_split_depth_patch_reports_itself_as_uncertain():
    """Half object, half far wall: the median is fine, the sigma must not be."""
    tight = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    split = tight.copy()
    split[:, WIDTH // 2:] = 2.8
    _, tight_sigma, _ = depth_in_box(tight, (0.3, 0.3, 0.7, 0.7))
    _, split_sigma, _ = depth_in_box(split, (0.3, 0.3, 0.7, 0.7))
    assert split_sigma > tight_sigma * 5


def test_a_flat_patch_still_has_a_floored_sigma():
    """A zero sigma would give that one view infinite weight in the fusion."""
    grid = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    _, sigma, _ = depth_in_box(grid, (0.3, 0.3, 0.7, 0.7))
    assert sigma >= 0.02


# ------------------------------------------------------------ colour decode


def test_rgb_is_reordered_to_bgr():
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frame[..., 0] = 10   # R
    frame[..., 2] = 30   # B
    decoded = color_image_to_bgr(frame.tobytes(), 4, 4, 12, 'rgb8')
    assert tuple(decoded[0, 0]) == (30, 0, 10)


def test_bgr_is_left_alone():
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frame[..., 0] = 30
    decoded = color_image_to_bgr(frame.tobytes(), 4, 4, 12, 'bgr8')
    assert tuple(decoded[0, 0]) == (30, 0, 0)


def test_mono_is_expanded_to_three_channels():
    frame = np.full((4, 4, 1), 77, dtype=np.uint8)
    decoded = color_image_to_bgr(frame.tobytes(), 4, 4, 4, 'mono8')
    assert decoded.shape == (4, 4, 3)
    assert tuple(decoded[0, 0]) == (77, 77, 77)


def test_an_unknown_colour_encoding_is_refused():
    with pytest.raises(ValueError, match='unsupported colour encoding'):
        color_image_to_bgr(b'', 1, 1, 1, 'yuyv')


# ----------------------------------------------------------- the wire format


def _payload(**overrides):
    intrinsics = resolve_intrinsics(
        WIDTH, HEIGHT, [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1], 70.4)
    document = build_payload(
        stamp_sec=12, stamp_nanosec=345,
        frame_id='webcam_optical_frame',
        width=WIDTH, height=HEIGHT, intrinsics=intrinsics,
        detections=[
            LandmarkDetection(
                landmark='cow_enclosure', model_class='cow-enclosure',
                confidence=0.6, box_norm=(0.1, 0.1, 0.4, 0.4)),
            LandmarkDetection(
                landmark='food_dispenser', model_class='food_dispenser',
                confidence=0.9, box_norm=(0.4, 0.4, 0.6, 0.7),
                range_m=1.84, range_sigma_m=0.07, range_source='depth'),
        ],
        inference_ms=12.5, model={'backend': 'ultralytics'})
    document.update(overrides)
    return document


def test_a_payload_round_trips():
    frame = parse_payload(json.dumps(_payload()))
    assert frame.frame_id == 'webcam_optical_frame'
    assert (frame.stamp_sec, frame.stamp_nanosec) == (12, 345)
    assert (frame.width, frame.height) == (WIDTH, HEIGHT)
    assert frame.intrinsics.fx == pytest.approx(600.0)
    # Highest confidence first, so a consumer taking one gets the best.
    assert [d.landmark for d in frame.detections] == [
        'food_dispenser', 'cow_enclosure']
    assert frame.detections[0].range_m == pytest.approx(1.84)
    assert frame.detections[0].range_source == 'depth'
    assert frame.detections[1].range_m is None


def test_a_payload_is_small_enough_to_stream_over_the_arena_wifi():
    """The point of the whole split: bytes, not a VGA frame."""
    assert len(json.dumps(_payload())) < 1000


def test_a_payload_from_the_wrong_schema_is_refused():
    with pytest.raises(ValueError, match='detection schema'):
        parse_payload(json.dumps(_payload(schema_version=99)))


def test_a_payload_without_a_frame_is_refused():
    """A bearing with no frame cannot be placed, and must not be guessed."""
    with pytest.raises(ValueError, match='no frame_id'):
        parse_payload(json.dumps(_payload(frame_id='')))


def test_a_payload_with_unusable_intrinsics_is_refused():
    with pytest.raises(ValueError, match='focal length'):
        parse_payload(json.dumps(_payload(
            intrinsics={'fx': 0.0, 'fy': 0.0, 'cx': 1.0, 'cy': 1.0})))


def test_a_payload_with_a_degenerate_box_is_refused():
    with pytest.raises(ValueError, match='zero area'):
        parse_payload(json.dumps(_payload(detections=[{
            'landmark': 'food_dispenser', 'confidence': 0.9,
            'box_norm': [0.5, 0.5, 0.5, 0.9]}])))


def test_a_payload_with_an_impossible_confidence_is_refused():
    with pytest.raises(ValueError, match='confidence'):
        parse_payload(json.dumps(_payload(detections=[{
            'landmark': 'food_dispenser', 'confidence': 3.0,
            'box_norm': [0.4, 0.4, 0.6, 0.6]}])))


def test_a_frame_with_no_detections_is_valid():
    frame = parse_payload(json.dumps(_payload(detections=[])))
    assert frame.detections == ()


# ----------------------------------------------------------------- backends


def test_the_registry_offers_the_backends():
    assert set(available_backends()) == {
        'ultralytics', 'onnx', 'stub', 'claude', 'disabled'}


def test_disabled_backend_reports_no_objects_without_weights():
    backend = create_backend('disabled', {})
    backend.load()
    assert backend.detect(np.zeros((12, 16, 3), dtype=np.uint8)) == []
    assert backend.describe()['backend'] == 'disabled'


def test_an_unknown_backend_names_the_ones_that_exist():
    with pytest.raises(ValueError, match='unknown backend'):
        create_backend('yolov99', {})


def test_the_stub_backend_scales_its_box_to_the_image():
    backend = create_backend('stub', {'stub_detections': json.dumps(
        [{'class': 'food_dispenser', 'box': [0.25, 0.25, 0.75, 0.75]}])})
    backend.load()
    detections = backend.detect(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))
    assert len(detections) == 1
    assert detections[0].box_xyxy_px == pytest.approx((160.0, 120.0, 480.0, 360.0))
    assert detections[0].model_class == 'food_dispenser'


def test_the_stub_backend_refuses_an_unusable_specification():
    with pytest.raises(ValueError):
        create_backend('stub', {'stub_detections': ''}).load()
    with pytest.raises(ValueError, match='four values in 0-1'):
        create_backend('stub', {'stub_detections': json.dumps(
            [{'class': 'x', 'box': [0.1, 0.1, 2.0, 0.5]}])}).load()


def test_a_missing_model_file_is_reported_as_such():
    """The node keeps running on this, so the message has to be the clue."""
    for name in ('ultralytics', 'onnx'):
        with pytest.raises(FileNotFoundError):
            create_backend(name, {'model_path': '/nonexistent/model.pt'}).load()


def test_the_onnx_backend_insists_on_class_names():
    """An ONNX graph carries no labels, and guessing them would mislabel."""
    backend = create_backend('onnx', {'model_path': __file__})
    with pytest.raises(RuntimeError, match='class_names'):
        backend.load()


def test_a_backend_describes_itself_for_the_payload():
    backend = create_backend('stub', {'stub_detections': json.dumps(
        [{'class': 'food_dispenser', 'box': [0.1, 0.1, 0.2, 0.2]}])})
    backend.load()
    assert backend.describe()['backend'] == 'stub'
    assert backend.class_names == ('food_dispenser',)


def test_the_stub_backend_can_be_driven_without_any_json():
    """The form that survives being typed through nested ssh quoting."""
    backend = create_backend('stub', {
        'stub_landmark': 'food_dispenser', 'stub_box_fraction': 0.2})
    backend.load()
    detections = backend.detect(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))
    assert len(detections) == 1
    assert detections[0].model_class == 'food_dispenser'
    # Centred, covering a fifth of the frame in each axis.
    assert detections[0].box_xyxy_px == pytest.approx(
        (0.4 * WIDTH, 0.4 * HEIGHT, 0.6 * WIDTH, 0.6 * HEIGHT))


def test_explicit_stub_detections_win_over_the_simple_form():
    backend = create_backend('stub', {
        'stub_landmark': 'food_dispenser',
        'stub_detections': json.dumps(
            [{'class': 'cow_enclosure', 'box': [0.1, 0.1, 0.3, 0.3]}])})
    backend.load()
    assert backend.detect(
        np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))[0].model_class == \
        'cow_enclosure'


def test_an_impossible_stub_box_fraction_is_refused():
    for fraction in (0.0, 1.5, -0.2):
        with pytest.raises(ValueError, match='stub_box_fraction'):
            create_backend('stub', {
                'stub_landmark': 'food_dispenser',
                'stub_box_fraction': fraction}).load()


def test_the_stub_still_needs_to_be_told_something():
    with pytest.raises(ValueError, match='stub_landmark'):
        create_backend('stub', {}).load()
