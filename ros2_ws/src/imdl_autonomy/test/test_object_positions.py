"""Map geometry and persistence regression cases (no ROS required)."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from imdl_autonomy.object_positions import ObjectPositionStore, project_box


def test_projection_uses_optical_depth_and_camera_offset():
    k = SimpleNamespace(fx=100, fy=100, cx=100, cy=100)
    # Optical z -> rover x, optical x -> rover -y, optical y -> rover -z.
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    point = project_box((0.5, 0.4, 0.7, 0.6), 200, 200, k, 2.0,
                        rotation, (1.1, 3.0, 0.2))
    assert point == pytest.approx({'x': 3.1, 'y': 2.6, 'z': 0.2})


def entry(x, y, origin=None, stamp=None):
    origin = origin or (x - 2, y)
    return {'position_odom_m': {'x': x, 'y': y, 'z': 0.0},
            'camera_origin_odom_m': {'x': origin[0], 'y': origin[1], 'z': 0.0},
            'confidence': 0.9, 'fit_residual': 0.0, 'growth_ratio': 0.1,
            **({'source_image_stamp': {'sec': stamp, 'nanosec': 0},
                'camera_frame': 'camera_optical'} if stamp is not None else {})}


def test_two_objects_are_saved_together_and_follow_map_corrections(tmp_path):
    path = tmp_path / 'objects.json'
    store = ObjectPositionStore(str(path), map_id='arena_trial_1')
    store.save('food_dispenser', entry(2, 3))
    store.save('cow_enclosure', entry(4, 5))
    assert store.reproject(np.eye(3), (10, -1, 0))
    document = json.loads(path.read_text())
    assert document['frame_id'] == 'map'
    assert document['map_id'] == 'arena_trial_1'
    assert set(document['objects']) == {'food_dispenser', 'cow_enclosure'}
    assert document['objects']['food_dispenser']['position_map_m']['x'] == 12
    assert document['objects']['cow_enclosure']['position_map_m']['y'] == 4
    assert not store.reproject(np.eye(3), (10, -1, 0))


def test_existing_run_is_not_overwritten(tmp_path):
    path = tmp_path / 'objects.json'
    path.write_text('previous run')
    with pytest.raises(FileExistsError):
        ObjectPositionStore(str(path))
    assert path.read_text() == 'previous run'


def test_failed_atomic_write_keeps_previous_file_and_registry(tmp_path, monkeypatch):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    store.save('food_dispenser', entry(1, 2))
    previous = store.path.read_text()

    def fail_replace(*args):
        raise OSError('disk unavailable')

    monkeypatch.setattr('imdl_autonomy.object_positions.os.replace', fail_replace)
    with pytest.raises(OSError):
        store.save('cow_enclosure', entry(3, 4))
    assert store.path.read_text() == previous
    assert set(store.objects) == {'food_dispenser'}


def test_second_angle_refines_first_position_instead_of_replacing_it(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    first = store.save('food_dispenser', entry(2.4, 0, origin=(0, 0)))
    fused = store.save('food_dispenser', entry(2.0, 0, origin=(2, -2)))
    assert 2.0 < fused['position_odom_m']['x'] < first['position_odom_m']['x']
    assert fused['measurement_count'] == fused['used_measurement_count'] == 2
    assert fused['view_angle_spread_deg'] == pytest.approx(90)
    assert fused['sigma_major_m'] < first['sigma_major_m']
    assert fused['last_measurement_used']


def test_crossing_views_constrain_more_than_parallel_views(tmp_path):
    parallel = ObjectPositionStore(str(tmp_path / 'parallel.json'))
    crossing = ObjectPositionStore(str(tmp_path / 'crossing.json'))
    for store in (parallel, crossing):
        store.save('food_dispenser', entry(2, 0, origin=(0, 0)))
    same = parallel.save('food_dispenser', entry(2, 0, origin=(0, 0)))
    cross = crossing.save('food_dispenser', entry(2, 0, origin=(2, -2)))
    assert cross['sigma_major_m'] < same['sigma_major_m']


def test_conflicting_new_measurement_cannot_overwrite_existing_group(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    store.save('food_dispenser', entry(2, 0, origin=(0, 0)))
    before = store.save('food_dispenser', entry(2, 0, origin=(2, -2)))
    after = store.save('food_dispenser', entry(8, 8, origin=(8, 6)))
    assert after['position_odom_m'] == pytest.approx(before['position_odom_m'])
    assert after['measurement_count'] == 3
    assert after['used_measurement_count'] == 2
    assert after['rejected_measurement_count'] == 1
    assert not after['last_measurement_used']
    assert after['fusion_status'] == 'disagreement'


def test_agreeing_later_views_can_correct_bad_first_measurement(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    store.save('food_dispenser', entry(8, 0, origin=(-1, 0)))
    conflict = store.save('food_dispenser', entry(0, 0, origin=(-1, 0)))
    assert conflict['position_odom_m']['x'] == pytest.approx(8)
    corrected = store.save('food_dispenser', entry(0, 0, origin=(0, -1)))
    assert corrected['position_odom_m']['x'] == pytest.approx(0)
    assert corrected['used_measurement_count'] == 2
    assert not corrected['retained_measurements'][0]['used']


def test_duplicate_image_adds_no_confidence(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    measurement = entry(2, 3, stamp=10)
    first = store.save('food_dispenser', measurement)
    duplicate = store.save('food_dispenser', measurement)
    assert duplicate == first
    assert duplicate['measurement_count'] == 1


def test_history_preserves_other_angles_and_limits_repeated_views(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    store.save('food_dispenser', entry(2, 0, origin=(2, -2), stamp=1))
    for stamp in range(2, 15):
        result = store.save('food_dispenser', entry(2, 0, origin=(0, 0), stamp=stamp))
    assert result['measurement_count'] == 14
    assert result['retained_measurement_count'] == 4
    assert result['view_angle_spread_deg'] == pytest.approx(90)
    assert result['sigma_major_m'] >= store.fusion_config.position_sigma_floor_m


def test_map_rotation_also_rotates_fused_covariance(tmp_path):
    store = ObjectPositionStore(str(tmp_path / 'objects.json'))
    original = store.save('food_dispenser', entry(0, 0))
    old_covariance = original['covariance_xy_map_m2']
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert store.reproject(rotation, (0, 0, 0))
    new_covariance = store.objects['food_dispenser']['covariance_xy_map_m2']
    assert new_covariance[0] == pytest.approx(old_covariance[3])
    assert new_covariance[3] == pytest.approx(old_covariance[0])
