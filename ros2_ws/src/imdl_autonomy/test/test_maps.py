from pathlib import Path
import xml.etree.ElementTree as ET

from imdl_autonomy.map_from_world import generate_map, parse_pose
from imdl_autonomy.next_best_view import OccupancyGrid
import numpy as np


ROOT = Path(__file__).parents[2]
WORLD_DIR = ROOT / 'imdl_simulation' / 'worlds'
MAP_DIR = Path(__file__).parents[1] / 'maps'


def test_all_ai_maps_match_world_static_geometry_without_blocks(tmp_path):
    for task_number in range(1, 6):
        for variant in ('a', 'b'):
            stem = f'arena_task{task_number}_{variant}'
            generated_yaml = generate_map(
                WORLD_DIR / f'{stem}.world',
                tmp_path,
                0.02,
                exclude_model_suffixes=('_block',),
            )
            generated = OccupancyGrid.from_yaml(generated_yaml)
            repository = OccupancyGrid.from_yaml(MAP_DIR / f'{stem}.yaml')

            assert generated.resolution == repository.resolution
            assert generated.origin_x == repository.origin_x
            assert generated.origin_y == repository.origin_y
            np.testing.assert_array_equal(
                generated.occupied,
                repository.occupied,
                err_msg=f'{stem} map differs from its Gazebo static geometry',
            )


def test_unknown_object_centers_are_free_in_ai_maps():
    for world_path in sorted(WORLD_DIR.glob('arena_task[345]_[ab].world')):
        grid = OccupancyGrid.from_yaml(MAP_DIR / f'{world_path.stem}.yaml')
        root = ET.parse(world_path).getroot()
        for model in root.findall('.//model'):
            if not model.get('name', '').endswith('_block'):
                continue
            x, y, *_ = parse_pose(model.findtext('pose'))
            cell = grid.world_to_cell(x, y)
            assert cell is not None
            assert not grid.occupied[cell], (
                f'{world_path.stem} leaks {model.get("name")} into autonomy map'
            )


def test_canonical_unknown_map_contains_walls_but_no_cylinders():
    unknown = OccupancyGrid.from_yaml(MAP_DIR / 'arena_unknown.yaml')
    walls_only = OccupancyGrid.from_yaml(MAP_DIR / 'arena_task1_a.yaml')

    np.testing.assert_array_equal(unknown.occupied, walls_only.occupied)
    assert np.count_nonzero(unknown.occupied) > 0
    for world_path in sorted(WORLD_DIR.glob('arena_task[2345]_[ab].world')):
        root = ET.parse(world_path).getroot()
        for model in root.findall('.//model'):
            if not model.get('name', '').startswith('obstacle_cylinder'):
                continue
            x, y, *_ = parse_pose(model.findtext('pose'))
            cell = unknown.world_to_cell(x, y)
            assert cell is not None
            assert not unknown.occupied[cell], (
                f'{world_path.stem} cylinder leaked into arena_unknown map'
            )
