from pathlib import Path

from imdl_autonomy.map_from_world import generate_map, world_obstacles


def test_world_obstacles_reads_cylinders_and_walls():
    world = Path(__file__).parents[2] / 'imdl_simulation' / 'worlds' / 'arena_task2_a.world'

    obstacles = world_obstacles(world)

    names = {item[-1] for item in obstacles}
    assert 'arena_walls' in names
    assert 'obstacle_cylinder_1' in names
    assert 'obstacle_cylinder_2' in names


def test_generate_map_outputs_yaml_and_pgm(tmp_path):
    world = Path(__file__).parents[2] / 'imdl_simulation' / 'worlds' / 'arena_task2_a.world'

    yaml_path = generate_map(world, tmp_path, 0.05)

    assert yaml_path.exists()
    assert yaml_path.with_suffix('.pgm').exists()
    assert 'resolution: 0.0500' in yaml_path.read_text(encoding='ascii')
