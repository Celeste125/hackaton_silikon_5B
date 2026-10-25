"""Contract checks on how landmark localisation is wired together.

These catch the failures that are silent at build time and expensive at the
arena: an executable that is not installed, a launch file that is not shipped,
a detector and a mapper reading different landmark configs, and an RViz
profile with nothing to show.
"""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from perception.landmark_detection import canonical_class, load_landmark_specs

AUTONOMY = Path(__file__).parents[1]
WORKSPACE = AUTONOMY.parent
PERCEPTION = WORKSPACE / 'perception'
CONFIG = PERCEPTION / 'config' / 'landmarks.yaml'


def test_both_nodes_are_installed_as_executables():
    detector = (PERCEPTION / 'setup.py').read_text(encoding='utf-8')
    mapper = (AUTONOMY / 'setup.py').read_text(encoding='utf-8')
    assert (
        'landmark_detector = perception.landmark_detector_node:main'
        in detector)
    assert 'landmark_mapper = imdl_autonomy.landmark_mapper:main' in mapper


def test_both_launch_files_are_shipped():
    assert (PERCEPTION / 'launch' / 'landmark_detector.launch.py').is_file()
    assert (AUTONOMY / 'launch' / 'landmark_mapping.launch.py').is_file()
    # setup.py installs launch/*.py for perception and launch/*.launch.py for
    # autonomy; both patterns cover these names, but check the autonomy one,
    # whose glob is the stricter of the two.
    assert "glob('launch/*.launch.py')" in (
        AUTONOMY / 'setup.py').read_text(encoding='utf-8')


def test_the_mapper_may_import_the_shared_wire_format():
    """It parses the detector's payload, so the dependency must be declared."""
    manifest = ET.parse(AUTONOMY / 'package.xml').getroot()
    dependencies = {
        element.text for element in manifest.iter()
        if element.tag.endswith('depend')
    }
    assert 'perception' in dependencies


def test_the_shipped_landmark_config_loads():
    specs = load_landmark_specs(
        yaml.safe_load(CONFIG.read_text(encoding='utf-8')))
    assert {'cow_enclosure'} <= set(specs)


def test_the_cow_model_classes_are_claimed():
    """The weights emit cow_head and cow_body; both must reach the mapper."""
    specs = load_landmark_specs(
        yaml.safe_load(CONFIG.read_text(encoding='utf-8')))
    claimed = {
        model_class
        for spec in specs.values() for model_class in spec.model_classes}
    assert {'cow_head', 'cow_body'} <= claimed


def test_the_shipped_config_is_installed_with_the_package():
    assert "glob('config/*.yaml')" in (
        PERCEPTION / 'setup.py').read_text(encoding='utf-8')


def test_both_nodes_default_to_the_same_config():
    """A detector and a mapper disagreeing about landmarks drop everything."""
    for node, name in (
        (PERCEPTION / 'perception' / 'landmark_detector_node.py', 'detector'),
        (AUTONOMY / 'imdl_autonomy' / 'landmark_mapper.py', 'mapper'),
    ):
        source = node.read_text(encoding='utf-8')
        assert "get_package_share_directory('perception')" in source, name
        assert "'landmarks.yaml'" in source, name


def test_the_config_class_names_are_already_canonical():
    """So what the file says is what a model's labels are matched against."""
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    for name, entry in document['landmarks'].items():
        assert canonical_class(name) == name
        for model_class in entry['model_classes']:
            assert canonical_class(model_class) == model_class, model_class


def test_a_wide_landmark_is_configured_as_less_trustworthy_per_view():
    """An enclosure's box centroid wanders, so each view must count for less
    than a compact object's would (2-3 deg, see landmarks.yaml)."""
    specs = load_landmark_specs(
        yaml.safe_load(CONFIG.read_text(encoding='utf-8')))
    assert specs['cow_enclosure'].bearing_sigma_rad > 0.087  # > 5 deg


def test_the_navigation_rviz_profile_shows_the_landmarks():
    profile = (
        WORKSPACE / 'imdl_simulation' / 'rviz' / 'real_create3.rviz'
    ).read_text(encoding='utf-8')
    assert '/landmarks/markers' in profile


def test_the_detector_keeps_images_off_the_network_by_default():
    """The debug JPEG is the one output that costs real bandwidth."""
    launch = (
        PERCEPTION / 'launch' / 'landmark_detector.launch.py'
    ).read_text(encoding='utf-8')
    assert "'publish_debug_image', default_value='false'" in launch


def test_the_detector_defaults_to_a_backend_that_needs_no_model():
    """So the chain can be brought up before any weights exist."""
    launch = (
        PERCEPTION / 'launch' / 'landmark_detector.launch.py'
    ).read_text(encoding='utf-8')
    assert "'backend', default_value='stub'" in launch


@pytest.mark.parametrize('backend', ['ultralytics', 'onnx', 'stub', 'claude'])
def test_every_backend_is_selectable_from_the_launch_file(backend):
    launch = (
        PERCEPTION / 'launch' / 'landmark_detector.launch.py'
    ).read_text(encoding='utf-8')
    assert f"'{backend}'" in launch


# --------------------------------------------------------------------------
# The location prior: the robot side, the training side, and the runbook.
# --------------------------------------------------------------------------

TRAINING = WORKSPACE.parents[1] / 'landmark_prior'


def test_the_training_side_exists_outside_the_ros_workspace():
    """It needs torch, which deliberately never goes on the robot."""
    assert (TRAINING / 'landmark_prior_net.py').is_file()
    assert (TRAINING / 'README.md').is_file()
    for script in (
        'build_landmark_prior_dataset.py',
        'train_landmark_prior.py',
        'inspect_landmark_prior_dataset.py',
    ):
        assert (TRAINING / 'scripts' / script).is_file(), script


def test_the_robot_never_imports_torch():
    """onnxruntime only, so the Jetson's JetPack build is not a constraint."""
    for module in (
        AUTONOMY / 'imdl_autonomy' / 'landmark_prior.py',
        AUTONOMY / 'imdl_autonomy' / 'landmark_mapper.py',
        AUTONOMY / 'imdl_autonomy' / 'landmark_estimation.py',
    ):
        source = module.read_text(encoding='utf-8')
        assert 'import torch' not in source, module.name


def test_the_geometry_has_one_definition_shared_by_both_sides():
    """A half-cell disagreement would train a quietly offset prior."""
    builder = (
        TRAINING / 'scripts' / 'build_landmark_prior_dataset.py'
    ).read_text(encoding='utf-8')
    assert 'from imdl_autonomy.landmark_prior import' in builder
    assert 'Window' in builder


def _imported_modules(path):
    """Top-level module names a file imports, from its AST.

    Parsed rather than grepped: these files describe the ROS message types
    they are deliberately independent of, and a substring match over the
    prose would fail on the documentation.
    """
    tree = ast.parse(path.read_text(encoding='utf-8'))
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
            modules.add(node.module.split('.')[0])
    return modules


ROS_ONLY = {
    'rclpy', 'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'std_msgs',
    'tf2_ros', 'tf2_geometry_msgs', 'visualization_msgs', 'nav2_msgs',
    'action_msgs', 'ament_index_python',
}


def test_the_prior_module_stays_free_of_ros():
    """So the training scripts can import it with nothing else installed."""
    imported = _imported_modules(
        AUTONOMY / 'imdl_autonomy' / 'landmark_prior.py')
    assert not (imported & ROS_ONLY), imported & ROS_ONLY


def test_the_prior_is_off_by_default():
    """A stale file or a foreign network must be opted into deliberately."""
    launch = (
        AUTONOMY / 'launch' / 'landmark_mapping.launch.py'
    ).read_text(encoding='utf-8')
    assert "'prior_source', default_value='none'" in launch
    assert "'dataset_dir', default_value=''" in launch


def test_every_prior_parameter_the_node_reads_is_a_launch_argument():
    """Otherwise the runbook documents a flag that silently does nothing."""
    mapper = (
        AUTONOMY / 'imdl_autonomy' / 'landmark_mapper.py'
    ).read_text(encoding='utf-8')
    launch = (
        AUTONOMY / 'launch' / 'landmark_mapping.launch.py'
    ).read_text(encoding='utf-8')
    for name in (
        'prior_source', 'prior_file', 'prior_file_sigma_m',
        'prior_network_path', 'prior_inflate', 'prior_min_sigma_m',
        'prior_update_interval_sec',
        'dataset_dir', 'dataset_interval_sec', 'base_frame',
    ):
        assert f"declare_parameter('{name}'" in mapper, name
        assert f"DeclareLaunchArgument(\n            '{name}'" in launch \
            or f"DeclareLaunchArgument('{name}'" in launch, name
        assert f"LaunchConfiguration('{name}')" in launch, name


def test_the_runbook_matches_the_scripts_it_documents():
    readme = (TRAINING / 'README.md').read_text(encoding='utf-8')
    for command in (
        'build_landmark_prior_dataset.py',
        'train_landmark_prior.py',
        'inspect_landmark_prior_dataset.py',
    ):
        assert command in readme, command
    # The dependency list has to include onnxscript: torch's exporter needs it
    # and fails only at the very end of a training run without it.
    assert 'onnxscript' in readme


def test_the_runbook_warns_about_the_case_where_a_prior_is_meaningless():
    """If the landmarks move between runs, there is nothing to predict."""
    readme = (TRAINING / 'README.md').read_text(encoding='utf-8')
    assert 'same place from run to run' in readme


# --------------------------------------------------------------------------
# Camera coverage: the second explorer, and the handover between them.
# --------------------------------------------------------------------------


def test_the_coverage_explorer_is_installed_and_launchable():
    setup = (AUTONOMY / 'setup.py').read_text(encoding='utf-8')
    assert 'imdl_autonomy.camera_coverage_explorer:main' in setup
    assert (AUTONOMY / 'launch' / 'camera_coverage.launch.py').is_file()


def test_the_two_explorers_hand_over_rather_than_fight_for_nav2():
    """Both drive through navigate_to_pose, so only one may hold it."""
    explorer = (
        AUTONOMY / 'imdl_autonomy' / 'frontier_explorer.py'
    ).read_text(encoding='utf-8')
    coverage = (
        AUTONOMY / 'imdl_autonomy' / 'camera_coverage_explorer.py'
    ).read_text(encoding='utf-8')

    # The frontier explorer announces the end of its sweep...
    assert '/frontier_explorer/done' in explorer
    # ...latched, so a coverage node that starts late still hears it.
    assert 'TRANSIENT_LOCAL' in explorer
    # ...and the coverage node waits for it by default.
    assert '/frontier_explorer/done' in coverage
    assert "'start_mode', 'wait_for_frontier'" in coverage


def test_the_done_signal_is_additive_to_the_ported_explorer():
    """frontier_explorer.py is kept diffable against upstream on purpose, so
    the handover may only add lines, never rewrite its logic."""
    explorer = (
        AUTONOMY / 'imdl_autonomy' / 'frontier_explorer.py'
    ).read_text(encoding='utf-8')
    assert 'stays diffable against upstream' in explorer
    assert 'Additive to the upstream logic' in explorer
    # The three places that end exploration all route through one method.
    assert explorer.count('self._done_pub.publish(Empty())') == 1


def test_coverage_counts_free_floor_rather_than_walls():
    """An object stands on floor the lidar reads as free -- that is the whole
    reason the camera pipeline exists -- so free cells are the right target."""
    module = (
        AUTONOMY / 'imdl_autonomy' / 'camera_coverage.py'
    ).read_text(encoding='utf-8')
    assert 'cannot see the dispenser' in module


def test_the_coverage_geometry_is_pure_enough_to_test():
    """The geometry must not drag ROS in, or none of it is checkable."""
    imported = _imported_modules(
        AUTONOMY / 'imdl_autonomy' / 'camera_coverage.py')
    assert not (imported & ROS_ONLY), imported & ROS_ONLY


def test_the_navigation_rviz_profile_shows_camera_coverage():
    profile = (
        WORKSPACE / 'imdl_simulation' / 'rviz' / 'real_create3.rviz'
    ).read_text(encoding='utf-8')
    assert '/camera_coverage' in profile


def test_the_launcher_runs_the_coverage_pass():
    launcher = (
        WORKSPACE.parents[1] / 'scripts' / 'imdl_tmux.sh'
    ).read_text(encoding='utf-8')
    assert 'camera_coverage.launch.py' in launcher
