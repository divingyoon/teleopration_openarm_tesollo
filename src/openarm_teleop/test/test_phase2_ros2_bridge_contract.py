"""Path contract checks for phase2 ROS2 bridge scripts."""

import os
from pathlib import Path

from pour_v1_mimic_contract import resolve_phase2_ros2_bridge_scripts_path


def test_default_prefers_usr_home_path():
    os.environ.pop("SIM2REAL_SCRIPTS_DIR", None)
    resolved = resolve_phase2_ros2_bridge_scripts_path()
    assert resolved.startswith("/home/usr/rl_ws/sim2real/scripts")


def test_env_override_wins(tmp_path):
    custom = tmp_path / "scripts"
    custom.mkdir()
    os.environ["SIM2REAL_SCRIPTS_DIR"] = str(custom)
    try:
        resolved = resolve_phase2_ros2_bridge_scripts_path()
        assert resolved == str(custom)
    finally:
        os.environ.pop("SIM2REAL_SCRIPTS_DIR", None)


def test_default_path_exists_in_current_environment():
    resolved = Path(resolve_phase2_ros2_bridge_scripts_path())
    assert resolved.exists()

