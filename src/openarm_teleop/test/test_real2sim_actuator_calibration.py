import json

import h5py
import numpy as np
import pytest

from real2sim_actuator_calibration import (
    ActuatorGroupDefaults,
    CalibrationBuildConfig,
    compute_joint_metrics,
    estimate_group_calibration,
    load_hdf5_joint_pair,
    select_joint_pattern,
    write_calibration_json,
)


def test_compute_joint_metrics_reports_tracking_error_and_lag():
    command = np.zeros((20, 1), dtype=np.float32)
    command[5:, 0] = 1.0
    measured = np.zeros((20, 1), dtype=np.float32)
    measured[7:, 0] = 0.8
    timestamps_ns = np.arange(20, dtype=np.int64) * 10_000_000

    metrics = compute_joint_metrics(
        command=command,
        measured=measured,
        timestamps_ns=timestamps_ns,
        joint_names=["joint_a"],
    )

    assert metrics["joint_a"]["rmse_rad"] > 0.0
    assert metrics["joint_a"]["steady_state_error_rad"] == pytest.approx(0.2)
    assert metrics["joint_a"]["lag_sec"] == 0.02
    assert metrics["joint_a"]["max_velocity_rad_s"] > 0.0


def test_estimate_group_calibration_uses_defaults_and_real_response_metrics():
    command = np.zeros((40, 2), dtype=np.float32)
    command[10:, :] = 1.0
    measured = np.zeros((40, 2), dtype=np.float32)
    measured[13:, 0] = 0.7
    measured[12:, 1] = 0.9
    timestamps_ns = np.arange(40, dtype=np.int64) * 10_000_000

    defaults = ActuatorGroupDefaults(
        stiffness=30.0,
        damping=5.0,
        effort_limit=7.5,
        velocity_limit=3.14,
        joint_friction=0.0,
    )
    calibration = estimate_group_calibration(
        group_name="tesollo_hand_curl",
        command=command,
        measured=measured,
        timestamps_ns=timestamps_ns,
        joint_names=["j1", "j2"],
        defaults=defaults,
    )

    assert calibration["stiffness"] >= defaults.stiffness
    assert calibration["damping"] >= defaults.damping
    assert calibration["joint_friction"] > 0.0
    assert calibration["delay_steps"] >= 2
    assert set(calibration["joint_metrics"]) == {"j1", "j2"}


def test_load_hdf5_joint_pair_and_write_calibration_json(tmp_path):
    dataset = tmp_path / "demo.hdf5"
    with h5py.File(dataset, "w") as h5:
        demo = h5.create_group("data/demo_0")
        demo.create_dataset("timestamps_ns", data=np.arange(5, dtype=np.int64))
        obs = demo.create_group("obs")
        cmd = obs.create_dataset("cmd", data=np.ones((5, 2), dtype=np.float32))
        cmd.attrs["joint_names"] = json.dumps(["j1", "j2"])
        obs.create_dataset("measured", data=np.zeros((5, 2), dtype=np.float32))

    pair = load_hdf5_joint_pair(
        dataset_path=dataset,
        demo_key="demo_0",
        command_dataset="obs/cmd",
        measured_dataset="obs/measured",
    )
    assert pair.joint_names == ["j1", "j2"]
    assert pair.command.shape == (5, 2)
    assert pair.measured.shape == (5, 2)

    output = tmp_path / "calibration.json"
    config = CalibrationBuildConfig(
        robot_asset="openarm_tesollo_sensor",
        source_dataset=str(dataset),
        groups={"example": {"stiffness": 1.0, "damping": 2.0}},
    )
    write_calibration_json(output, config)
    saved = json.loads(output.read_text())
    assert saved["schema_version"] == 1
    assert saved["robot_asset"] == "openarm_tesollo_sensor"
    assert saved["groups"]["example"]["damping"] == 2.0


def test_select_joint_pattern_filters_command_and_measured_columns(tmp_path):
    dataset = tmp_path / "demo.hdf5"
    with h5py.File(dataset, "w") as h5:
        demo = h5.create_group("data/demo_0")
        demo.create_dataset("timestamps_ns", data=np.arange(3, dtype=np.int64))
        obs = demo.create_group("obs")
        cmd = obs.create_dataset("cmd", data=np.arange(12, dtype=np.float32).reshape(3, 4))
        cmd.attrs["joint_names"] = json.dumps(["rj_dg_1_1", "rj_dg_1_2", "rj_dg_2_1", "rj_dg_2_2"])
        obs.create_dataset("measured", data=np.arange(100, 112, dtype=np.float32).reshape(3, 4))

    pair = load_hdf5_joint_pair(dataset, "demo_0", "obs/cmd", "obs/measured")
    selected = select_joint_pattern(pair, r"rj_dg_[1-5]_2")

    assert selected.joint_names == ["rj_dg_1_2", "rj_dg_2_2"]
    assert selected.command.shape == (3, 2)
    assert selected.command[0].tolist() == [1.0, 3.0]
