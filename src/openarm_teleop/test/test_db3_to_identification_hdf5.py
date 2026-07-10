"""db3 변환기의 메시지/설정 해석 검증. rosbag 없이 돈다."""

import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from db3_to_identification_hdf5 import (  # noqa: E402
    IdentificationError,
    _to_named_sample,
    load_controller_joint_order,
)

BRINGUP_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml"
)

ARM_JOINTS = tuple(f"openarm_right_joint{i}" for i in range(1, 8))


@dataclass
class FakeMultiDOFCommand:
    dof_names: list
    values: list


@dataclass
class FakeJointState:
    name: list
    position: list
    velocity: list


@dataclass
class FakeFloat64MultiArray:
    data: list


def test_multi_dof_command_is_self_describing():
    message = FakeMultiDOFCommand(dof_names=["rj_dg_1_2"], values=[0.25])

    sample = _to_named_sample(7, message)

    assert sample.names == ("rj_dg_1_2",)
    assert sample.positions == (0.25,)


def test_joint_state_keeps_reported_velocity():
    message = FakeJointState(name=["rj_dg_1_2"], position=[0.2], velocity=[1.5])

    sample = _to_named_sample(7, message)

    assert sample.velocities == (1.5,)


def test_joint_state_with_empty_velocity_reports_none():
    message = FakeJointState(name=["rj_dg_1_2"], position=[0.2], velocity=[])

    sample = _to_named_sample(7, message)

    assert sample.velocities is None


def test_float64_multi_array_needs_an_explicit_joint_order():
    """팔 명령에는 이름이 없다. 순서가 어긋나면 관절이 뒤섞인 채 MSE가 계산된다."""
    message = FakeFloat64MultiArray(data=[0.1] * 7)

    with pytest.raises(IdentificationError, match="carries no joint names"):
        _to_named_sample(7, message)


def test_float64_multi_array_takes_the_given_joint_order():
    message = FakeFloat64MultiArray(data=[float(i) for i in range(7)])

    sample = _to_named_sample(7, message, ARM_JOINTS)

    assert sample.names == ARM_JOINTS
    assert sample.positions[0] == 0.0 and sample.positions[-1] == 6.0


def test_float64_multi_array_length_mismatch_is_rejected():
    message = FakeFloat64MultiArray(data=[0.1, 0.2])

    with pytest.raises(IdentificationError, match="2 values but 7 joint names"):
        _to_named_sample(7, message, ARM_JOINTS)


def test_unsupported_message_type_is_rejected():
    with pytest.raises(IdentificationError, match="unsupported message type"):
        _to_named_sample(7, object())


def test_controller_joint_order_is_read_from_the_real_bringup_config():
    if not BRINGUP_CONFIG.is_file():
        pytest.skip(f"bringup config not found: {BRINGUP_CONFIG}")

    joints = load_controller_joint_order(BRINGUP_CONFIG, "right_forward_position_controller")

    assert joints == ARM_JOINTS


def test_unknown_controller_name_is_rejected():
    if not BRINGUP_CONFIG.is_file():
        pytest.skip(f"bringup config not found: {BRINGUP_CONFIG}")

    with pytest.raises(IdentificationError, match="has no 'joints'"):
        load_controller_joint_order(BRINGUP_CONFIG, "no_such_controller")
