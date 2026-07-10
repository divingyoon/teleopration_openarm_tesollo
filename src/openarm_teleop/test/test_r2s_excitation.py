"""excitation plan의 순수 부분 검증. ROS/하드웨어 불필요.

이 노드가 실물에 그대로 나가므로, 관절 한계를 넘지 않는다는 것이 가장 중요한 성질이다.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

# excitation 시퀀스의 정의는 hdgp에만 있다. sim과 실물이 같은 시퀀스를 쓰기 위함이다.
HDGP_SCRIPTS = Path("/home/user/rl_ws/hdgp/scripts")
if not (HDGP_SCRIPTS / "r2s_autotune" / "excitation.py").is_file():
    pytest.skip("hdgp r2s_autotune not available", allow_module_level=True)
sys.path.insert(0, str(HDGP_SCRIPTS))

pytest.importorskip("yaml")

from r2s_autotune.excitation import ExcitationSpec  # noqa: E402

from r2s_excitation import (  # noqa: E402
    ARM_JOINTS,
    ASSET_DIR,
    HAND_JOINTS,
    TARGETS,
    build_plan,
    load_joint_limits,
)


@pytest.fixture(scope="module")
def limits():
    return load_joint_limits(HAND_JOINTS, ASSET_DIR)


@pytest.fixture(scope="module")
def plan():
    return build_plan(TARGETS["hand"], ExcitationSpec(), asset_dir=ASSET_DIR)


def test_driver_joint_order_is_the_full_twenty_dof_hand():
    assert len(HAND_JOINTS) == 20
    assert HAND_JOINTS[0] == "rj_dg_1_1"
    assert HAND_JOINTS[-1] == "rj_dg_5_4"


def test_limits_resolve_for_every_driver_joint(limits):
    lower, upper = limits

    assert lower.shape == upper.shape == (20,)
    assert np.all(lower <= upper)


def test_unknown_joint_name_is_rejected():
    with pytest.raises(KeyError, match="no limit found"):
        load_joint_limits(("rj_dg_9_9",), ASSET_DIR)


def test_only_curl_joints_are_excited(plan):
    assert plan.excited == ("rj_dg_1_2", "rj_dg_2_2", "rj_dg_3_2", "rj_dg_4_2", "rj_dg_5_2")


def test_every_command_stays_inside_the_joint_limits(plan, limits):
    """실물에 나가는 명령이다. 한계를 넘으면 하드웨어가 다친다."""
    lower, upper = limits

    assert np.all(plan.q_cmd >= lower[None, :] - 1e-9)
    assert np.all(plan.q_cmd <= upper[None, :] + 1e-9)


def test_non_excited_joints_are_held_constant(plan):
    held = [i for i, n in enumerate(HAND_JOINTS) if n not in plan.excited]

    assert np.allclose(np.ptp(plan.q_cmd[:, held], axis=0), 0.0)


def test_rest_pose_is_off_the_joint_limit_for_curl_joints(plan, limits):
    """default(0)는 index/middle/ring curl의 하한과 겹친다. 그 자리면 관절이 한계를 뚫는다."""
    lower, _ = limits

    for name in plan.excited:
        index = HAND_JOINTS.index(name)
        assert abs(plan.q_cmd[0, index] - lower[index]) > 0.04


def test_excited_joints_actually_move(plan):
    for name in plan.excited:
        index = HAND_JOINTS.index(name)
        assert float(np.ptp(plan.q_cmd[:, index])) > 0.1


def test_amplitude_scale_shrinks_the_commanded_range(plan):
    base = ExcitationSpec()
    small = build_plan(
        TARGETS["hand"],
        ExcitationSpec(step_rad=base.step_rad * 0.3, sine_amp_rad=base.sine_amp_rad * 0.3),
        asset_dir=ASSET_DIR,
    )

    index = HAND_JOINTS.index("rj_dg_2_2")
    assert np.ptp(small.q_cmd[:, index]) < np.ptp(plan.q_cmd[:, index])


def test_arm_plan_excites_only_the_requested_gain_group():
    """팔은 group 단위로 흔든다. 7축 동시 여기는 중력 커플링으로 식별성이 떨어진다."""
    plan = build_plan(TARGETS["arm"], ExcitationSpec(), group="wrist", asset_dir=ASSET_DIR)

    assert plan.excited == ("openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7")
    held = [i for i, n in enumerate(ARM_JOINTS) if n not in plan.excited]
    assert np.allclose(np.ptp(plan.q_cmd[:, held], axis=0), 0.0)


def test_arm_commands_stay_inside_joint_limits():
    plan = build_plan(TARGETS["arm"], ExcitationSpec(), group="proximal", asset_dir=ASSET_DIR)
    lower, upper = load_joint_limits(ARM_JOINTS, ASSET_DIR)

    assert np.all(plan.q_cmd >= lower[None, :] - 1e-9)
    assert np.all(plan.q_cmd <= upper[None, :] + 1e-9)


def test_unknown_arm_group_is_rejected():
    with pytest.raises(ValueError, match="unknown arm group"):
        build_plan(TARGETS["arm"], ExcitationSpec(), group="shoulder", asset_dir=ASSET_DIR)


def test_plan_matches_the_publish_rate(plan):
    assert plan.q_cmd.shape == (plan.time.shape[0], 20)
    np.testing.assert_allclose(np.diff(plan.time), 0.01)
