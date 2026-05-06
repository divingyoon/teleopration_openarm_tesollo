"""
Forward kinematics for openarm_tesollo_sensor.

Kinematic chain extracted from:
  /home/usr/rl_ws/hdgp/assets/openarm_tesollo_sensor/openarm_tesollo_sensor.urdf

EEF definitions (matching PourMimicEnv):
  right arm -> rl_dg_palm        (body used by FabricsActionTerm)
  left arm  -> openarm_left_hand_tcp
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# Primitive transform builders
# ---------------------------------------------------------------------------

def _rpy_mat(rpy) -> np.ndarray:
    return Rotation.from_euler("xyz", rpy).as_matrix()


def _fixed(xyz, rpy=None) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = xyz
    if rpy is not None:
        T[:3, :3] = _rpy_mat(rpy)
    return T


def _revolute(axis, angle: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(axis, dtype=np.float64) * angle).as_matrix()
    return T


# ---------------------------------------------------------------------------
# Right arm: world -> openarm_right_link0..7 -> rl_dg_palm
# ---------------------------------------------------------------------------

_T_W_RBASE = _fixed([0.0, -0.031, 0.698], [np.pi / 2, 0.0, 0.0])

_RIGHT_JOINTS = [
    (_fixed([0.000,    0.000,  0.0625]),               [0.0,  0.0,  1.0]),   # j1
    (_fixed([-0.0301,  0.000,  0.0600], [np.pi/2, 0, 0]), [-1.0,  0.0,  0.0]),   # j2
    (_fixed([0.0301,   0.000,  0.0663]),               [0.0,  0.0,  1.0]),   # j3
    (_fixed([0.000,    0.0315, 0.1537]),               [0.0,  1.0,  0.0]),   # j4
    (_fixed([0.000,   -0.0315, 0.0955]),               [0.0,  0.0,  1.0]),   # j5
    (_fixed([0.0375,   0.000,  0.1205]),               [1.0,  0.0,  0.0]),   # j6
    (_fixed([-0.0375,  0.000,  0.0000]),               [0.0,  1.0,  0.0]),   # j7
]

# link7 -> right_base_link -> rl_dg_mount -> rl_dg_base -> rl_dg_palm (all fixed)
_T_RLINK7_TO_PALM = (
    _fixed([0.0, 0.0, 0.0596], [0.0, 0.0, -np.pi / 2])
    @ _fixed([0.0, 0.0, 0.000])
    @ _fixed([0.0, 0.0, 0.004])
    @ _fixed([0.0, 0.0, 0.0698])
)

# ---------------------------------------------------------------------------
# Left arm: world -> openarm_left_link0..7 -> openarm_left_hand_tcp
# ---------------------------------------------------------------------------

_T_W_LBASE = _fixed([0.0, 0.031, 0.698], [-np.pi / 2, 0.0, 0.0])

_LEFT_JOINTS = [
    (_fixed([0.000,    0.000,  0.0625]),                [0.0,  0.0,  1.0]),   # j1
    (_fixed([-0.0301,  0.000,  0.0600], [-np.pi/2, 0, 0]), [-1.0,  0.0,  0.0]),   # j2
    (_fixed([0.0301,   0.000,  0.0663]),                [0.0,  0.0,  1.0]),   # j3
    (_fixed([0.000,    0.0315, 0.1537]),                [0.0,  1.0,  0.0]),   # j4
    (_fixed([0.000,   -0.0315, 0.0955]),                [0.0,  0.0,  1.0]),   # j5
    (_fixed([0.0375,   0.000,  0.1205]),                [1.0,  0.0,  0.0]),   # j6
    (_fixed([-0.0375,  0.000,  0.0000]),                [0.0, -1.0,  0.0]),   # j7
]

# link7 -> openarm_left_hand -> openarm_left_hand_tcp (both fixed)
_T_LLINK7_TO_TCP = _fixed([0.0, 0.0, 0.1001]) @ _fixed([0.0, 0.0, 0.08])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def fk_right_palm(q: np.ndarray) -> np.ndarray:
    """
    FK for right arm EEF (rl_dg_palm).

    Args:
        q: [7] joint angles (rad), openarm_right_joint1..7.

    Returns:
        [4, 4] homogeneous transform in world frame.
    """
    T = _T_W_RBASE.copy()
    for (T_off, axis), angle in zip(_RIGHT_JOINTS, q):
        T = T @ T_off @ _revolute(axis, angle)
    return T @ _T_RLINK7_TO_PALM


def fk_left_tcp(q: np.ndarray) -> np.ndarray:
    """
    FK for left arm EEF (openarm_left_hand_tcp).

    Args:
        q: [7] joint angles (rad), openarm_left_joint1..7.

    Returns:
        [4, 4] homogeneous transform in world frame.
    """
    T = _T_W_LBASE.copy()
    for (T_off, axis), angle in zip(_LEFT_JOINTS, q):
        T = T @ T_off @ _revolute(axis, angle)
    return T @ _T_LLINK7_TO_TCP


def fk_trajectory(q_traj: np.ndarray, arm: str = "right") -> np.ndarray:
    """
    Batch FK over a joint trajectory.

    Args:
        q_traj: [T, 7] joint angle trajectory (rad).
        arm: "right" -> rl_dg_palm, "left" -> openarm_left_hand_tcp.

    Returns:
        [T, 4, 4] EEF poses in world frame.
    """
    fn = fk_right_palm if arm == "right" else fk_left_tcp
    return np.stack([fn(q) for q in q_traj])


def pose_to_action_delta(T_curr: np.ndarray, T_next: np.ndarray) -> np.ndarray:
    """
    Compute 6D task-space action delta from consecutive EEF poses.

    Matches pour_mimic_math.target_pose_to_action (action[0:6]):
      [0:3] = world-frame XYZ translation delta (m)
      [3:6] = world-frame axis-angle rotation delta (rad),
              where R_next = R_delta @ R_curr (left multiplication)

    Args:
        T_curr: [4, 4] current EEF pose.
        T_next: [4, 4] next-step EEF pose.

    Returns:
        [6] action delta.
    """
    delta_xyz = T_next[:3, 3] - T_curr[:3, 3]
    R_delta = T_next[:3, :3] @ T_curr[:3, :3].T
    axis_angle = Rotation.from_matrix(R_delta).as_rotvec()
    return np.concatenate([delta_xyz, axis_angle])


def apply_action_delta(T_curr: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """
    Apply 6D action delta to recover target pose (inverse of pose_to_action_delta).

    Matches pour_mimic_math.action_to_target_pose.

    Args:
        T_curr: [4, 4] current EEF pose.
        delta: [6] action delta.

    Returns:
        [4, 4] predicted target pose.
    """
    T_target = T_curr.copy()
    T_target[:3, 3] = T_curr[:3, 3] + delta[:3]
    R_delta = Rotation.from_rotvec(delta[3:6]).as_matrix()
    T_target[:3, :3] = R_delta @ T_curr[:3, :3]
    return T_target
