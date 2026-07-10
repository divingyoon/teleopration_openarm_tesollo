#!/usr/bin/env python3
"""OpenArm 팔 / Tesollo 손에 identification excitation을 인가하는 ROS2 노드.

`record_real2sim_identification_bag.sh`는 토픽을 녹화만 한다. 명령을 주는 주체가 없으면
정지한 로봇의 데이터가 쌓인다. 이 노드가 그 명령을 만든다.

sim replay와 **같은 시퀀스**를 써야 하므로 hdgp의 excitation 모듈을 그대로 import한다.
시퀀스 정의를 복제하면 언젠가 갈라지고, 그 순간 R2S 오차는 의미를 잃는다.

명령 경로 (둘 다 중간 보간이 없다):

    hand  /dg5f_right/rj_dg_pospid/reference        control_msgs/MultiDOFCommand   (dof_names 포함)
    arm   /right_forward_position_controller/commands  std_msgs/Float64MultiArray  (이름 없음)

`/isaacsim/*_cmd` 를 쓰면 안 된다. isaacsim_bridge가 위치 목표를 trajectory_time_sec 짜리
궤적으로 보간하므로, 그 경로로 수집하면 액추에이터가 아니라 브리지 필터를 식별하게 된다.
팔도 joint_trajectory_controller가 아니라 forward_position_controller로 띄운다.

  ros2 launch openarm_bringup openarm.bimanual.launch.py \
      robot_controller:=forward_position_controller

안전:
  - 첫 실행은 반드시 --dry-run 으로 궤적을 확인한다.
  - --amplitude-scale 로 진폭을 줄여 시작한다 (권장 0.3).
  - 현재 자세에서 시작 자세까지 --approach-sec 동안 선형 접근한 뒤에야 excitation을 시작한다.
  - 모든 명령은 URDF 관절 한계 안으로 clamp된다.
  - 팔은 한 group씩만 흔든다. 7축을 동시에 흔들면 중력 커플링으로 식별성이 떨어진다.

사용:
  python3 r2s_excitation.py --target hand --dry-run
  python3 r2s_excitation.py --target arm --excite wrist --dry-run
  ROS_DOMAIN_ID=126 python3 r2s_excitation.py --target hand --amplitude-scale 0.3
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

HDGP_SCRIPTS = Path("/home/user/rl_ws/hdgp/scripts")
sys.path.insert(0, str(HDGP_SCRIPTS))

from r2s_autotune.excitation import (  # noqa: E402
    ExcitationSpec,
    build_excitation,
    interior_neutral,
    is_saturated,
)

ASSET_DIR = Path("/home/user/rl_ws/hdgp/assets/robot/openarm_tesollo_sensor_rl")

# 드라이버가 기대하는 순서. 손은 MultiDOFCommand가 이름을 싣지만, 팔은 싣지 않는다.
HAND_JOINTS = tuple(f"rj_dg_{finger}_{link}" for finger in range(1, 6) for link in range(1, 5))
ARM_JOINTS = tuple(f"openarm_right_joint{i}" for i in range(1, 8))

# 팔 excitation group. 실물 MIT PD 게인 구조를 따른다 (kp 70 / 60 / 10).
ARM_GROUPS = {
    "proximal": (1, 2, 3),
    "elbow": (4,),
    "wrist": (5, 6, 7),
    "all": (1, 2, 3, 4, 5, 6, 7),
}


@dataclass(frozen=True)
class TargetSpec:
    name: str
    joints: tuple[str, ...]
    command_topic: str
    measured_topic: str
    message: str  # "multidof" | "float64"


TARGETS = {
    "hand": TargetSpec(
        name="hand",
        joints=HAND_JOINTS,
        command_topic="/dg5f_right/rj_dg_pospid/reference",
        measured_topic="/dg5f_right/joint_states",
        message="multidof",
    ),
    "arm": TargetSpec(
        name="arm",
        joints=ARM_JOINTS,
        command_topic="/right_forward_position_controller/commands",
        measured_topic="/joint_states",
        message="float64",
    ),
}


@dataclass(frozen=True)
class ExcitationPlan:
    """오프라인으로 다 만들어 두는 명령 시퀀스. 발행 중에는 계산하지 않는다."""

    target: TargetSpec
    time: np.ndarray  # [T]
    q_cmd: np.ndarray  # [T, J]
    excited: tuple[str, ...]

    @property
    def duration_sec(self) -> float:
        return float(self.time[-1])


def load_joint_limits(
    joint_names: tuple[str, ...],
    asset_dir: Path = ASSET_DIR,
) -> tuple[np.ndarray, np.ndarray]:
    """legacy 관절 이름에 대한 (lower, upper) 한계를 RL URDF에서 읽는다.

    sim이 쓰는 것과 같은 URDF를 본다. 실물 드라이버의 한계와 다르면 그건 asset 불일치이지
    excitation이 흡수할 문제가 아니다.
    """
    manifest = yaml.safe_load((asset_dir / f"{asset_dir.name}_manifest.yaml").read_text())
    legacy_to_canonical = manifest["source_to_canonical_joints"]

    tree = ET.parse(asset_dir / f"{asset_dir.name}.urdf")
    limits = {}
    for joint in tree.getroot().iter("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))

    lower, upper = [], []
    for name in joint_names:
        canonical = legacy_to_canonical.get(name)
        if canonical is None or canonical not in limits:
            raise KeyError(f"no limit found for '{name}' (canonical: {canonical})")
        lo, hi = limits[canonical]
        lower.append(lo)
        upper.append(hi)
    return np.array(lower), np.array(upper)


def excited_indices(target: TargetSpec, group: str) -> list[int]:
    """흔들 관절. 손은 curl(_2)만, 팔은 게인 group 단위로."""
    if target.name == "hand":
        return [i for i, n in enumerate(target.joints) if n.endswith("_2")]
    if group not in ARM_GROUPS:
        raise ValueError(f"unknown arm group '{group}'. 선택: {sorted(ARM_GROUPS)}")
    return [joint - 1 for joint in ARM_GROUPS[group]]


def build_plan(
    target: TargetSpec,
    spec: ExcitationSpec,
    group: str = "all",
    asset_dir: Path = ASSET_DIR,
) -> ExcitationPlan:
    """전체 관절 명령을 만든다. 대상 관절만 흔들고 나머지는 기준 자세로 붙든다."""
    lower, upper = load_joint_limits(target.joints, asset_dir)

    # sim의 rest_pose와 같은 규칙. default(0)는 손 curl 관절의 하한과 겹쳐 쓸 수 없다.
    rest = interior_neutral(np.zeros(len(target.joints)), lower, upper, spec)

    index = excited_indices(target, group)
    excited_names = tuple(target.joints[i] for i in index)

    saturated = is_saturated(rest[index], lower[index], upper[index], spec)
    if saturated.any():
        clipped = [excited_names[i] for i in range(len(excited_names)) if saturated[i]]
        raise ValueError(f"excitation이 관절 한계에 잘린다 → 식별 불가: {clipped}")

    time, excited_cmd = build_excitation(rest[index], lower[index], upper[index], spec)
    q_cmd = np.tile(rest, (time.shape[0], 1))
    q_cmd[:, index] = excited_cmd
    return ExcitationPlan(target=target, time=time, q_cmd=q_cmd, excited=excited_names)


def _describe(plan: ExcitationPlan) -> str:
    lines = [
        f"target        : {plan.target.name}  → {plan.target.command_topic}",
        f"joints        : {len(plan.target.joints)} (excited: {len(plan.excited)})",
        f"duration      : {plan.duration_sec:.2f} s  ({plan.q_cmd.shape[0]} steps)",
    ]
    for name in plan.excited:
        column = plan.target.joints.index(name)
        series = plan.q_cmd[:, column]
        lines.append(
            f"  {name:22s} rest={series[0]:+.4f}  range [{series.min():+.4f}, {series.max():+.4f}] rad"
        )
    return "\n".join(lines)


def _run_node(plan: ExcitationPlan, args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    target = plan.target
    if target.message == "multidof":
        from control_msgs.msg import MultiDOFCommand as CommandMsg
    else:
        from std_msgs.msg import Float64MultiArray as CommandMsg

    class Excitation(Node):
        def __init__(self) -> None:
            super().__init__(f"r2s_{target.name}_excitation")
            self._pub = self.create_publisher(CommandMsg, target.command_topic, 10)
            self._measured: np.ndarray | None = None
            self.create_subscription(JointState, target.measured_topic, self._on_state, 20)
            self._step = 0
            self._approach_steps = int(round(args.approach_sec / args.dt))
            self._start: np.ndarray | None = None
            self.create_timer(args.dt, self._tick)

        def _on_state(self, message: JointState) -> None:
            lookup = dict(zip(message.name, message.position))
            if all(n in lookup for n in target.joints):
                self._measured = np.array([lookup[n] for n in target.joints])

        def _publish(self, values: np.ndarray) -> None:
            out = CommandMsg()
            if target.message == "multidof":
                out.dof_names = list(target.joints)
                out.values = [float(v) for v in values]
                out.values_dot = [0.0] * len(target.joints)
            else:
                out.data = [float(v) for v in values]
            self._pub.publish(out)

        def _tick(self) -> None:
            if self._measured is None:
                self.get_logger().warn(
                    f"waiting for {target.measured_topic}...", throttle_duration_sec=2.0
                )
                return
            if self._start is None:
                self._start = self._measured.copy()
                self.get_logger().info(
                    f"approach {args.approach_sec}s → excitation {plan.duration_sec:.1f}s"
                )

            if self._step < self._approach_steps:
                alpha = (self._step + 1) / self._approach_steps
                self._publish((1.0 - alpha) * self._start + alpha * plan.q_cmd[0])
            elif self._step - self._approach_steps < plan.q_cmd.shape[0]:
                self._publish(plan.q_cmd[self._step - self._approach_steps])
            else:
                self._publish(plan.q_cmd[-1])
                self.get_logger().info("excitation complete; holding rest pose")
                raise SystemExit(0)
            self._step += 1

    rclpy.init()
    node = Excitation()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish identification excitation to arm or hand.")
    parser.add_argument("--target", choices=sorted(TARGETS), default="hand")
    parser.add_argument(
        "--excite",
        default="wrist",
        choices=sorted(ARM_GROUPS),
        help="팔에서 흔들 group. 손에서는 무시된다 (항상 curl).",
    )
    parser.add_argument("--dry-run", action="store_true", help="발행하지 않고 궤적만 출력한다")
    parser.add_argument("--amplitude-scale", type=float, default=1.0, help="안전을 위한 진폭 배율")
    parser.add_argument("--approach-sec", type=float, default=3.0, help="현재 자세 → 시작 자세 접근 시간")
    parser.add_argument("--dt", type=float, default=0.01, help="발행 주기 [s]")
    parser.add_argument("--asset-dir", type=Path, default=ASSET_DIR)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not 0.0 < args.amplitude_scale <= 1.0:
        raise SystemExit("--amplitude-scale must be in (0, 1]")

    base = ExcitationSpec()
    spec = ExcitationSpec(
        dt=args.dt,
        step_rad=base.step_rad * args.amplitude_scale,
        sine_amp_rad=base.sine_amp_rad * args.amplitude_scale,
    )
    plan = build_plan(TARGETS[args.target], spec, args.excite, args.asset_dir)

    print(_describe(plan))
    if args.target == "arm" and args.excite == "all":
        print("\n경고: 7축을 동시에 흔들면 중력 커플링으로 식별성이 떨어진다. group 단위 권장.")
    if args.dry_run:
        print("\n[dry-run] 아무것도 발행하지 않았다.")
        return 0

    print(f"\n발행 → {plan.target.command_topic} @ {1/args.dt:.0f} Hz")
    return _run_node(plan, args)


if __name__ == "__main__":
    raise SystemExit(main())
