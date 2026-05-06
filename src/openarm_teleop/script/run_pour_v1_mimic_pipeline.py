#!/usr/bin/env python3
"""Single entrypoint for pour_v1_mimic pipeline stages."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import h5py

from pour_v1_mimic_contract import PRE_POUR_INTENT, PRE_POUR_TRUNCATE_SIGNAL, TASK_ID_EVAL, TASK_ID_MIMIC
from pre_pour_hdf5_tools import collect_annotation_issues, filter_demos_by_issue_policy, truncate_dataset_at_term_signal


DEFAULT_TOPICS = [
    "/openarm/left/joint_states",
    "/openarm/right/joint_states",
    "/openarm/left/leader/gripper_state",
    "/dg5f_right/rj_dg_pospid/reference",
    "/dg5f_right/joint_states",
    "/tesollo/right/sensor",
    "/tf",
    "/tf_static",
]


def _run(cmd: list[str], dry_run: bool) -> int:
    print("+", " ".join(shlex.quote(x) for x in cmd))
    if dry_run:
        return 0
    return subprocess.run(cmd, check=False).returncode


def _run_capture(cmd: list[str], dry_run: bool) -> tuple[int, str, str]:
    print("+", " ".join(shlex.quote(x) for x in cmd))
    if dry_run:
        return 0, "", ""
    proc = subprocess.run(cmd, check=False, text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="")
    return proc.returncode, proc.stdout or "", proc.stderr or ""


def _count_demo_groups(hdf5_file: Path) -> int:
    if not hdf5_file.exists():
        return 0
    with h5py.File(hdf5_file, "r") as h5:
        if "data" not in h5:
            return 0
        return sum(1 for key in h5["data"].keys() if key.startswith("demo_"))


def _summarize_failure_reasons(log_text: str, max_items: int = 5) -> list[str]:
    reasons: dict[str, int] = {}
    patterns = [
        r"([^\n]*\bcollision\b[^\n]*)",
        r"([^\n]*\btimeout\b[^\n]*)",
        r"([^\n]*\bfailed\b[^\n]*)",
        r"([^\n]*\berror\b[^\n]*)",
        r"([^\n]*\binvalid\b[^\n]*)",
    ]
    for pattern in patterns:
        for match in re.findall(pattern, log_text, flags=re.IGNORECASE):
            line = re.sub(r"\s+", " ", match).strip()
            if not line:
                continue
            key = line[:180]
            reasons[key] = reasons.get(key, 0) + 1

    ranked = sorted(reasons.items(), key=lambda item: item[1], reverse=True)
    return [f"{line} (x{count})" for line, count in ranked[:max_items]]


def _collect_db3(args: argparse.Namespace) -> int:
    out_dir = Path(args.output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_name = args.bag_name or f"pour_v1_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    bag_path = out_dir / bag_name

    metadata = {
        "task": TASK_ID_MIMIC,
        "intent": PRE_POUR_INTENT,
        "task_scope": "pre_pour_only",
        "operator": args.operator,
        "attempt_id": args.attempt_id,
        "label_success": args.success,
        "segment_tags": args.tags,
        "ros_domain_id": args.domain_id,
        "topics": args.topics,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "canonical_source": "db3",
    }
    with open(out_dir / f"{bag_name}_session_meta.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(bag_path),
        "-s",
        "sqlite3",
        "--max-cache-size",
        str(args.max_cache_size),
    ] + list(args.topics)

    print("+", " ".join(shlex.quote(x) for x in cmd))
    if args.dry_run:
        return 0

    env = {**os.environ, "ROS_DOMAIN_ID": str(args.domain_id)}
    # start_new_session puts ros2 and all its DDS/storage children in a new
    # process group so killpg(SIGINT) reaches every process simultaneously —
    # the same effect as pressing Ctrl+C in the terminal.
    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    try:
        time.sleep(args.record_time_sec)
    except KeyboardInterrupt:
        pass

    pgid = None
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        pass

    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        proc.wait()

    return proc.returncode


def _convert_hdf5(args: argparse.Namespace) -> int:
    script = Path(__file__).resolve().parent / "db3_to_pour_mimic_hdf5.py"
    cmd = [
        "python3",
        str(script),
        "--output-file",
        str(Path(args.output_file).expanduser().resolve()),
        "--target-hz",
        str(args.target_hz),
        "--max-drop-rate",
        str(args.max_drop_rate),
        "--max-gap-sec",
        str(args.max_gap_sec),
        "--object-pose-mode",
        str(args.object_pose_mode),
        "--source-cup-pose-w",
        str(args.source_cup_pose_w),
        "--target-cup-pose-w",
        str(args.target_cup_pose_w),
        "--tf-topic",
        str(args.tf_topic),
        "--source-cup-frame",
        str(args.source_cup_frame),
        "--target-cup-frame",
        str(args.target_cup_frame),
        "--tf-reference-frame",
        str(args.tf_reference_frame),
        "--grasp-curl-threshold",
        str(args.grasp_curl_threshold),
        "--lift-z-margin",
        str(args.lift_z_margin),
        "--align-xy-threshold",
        str(args.align_xy_threshold),
    ]
    if args.skip_invalid_demos:
        cmd.append("--skip-invalid-demos")
    for bag_dir in args.bag_dirs:
        cmd.extend(["--bag-dir", str(Path(bag_dir).expanduser().resolve())])
    return _run(cmd, dry_run=args.dry_run)


def _annotate_generate(args: argparse.Namespace) -> int:
    isaaclab_root = Path(args.isaaclab_root).expanduser().resolve()
    isaaclab_sh = isaaclab_root / "isaaclab.sh"
    annotate = isaaclab_root / "scripts/imitation_learning/isaaclab_mimic/annotate_demos.py"
    generate = isaaclab_root / "scripts/imitation_learning/isaaclab_mimic/generate_dataset.py"
    bootstrap = Path(__file__).resolve().parent / "isaaclab_task_bootstrap.py"

    input_file = Path(args.input_file).expanduser().resolve()
    output_file = Path(args.output_file).expanduser().resolve()
    annotated_file = (
        Path(args.annotated_output_file).expanduser().resolve()
        if args.annotated_output_file
        else input_file.with_name(input_file.stem + "_annotated.hdf5")
    )

    cmd1 = [
        str(isaaclab_sh),
        "-p",
        str(bootstrap),
        "--isaaclab-script",
        str(annotate),
        "--openarm-src-path",
        str(Path(args.openarm_src_path).expanduser().resolve()),
    ]
    for module_name in (args.task_import_module or []):
        cmd1.extend(["--task-import-module", module_name])
    cmd1.extend(
        [
            "--",
        "--task",
        TASK_ID_MIMIC,
        "--input_file",
        str(input_file),
        "--output_file",
        str(annotated_file),
        "--auto",
        ]
    )
    if args.annotate_subtask_start_signals:
        cmd1.append("--annotate_subtask_start_signals")

    ret = _run(cmd1, dry_run=args.dry_run)
    if ret != 0:
        return ret

    if args.dry_run:
        return 0

    issues = collect_annotation_issues(annotated_file)
    validated_input = annotated_file
    if issues:
        if args.annotation_validation_policy == "fail_fast":
            detail = json.dumps(issues[:10], indent=2)
            raise RuntimeError(
                f"annotation validation failed: {len(issues)} demos have missing pre-pour signals\n{detail}"
            )

        validated_input = (
            Path(args.validated_input_file).expanduser().resolve()
            if args.validated_input_file
            else annotated_file.with_name(annotated_file.stem + "_valid.hdf5")
        )
        kept, skipped = filter_demos_by_issue_policy(annotated_file, validated_input, issues)
        print(
            f"[WARN] annotation policy=skip filtered source demos: "
            f"kept={kept} skipped={skipped} validated_input={validated_input}"
        )

    cmd2 = [
        str(isaaclab_sh),
        "-p",
        str(bootstrap),
        "--isaaclab-script",
        str(generate),
        "--openarm-src-path",
        str(Path(args.openarm_src_path).expanduser().resolve()),
    ]
    for module_name in (args.task_import_module or []):
        cmd2.extend(["--task-import-module", module_name])
    cmd2.extend(
        [
            "--",
        "--task",
        TASK_ID_MIMIC,
        "--input_file",
        str(validated_input),
        "--output_file",
        str(output_file),
        "--num_envs",
        str(args.num_envs),
        "--generation_num_trials",
        str(args.generation_num_trials),
        ]
    )
    ret, stdout, stderr = _run_capture(cmd2, dry_run=False)

    generated_success = _count_demo_groups(output_file)
    requested = int(args.generation_num_trials)
    success_rate = (float(generated_success) / float(requested)) if requested > 0 else 0.0
    failed = max(0, requested - generated_success)
    print(
        f"[REPORT] requested={requested} success={generated_success} failed={failed} "
        f"success_rate={success_rate:.3f} output={output_file}"
    )
    if failed > 0:
        reasons = _summarize_failure_reasons(stdout + "\n" + stderr)
        if reasons:
            for reason in reasons:
                print(f"[REPORT] failure_reason={reason}")
        else:
            print("[REPORT] failure_reason=unclassified_from_logs")
    return ret


def _train_bc(args: argparse.Namespace) -> int:
    dataset_path = Path(args.dataset).expanduser().resolve()
    if args.truncate_at:
        output_file = (
            Path(args.truncate_output_file).expanduser().resolve()
            if args.truncate_output_file
            else dataset_path.with_name(dataset_path.stem + f"_trunc_{args.truncate_at}.hdf5")
        )
        if args.dry_run:
            print(
                f"[REPORT] dry_run pre_pour_truncate input={dataset_path} output={output_file} "
                f"term_signal={args.truncate_at}"
            )
        else:
            report = truncate_dataset_at_term_signal(
                input_file=dataset_path,
                output_file=output_file,
                term_signal=args.truncate_at,
                skip_invalid_demos=args.truncate_skip_invalid_demos,
            )
            print(f"[REPORT] pre_pour_truncate={json.dumps(report)}")
        dataset_path = output_file

    cmd = [
        "python3",
        "robomimic/scripts/train.py",
        "--algo",
        "bc_rnn",
        "--dataset",
        str(dataset_path),
        "--name",
        args.run_name,
    ]
    if args.config:
        cmd.extend(["--config", str(Path(args.config).expanduser().resolve())])

    print(
        f"[REPORT] bc_primary_metric={args.primary_metric} "
        f"bc_scope=pre_pour_until_{args.truncate_at or 'full_demo'}"
    )
    return _run(cmd, dry_run=args.dry_run)


def _train_ppo(args: argparse.Namespace) -> int:
    isaaclab_root = Path(args.isaaclab_root).expanduser().resolve()
    isaaclab_sh = isaaclab_root / "isaaclab.sh"
    train_script = isaaclab_root / "scripts/reinforcement_learning/rsl_rl/train.py"

    cmd = [
        str(isaaclab_sh),
        "-p",
        str(train_script),
        "--task",
        TASK_ID_EVAL,
        "--seed",
        str(args.seed),
        "--max_iterations",
        str(args.max_iterations),
    ]
    if args.mode == "warm_start":
        if not args.bc_checkpoint:
            raise ValueError("--bc-checkpoint is required for --mode warm_start")
        cmd.extend(["--load_run", str(Path(args.bc_checkpoint).expanduser().resolve())])
    elif args.mode == "bc_init_pose":
        cmd.extend(["--", "--bootstrap_init_pose_from_bc"])
        if args.bc_checkpoint:
            cmd.extend(["--bc_checkpoint", str(Path(args.bc_checkpoint).expanduser().resolve())])
    return _run(cmd, dry_run=args.dry_run)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_collect = sub.add_parser("collect_db3")
    p_collect.add_argument("--output-dir", required=True)
    p_collect.add_argument("--bag-name", default=None)
    p_collect.add_argument("--record-time-sec", type=float, default=15.0)
    p_collect.add_argument("--domain-id", type=int, default=126)
    p_collect.add_argument("--max-cache-size", type=int, default=100000000)
    p_collect.add_argument("--operator", default="unknown")
    p_collect.add_argument("--attempt-id", default="attempt_0")
    p_collect.add_argument("--success", action="store_true")
    p_collect.add_argument("--tags", nargs="*", default=[])
    p_collect.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS)
    p_collect.add_argument("--dry-run", action="store_true")
    p_collect.set_defaults(func=_collect_db3)

    p_convert = sub.add_parser("convert_hdf5")
    p_convert.add_argument("--bag-dirs", nargs="+", required=True)
    p_convert.add_argument("--output-file", required=True)
    p_convert.add_argument("--target-hz", type=int, default=100)
    p_convert.add_argument("--max-drop-rate", type=float, default=0.2)
    p_convert.add_argument("--max-gap-sec", type=float, default=0.1)
    p_convert.add_argument("--object-pose-mode", choices=("static", "tf"), default="static")
    p_convert.add_argument(
        "--source-cup-pose-w",
        default="1,0,0,0.35,0,1,0,-0.10,0,0,1,0.05,0,0,0,1",
        help="Row-major 4x4 world pose for source cup (used in static mode).",
    )
    p_convert.add_argument(
        "--target-cup-pose-w",
        default="1,0,0,0.35,0,1,0,0.10,0,0,1,0.05,0,0,0,1",
        help="Row-major 4x4 world pose for target cup (used in static mode).",
    )
    p_convert.add_argument("--tf-topic", default="/tf", help="TF object-pose mode topic.")
    p_convert.add_argument("--source-cup-frame", default="source_cup", help="TF frame id for source cup.")
    p_convert.add_argument("--target-cup-frame", default="target_cup", help="TF frame id for target cup.")
    p_convert.add_argument("--tf-reference-frame", default="world", help="TF reference/world frame.")
    p_convert.add_argument("--grasp-curl-threshold", type=float, default=0.55)
    p_convert.add_argument("--lift-z-margin", type=float, default=0.06)
    p_convert.add_argument("--align-xy-threshold", type=float, default=0.08)
    p_convert.add_argument("--skip-invalid-demos", action="store_true")
    p_convert.add_argument("--dry-run", action="store_true")
    p_convert.set_defaults(func=_convert_hdf5)

    p_annot = sub.add_parser("annotate_generate")
    p_annot.add_argument("--isaaclab-root", default="/home/usr/rl_ws/IsaacLab")
    p_annot.add_argument("--input-file", required=True)
    p_annot.add_argument("--output-file", required=True)
    p_annot.add_argument("--annotated-output-file", default=None)
    p_annot.add_argument("--validated-input-file", default=None)
    p_annot.add_argument("--num-envs", type=int, default=1)
    p_annot.add_argument("--generation-num-trials", type=int, default=200)
    p_annot.add_argument("--openarm-src-path", default="/home/usr/rl_ws/hdgp/source/openarm")
    p_annot.add_argument(
        "--task-import-module",
        action="append",
        default=[],
        help=(
            "Extra python module import(s) to register custom gym tasks before "
            "annotate/generate script execution."
        ),
    )
    p_annot.add_argument(
        "--annotation-validation-policy",
        choices=("fail_fast", "skip"),
        default="fail_fast",
    )
    p_annot.add_argument(
        "--annotate-subtask-start-signals",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    p_annot.add_argument("--dry-run", action="store_true")
    p_annot.set_defaults(func=_annotate_generate)

    p_bc = sub.add_parser("train_bc")
    p_bc.add_argument("--dataset", required=True)
    p_bc.add_argument("--run-name", default="pour_v1_mimic_bc")
    p_bc.add_argument("--config", default=None)
    p_bc.add_argument("--truncate-at", choices=(PRE_POUR_TRUNCATE_SIGNAL,), default=None)
    p_bc.add_argument("--truncate-output-file", default=None)
    p_bc.add_argument("--truncate-skip-invalid-demos", action="store_true")
    p_bc.add_argument("--primary-metric", default="align_done_reach_rate")
    p_bc.add_argument("--dry-run", action="store_true")
    p_bc.set_defaults(func=_train_bc)

    p_ppo = sub.add_parser("train_ppo")
    p_ppo.add_argument("--isaaclab-root", default="/home/usr/rl_ws/IsaacLab")
    p_ppo.add_argument("--mode", choices=("warm_start", "bc_init_pose"), default="warm_start")
    p_ppo.add_argument("--bc-checkpoint", default=None)
    p_ppo.add_argument("--seed", type=int, default=0)
    p_ppo.add_argument("--max-iterations", type=int, default=2000)
    p_ppo.add_argument("--dry-run", action="store_true")
    p_ppo.set_defaults(func=_train_ppo)
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
