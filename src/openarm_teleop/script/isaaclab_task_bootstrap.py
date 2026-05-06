#!/usr/bin/env python3
"""Bootstrap IsaacLab mimic scripts with custom task registration imports."""

from __future__ import annotations

import argparse
import importlib
import runpy
import sys
from pathlib import Path


DEFAULT_TASK_IMPORT = "openarm.tasks.manager_based.openarm_manipulation.pipeline.hand.both.pour_v1_mimic"
DEFAULT_OPENARM_SRC = "/home/usr/rl_ws/hdgp/source/openarm"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaaclab-script", required=True, help="Target IsaacLab script path to execute.")
    parser.add_argument(
        "--openarm-src-path",
        default=DEFAULT_OPENARM_SRC,
        help="Path to openarm source root to prepend into PYTHONPATH/sys.path.",
    )
    parser.add_argument(
        "--task-import-module",
        action="append",
        default=[],
        help="Python module to import for task gym.register side-effects. Can be repeated.",
    )
    parser.add_argument("script_args", nargs=argparse.REMAINDER)
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    openarm_src = Path(args.openarm_src_path).expanduser().resolve()
    if not openarm_src.exists():
        raise FileNotFoundError(f"openarm source path not found: {openarm_src}")
    sys.path.insert(0, str(openarm_src))

    modules = args.task_import_module or [DEFAULT_TASK_IMPORT]
    for module_name in modules:
        importlib.import_module(module_name)

    script_path = Path(args.isaaclab_script).expanduser().resolve()
    if not script_path.exists():
        raise FileNotFoundError(f"IsaacLab script not found: {script_path}")

    forward = list(args.script_args)
    if forward and forward[0] == "--":
        forward = forward[1:]
    sys.argv = [str(script_path), *forward]
    runpy.run_path(str(script_path), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
