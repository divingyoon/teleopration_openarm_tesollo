#!/usr/bin/env python3
"""Utilities for pre-pour annotation checks and align_done truncation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from pour_v1_mimic_contract import PRE_POUR_REQUIRED_START_SIGNALS, PRE_POUR_REQUIRED_TERM_SIGNALS, PRE_POUR_TRUNCATE_SIGNAL


def _demo_sort_key(name: str) -> tuple[int, str]:
    if name.startswith("demo_"):
        suffix = name[5:]
        if suffix.isdigit():
            return (int(suffix), name)
    return (10**9, name)


def list_demo_keys(data_group: h5py.Group) -> list[str]:
    return sorted([k for k in data_group.keys() if k.startswith("demo_")], key=_demo_sort_key)


def _copy_attrs(src: h5py.AttributeManager, dst: h5py.AttributeManager) -> None:
    for key in src.keys():
        dst[key] = src[key]


def _dataset_copy_kwargs(src_ds: h5py.Dataset, data: np.ndarray) -> dict[str, Any]:
    kwargs: dict[str, Any] = {}
    if data.ndim == 0 or data.shape[0] == 0:
        return kwargs

    if src_ds.compression is not None:
        kwargs["compression"] = src_ds.compression
    if src_ds.compression_opts is not None:
        kwargs["compression_opts"] = src_ds.compression_opts
    if src_ds.shuffle:
        kwargs["shuffle"] = True
    if src_ds.fletcher32:
        kwargs["fletcher32"] = True
    if src_ds.chunks is not None:
        chunks = list(src_ds.chunks)
        for i in range(min(len(chunks), data.ndim)):
            if chunks[i] > data.shape[i]:
                chunks[i] = max(1, data.shape[i])
        kwargs["chunks"] = tuple(chunks)
    return kwargs


def _copy_group_recursive(src_group: h5py.Group, dst_group: h5py.Group, *, orig_len: int | None = None, trunc_len: int | None = None) -> None:
    _copy_attrs(src_group.attrs, dst_group.attrs)
    for key, item in src_group.items():
        if isinstance(item, h5py.Group):
            child = dst_group.create_group(key)
            _copy_group_recursive(item, child, orig_len=orig_len, trunc_len=trunc_len)
            continue

        if not isinstance(item, h5py.Dataset):
            continue

        should_truncate = (
            trunc_len is not None
            and orig_len is not None
            and item.ndim > 0
            and item.shape[0] == orig_len
        )
        data = item[:trunc_len] if should_truncate else item[()]
        kwargs = _dataset_copy_kwargs(item, np.asarray(data))
        dst_ds = dst_group.create_dataset(key, data=data, **kwargs)
        _copy_attrs(item.attrs, dst_ds.attrs)


def _load_signal(demo_group: h5py.Group, signal_group: str, signal_name: str) -> np.ndarray:
    node = demo_group
    for segment in signal_group.strip("/").split("/"):
        if segment not in node:
            raise KeyError(f"missing signal group: {signal_group}")
        node = node[segment]
    if signal_name not in node:
        raise KeyError(f"missing signal: {signal_name}")
    return np.asarray(node[signal_name][:], dtype=bool).reshape(-1)


def collect_annotation_issues(
    input_file: Path,
    required_term_signals: tuple[str, ...] = PRE_POUR_REQUIRED_TERM_SIGNALS,
    required_start_signals: tuple[str, ...] = PRE_POUR_REQUIRED_START_SIGNALS,
) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    with h5py.File(input_file, "r") as h5:
        data = h5["data"]
        for demo_key in list_demo_keys(data):
            demo = data[demo_key]
            issue: dict[str, Any] = {"demo_key": demo_key}

            term_group_missing = "obs" not in demo or "datagen_info" not in demo["obs"] or "subtask_term_signals" not in demo["obs/datagen_info"]
            start_group_missing = "obs" not in demo or "datagen_info" not in demo["obs"] or "subtask_start_signals" not in demo["obs/datagen_info"]

            if term_group_missing:
                issue["missing_term_group"] = True
            if start_group_missing:
                issue["missing_start_group"] = True

            missing_terms: list[str] = []
            zero_terms: list[str] = []
            for signal in required_term_signals:
                try:
                    values = _load_signal(demo, "obs/datagen_info/subtask_term_signals", signal)
                except KeyError:
                    missing_terms.append(signal)
                    continue
                if not np.any(values):
                    zero_terms.append(signal)

            missing_starts: list[str] = []
            zero_starts: list[str] = []
            for signal in required_start_signals:
                try:
                    values = _load_signal(demo, "obs/datagen_info/subtask_start_signals", signal)
                except KeyError:
                    missing_starts.append(signal)
                    continue
                if not np.any(values):
                    zero_starts.append(signal)

            if missing_terms:
                issue["missing_terms"] = missing_terms
            if zero_terms:
                issue["zero_terms"] = zero_terms
            if missing_starts:
                issue["missing_starts"] = missing_starts
            if zero_starts:
                issue["zero_starts"] = zero_starts

            if len(issue) > 1:
                issues.append(issue)
    return issues


def filter_demos_by_issue_policy(input_file: Path, output_file: Path, issues: list[dict[str, Any]]) -> tuple[int, int]:
    bad = {entry["demo_key"] for entry in issues}
    with h5py.File(input_file, "r") as src, h5py.File(output_file, "w") as dst:
        src_data = src["data"]
        dst_data = dst.create_group("data")
        _copy_attrs(src_data.attrs, dst_data.attrs)

        kept = 0
        total_samples = 0
        for demo_key in list_demo_keys(src_data):
            if demo_key in bad:
                continue
            src_demo = src_data[demo_key]
            dst_demo = dst_data.create_group(f"demo_{kept}")
            _copy_group_recursive(src_demo, dst_demo)
            num_samples = int(dst_demo.attrs.get("num_samples", 0))
            total_samples += num_samples
            kept += 1

        dst_data.attrs["total"] = total_samples
        dst_data.attrs["pre_pour_issue_policy"] = "skip"
        dst_data.attrs["pre_pour_issues_count"] = len(issues)

    return kept, len(bad)


def truncate_dataset_at_term_signal(
    input_file: Path,
    output_file: Path,
    term_signal: str = PRE_POUR_TRUNCATE_SIGNAL,
    *,
    skip_invalid_demos: bool = False,
) -> dict[str, Any]:
    converted = 0
    skipped = 0
    total_samples = 0

    with h5py.File(input_file, "r") as src, h5py.File(output_file, "w") as dst:
        src_data = src["data"]
        dst_data = dst.create_group("data")
        _copy_attrs(src_data.attrs, dst_data.attrs)

        for demo_key in list_demo_keys(src_data):
            src_demo = src_data[demo_key]
            actions = src_demo["actions"]
            orig_len = int(actions.shape[0])
            try:
                signal = _load_signal(src_demo, "obs/datagen_info/subtask_term_signals", term_signal)
                if signal.shape[0] != orig_len:
                    raise ValueError(
                        f"signal length mismatch for {demo_key}: signal={signal.shape[0]} actions={orig_len}"
                    )
                indices = np.flatnonzero(signal)
                if indices.size == 0:
                    raise ValueError(f"signal never reached true: {term_signal}")
                trunc_len = int(indices[0]) + 1
            except Exception as exc:
                if skip_invalid_demos:
                    skipped += 1
                    print(f"[WARN] skip demo={demo_key}: {exc}")
                    continue
                raise

            dst_demo = dst_data.create_group(f"demo_{converted}")
            _copy_group_recursive(src_demo, dst_demo, orig_len=orig_len, trunc_len=trunc_len)
            dst_demo.attrs["num_samples"] = trunc_len
            dst_demo.attrs["pre_pour_truncate_signal"] = term_signal
            converted += 1
            total_samples += trunc_len

        if converted == 0:
            raise RuntimeError("no demos left after truncation")

        dst_data.attrs["total"] = total_samples
        dst_data.attrs["pre_pour_scope"] = "align_done_only"
        dst_data.attrs["pre_pour_truncate_signal"] = term_signal

    return {
        "converted": converted,
        "skipped": skipped,
        "output_file": str(output_file),
        "term_signal": term_signal,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_validate = sub.add_parser("validate")
    p_validate.add_argument("--input-file", required=True)
    p_validate.set_defaults(command_func=_cmd_validate)

    p_trunc = sub.add_parser("truncate")
    p_trunc.add_argument("--input-file", required=True)
    p_trunc.add_argument("--output-file", required=True)
    p_trunc.add_argument("--term-signal", default=PRE_POUR_TRUNCATE_SIGNAL)
    p_trunc.add_argument("--skip-invalid-demos", action="store_true")
    p_trunc.set_defaults(command_func=_cmd_truncate)

    return parser.parse_args()


def _cmd_validate(args: argparse.Namespace) -> int:
    issues = collect_annotation_issues(Path(args.input_file).expanduser().resolve())
    report = {"issues": issues, "issue_count": len(issues)}
    print(json.dumps(report, indent=2))
    return 0 if not issues else 1


def _cmd_truncate(args: argparse.Namespace) -> int:
    report = truncate_dataset_at_term_signal(
        input_file=Path(args.input_file).expanduser().resolve(),
        output_file=Path(args.output_file).expanduser().resolve(),
        term_signal=args.term_signal,
        skip_invalid_demos=args.skip_invalid_demos,
    )
    print(json.dumps(report, indent=2))
    return 0


def main() -> int:
    args = parse_args()
    return int(args.command_func(args))


if __name__ == "__main__":
    raise SystemExit(main())
