"""Tests for pre-pour annotation validation and align_done truncation."""

from pathlib import Path

import h5py
import numpy as np
import pytest

from pre_pour_hdf5_tools import collect_annotation_issues, filter_demos_by_issue_policy, truncate_dataset_at_term_signal


def _write_demo(group: h5py.Group, name: str, action_steps: int, *, include_align: bool) -> None:
    demo = group.create_group(name)
    demo.attrs["num_samples"] = action_steps
    demo.create_dataset("actions", data=np.zeros((action_steps, 18), dtype=np.float32))

    obs = demo.create_group("obs")
    obs.create_dataset("actor_obs", data=np.zeros((action_steps, 69), dtype=np.float32))

    datagen = obs.create_group("datagen_info")
    terms = datagen.create_group("subtask_term_signals")
    starts = datagen.create_group("subtask_start_signals")

    terms.create_dataset("grasp_done", data=np.array([False, True, True, True, True, True][:action_steps]))
    terms.create_dataset("lift_done", data=np.array([False, False, True, True, True, True][:action_steps]))
    if include_align:
        terms.create_dataset("align_done", data=np.array([False, False, False, True, True, True][:action_steps]))
    terms.create_dataset("pour_done", data=np.array([False, False, False, False, True, True][:action_steps]))
    starts.create_dataset("pour_start", data=np.array([False, False, False, False, True, True][:action_steps]))


def _build_annotated_file(path: Path) -> None:
    with h5py.File(path, "w") as h5:
        data = h5.create_group("data")
        data.attrs["env_args"] = "{}"
        _write_demo(data, "demo_0", 6, include_align=True)
        _write_demo(data, "demo_1", 6, include_align=False)


def test_collect_annotation_issues_detects_missing_required_signal(tmp_path):
    input_file = Path(tmp_path) / "annotated.hdf5"
    _build_annotated_file(input_file)

    issues = collect_annotation_issues(input_file)
    assert len(issues) == 1
    assert issues[0]["demo_key"] == "demo_1"
    assert "align_done" in issues[0]["missing_terms"]


def test_filter_demos_by_issue_policy_skips_invalid_demos(tmp_path):
    input_file = Path(tmp_path) / "annotated.hdf5"
    output_file = Path(tmp_path) / "valid_only.hdf5"
    _build_annotated_file(input_file)

    issues = collect_annotation_issues(input_file)
    kept, skipped = filter_demos_by_issue_policy(input_file, output_file, issues)

    assert kept == 1
    assert skipped == 1
    with h5py.File(output_file, "r") as h5:
        assert "demo_0" in h5["data"]
        assert "demo_1" not in h5["data"]


def test_truncate_dataset_at_align_done(tmp_path):
    input_file = Path(tmp_path) / "input.hdf5"
    output_file = Path(tmp_path) / "truncated.hdf5"
    with h5py.File(input_file, "w") as h5:
        data = h5.create_group("data")
        demo = data.create_group("demo_0")
        demo.attrs["num_samples"] = 6
        demo.create_dataset("actions", data=np.arange(6 * 18, dtype=np.float32).reshape(6, 18))
        obs = demo.create_group("obs")
        obs.create_dataset("actor_obs", data=np.arange(6 * 69, dtype=np.float32).reshape(6, 69))
        datagen = obs.create_group("datagen_info")
        terms = datagen.create_group("subtask_term_signals")
        terms.create_dataset("align_done", data=np.array([False, False, False, True, True, True]))
        terms.create_dataset("grasp_done", data=np.array([False, True, True, True, True, True]))
        terms.create_dataset("lift_done", data=np.array([False, False, True, True, True, True]))
        terms.create_dataset("pour_done", data=np.array([False, False, False, False, True, True]))
        starts = datagen.create_group("subtask_start_signals")
        starts.create_dataset("pour_start", data=np.array([False, False, False, False, True, True]))

    report = truncate_dataset_at_term_signal(input_file, output_file, term_signal="align_done")
    assert report["converted"] == 1

    with h5py.File(output_file, "r") as h5:
        assert h5["data/demo_0/actions"].shape[0] == 4
        assert h5["data/demo_0/obs/actor_obs"].shape[0] == 4
        assert h5["data/demo_0/obs/datagen_info/subtask_term_signals/align_done"].shape[0] == 4


def test_truncate_raises_when_align_done_missing(tmp_path):
    input_file = Path(tmp_path) / "input_missing.hdf5"
    output_file = Path(tmp_path) / "out_missing.hdf5"
    _build_annotated_file(input_file)

    with pytest.raises(Exception):
        truncate_dataset_at_term_signal(input_file, output_file, term_signal="align_done", skip_invalid_demos=False)
