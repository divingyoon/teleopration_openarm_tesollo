"""Tests for run_pour_v1_mimic_pipeline CLI contracts."""

import argparse
import json
from pathlib import Path

import run_pour_v1_mimic_pipeline as pipeline


def test_collect_db3_writes_pre_pour_metadata(tmp_path, monkeypatch):
    def _fake_run(cmd, dry_run):
        return 0

    monkeypatch.setattr(pipeline, "_run", _fake_run)

    args = argparse.Namespace(
        output_dir=str(tmp_path),
        bag_name="demo_bag",
        record_time_sec=5.0,
        domain_id=126,
        max_cache_size=100,
        operator="tester",
        attempt_id="a01",
        success=True,
        tags=["pre_pour"],
        topics=["/a", "/b"],
        dry_run=True,
    )

    ret = pipeline._collect_db3(args)
    assert ret == 0

    meta_file = Path(tmp_path) / "demo_bag_session_meta.json"
    with open(meta_file, "r", encoding="utf-8") as f:
        meta = json.load(f)

    assert meta["intent"] == "pre_pour_init"
    assert meta["task_scope"] == "pre_pour_only"
    assert meta["canonical_source"] == "db3"


def test_annotate_generate_defaults_include_200_trials_and_start_signals():
    parser = pipeline._build_parser()
    args = parser.parse_args(
        [
            "annotate_generate",
            "--input-file",
            "/tmp/in.hdf5",
            "--output-file",
            "/tmp/out.hdf5",
        ]
    )

    assert args.generation_num_trials == 200
    assert args.annotation_validation_policy == "fail_fast"
    assert args.annotate_subtask_start_signals is True


def test_train_bc_invokes_truncation_when_requested(tmp_path, monkeypatch):
    dataset = Path(tmp_path) / "dataset.hdf5"
    dataset.write_bytes(b"stub")

    seen = {}

    def _fake_truncate(input_file, output_file, term_signal, skip_invalid_demos):
        seen["input"] = str(input_file)
        seen["output"] = str(output_file)
        seen["signal"] = term_signal
        seen["skip"] = skip_invalid_demos
        Path(output_file).write_bytes(b"stub_trunc")
        return {"converted": 1, "skipped": 0}

    def _fake_run(cmd, dry_run):
        seen["cmd"] = cmd
        return 0

    monkeypatch.setattr(pipeline, "truncate_dataset_at_term_signal", _fake_truncate)
    monkeypatch.setattr(pipeline, "_run", _fake_run)

    args = argparse.Namespace(
        dataset=str(dataset),
        run_name="bc_test",
        config=None,
        truncate_at="align_done",
        truncate_output_file=None,
        truncate_skip_invalid_demos=False,
        primary_metric="align_done_reach_rate",
        dry_run=False,
    )

    ret = pipeline._train_bc(args)
    assert ret == 0
    assert seen["signal"] == "align_done"
    assert "--dataset" in seen["cmd"]
    assert seen["cmd"][seen["cmd"].index("--dataset") + 1].endswith("_trunc_align_done.hdf5")


def test_convert_hdf5_passes_object_pose_options(monkeypatch):
    seen = {}

    def _fake_run(cmd, dry_run):
        seen["cmd"] = cmd
        seen["dry_run"] = dry_run
        return 0

    monkeypatch.setattr(pipeline, "_run", _fake_run)
    args = argparse.Namespace(
        bag_dirs=["/tmp/bag_a"],
        output_file="/tmp/out.hdf5",
        target_hz=100,
        max_drop_rate=0.2,
        max_gap_sec=0.1,
        object_pose_mode="static",
        source_cup_pose_w="1,0,0,0.35,0,1,0,-0.10,0,0,1,0.05,0,0,0,1",
        target_cup_pose_w="1,0,0,0.35,0,1,0,0.10,0,0,1,0.05,0,0,0,1",
        tf_topic="/tf",
        source_cup_frame="source_cup",
        target_cup_frame="target_cup",
        tf_reference_frame="world",
        grasp_curl_threshold=0.55,
        lift_z_margin=0.06,
        align_xy_threshold=0.08,
        skip_invalid_demos=False,
        dry_run=True,
    )
    ret = pipeline._convert_hdf5(args)
    assert ret == 0
    assert seen["dry_run"] is True
    assert "--object-pose-mode" in seen["cmd"]
    assert seen["cmd"][seen["cmd"].index("--object-pose-mode") + 1] == "static"
    assert "--source-cup-pose-w" in seen["cmd"]
    assert "--target-cup-pose-w" in seen["cmd"]
    assert "--grasp-curl-threshold" in seen["cmd"]
    assert "--lift-z-margin" in seen["cmd"]
    assert "--align-xy-threshold" in seen["cmd"]
