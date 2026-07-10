import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from identification_hdf5 import (  # noqa: E402
    IdentificationError,
    NamedSample,
    assert_identifiable,
    build_identification_track,
    write_identification_hdf5,
)

NAMES = ("rj_dg_1_2", "rj_dg_2_2")
MS = 1_000_000


def _cmd(t_ms, values):
    return NamedSample(timestamp_ns=t_ms * MS, names=NAMES, positions=tuple(values))


def _meas(t_ms, values, vel=None):
    return NamedSample(
        timestamp_ns=t_ms * MS,
        names=NAMES,
        positions=tuple(values),
        velocities=tuple(vel) if vel else None,
    )


def _stream(n, offset_ms=0, period_ms=10, scale=1.0, meas=False):
    out = []
    for i in range(n):
        v = [scale * 0.1 * i, scale * 0.2 * i]
        t = offset_ms + i * period_ms
        out.append(_meas(t, v) if meas else _cmd(t, v))
    return out


def test_track_uses_command_dof_names_by_default():
    track = build_identification_track(_stream(20), _stream(20, meas=True), dt=0.01)

    assert track.joint_names == NAMES


def test_grid_is_uniform_at_requested_dt():
    track = build_identification_track(_stream(20), _stream(20, meas=True), dt=0.01)

    deltas = np.diff(track.timestamps_ns)
    assert np.all(deltas == 10 * MS)


def test_grid_is_restricted_to_the_overlapping_window():
    """한쪽만 살아 있는 구간을 포함하면 nearest 보간이 가짜 정상상태를 만든다."""
    command = _stream(20)  # 0..190 ms
    measured = _stream(20, offset_ms=100, meas=True)  # 100..290 ms

    track = build_identification_track(command, measured, dt=0.01)

    assert track.timestamps_ns[0] == 100 * MS
    assert track.timestamps_ns[-1] == 190 * MS


def test_non_overlapping_streams_are_rejected():
    command = _stream(5)  # 0..40 ms
    measured = _stream(5, offset_ms=1000, meas=True)

    with pytest.raises(IdentificationError, match="do not overlap"):
        build_identification_track(command, measured, dt=0.01)


def test_measured_joints_are_reordered_to_command_order():
    command = [_cmd(0, [1.0, 2.0]), _cmd(10, [1.0, 2.0])]
    swapped = tuple(reversed(NAMES))
    measured = [
        NamedSample(0, swapped, (20.0, 10.0)),
        NamedSample(10 * MS, swapped, (20.0, 10.0)),
    ]

    track = build_identification_track(command, measured, dt=0.01)

    assert track.joint_names == NAMES
    np.testing.assert_allclose(track.q_real[0], [10.0, 20.0])


def test_missing_joint_in_measured_stream_is_rejected():
    command = [_cmd(0, [1.0, 2.0]), _cmd(10, [1.0, 2.0])]
    measured = [NamedSample(0, ("rj_dg_1_2",), (1.0,)), NamedSample(10 * MS, ("rj_dg_1_2",), (1.0,))]

    with pytest.raises(IdentificationError, match="missing joints"):
        build_identification_track(command, measured, dt=0.01)


def test_velocity_is_finite_differenced_when_driver_reports_zeros():
    command = _stream(20)
    measured = _stream(20, meas=True)  # velocities=None → 0으로 채워짐

    track = build_identification_track(command, measured, dt=0.01)

    assert np.any(track.dq_real != 0.0)


def test_reported_velocity_is_preserved_when_present():
    command = [_cmd(0, [0.0, 0.0]), _cmd(10, [0.0, 0.0])]
    measured = [_meas(0, [0.0, 0.0], vel=[7.0, 8.0]), _meas(10, [0.0, 0.0], vel=[7.0, 8.0])]

    track = build_identification_track(command, measured, dt=0.01)

    np.testing.assert_allclose(track.dq_real[0], [7.0, 8.0])


def test_identifiability_check_rejects_a_motionless_command():
    """excitation 노드를 안 띄우고 bag만 녹화하면 정지 데이터가 쌓인다."""
    still = [_cmd(t * 10, [0.0, 0.0]) for t in range(20)]
    measured = [_meas(t * 10, [0.0, 0.0]) for t in range(20)]
    track = build_identification_track(still, measured, dt=0.01)

    with pytest.raises(IdentificationError, match="barely moves"):
        assert_identifiable(track)


def test_identifiability_check_rejects_command_equal_to_measured():
    command = _stream(20)
    measured = [_meas(s.timestamp_ns // MS, s.positions) for s in command]
    track = build_identification_track(command, measured, dt=0.01)

    with pytest.raises(IdentificationError, match="command equals measured"):
        assert_identifiable(track)


def test_identifiability_check_passes_for_a_real_looking_track():
    command = _stream(60)
    measured = _stream(60, scale=0.9, meas=True)
    track = build_identification_track(command, measured, dt=0.01)

    assert_identifiable(track)  # 예외 없음


def test_hdf5_matches_the_schema_load_real_track_expects(tmp_path):
    import h5py

    track = build_identification_track(_stream(30), _stream(30, scale=0.9, meas=True), dt=0.01)
    path = tmp_path / "ident.hdf5"

    write_identification_hdf5(path, track, attrs={"source_bag": "/bags/run_001"})

    with h5py.File(path, "r") as handle:
        demo = handle["data/demo_0"]
        assert demo.attrs["num_samples"] == track.q_cmd.shape[0]
        assert handle["data"].attrs["source_bag"] == "/bags/run_001"
        assert "timestamps_ns" in demo
        for key in ("q_cmd", "q_real", "dq_real"):
            dataset = demo[f"obs/{key}"]
            assert dataset.shape == (track.q_cmd.shape[0], len(NAMES))
            assert tuple(json.loads(dataset.attrs["joint_names"])) == NAMES
