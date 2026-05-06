"""Tests for record_to_hdf5.py — TDD RED phase.

Tests cover:
- TopicBuffer: thread-safe latest-value store
- HDF5Writer: IsaacLab Mimic compatible file output
- EpisodeRecorder: recording state machine and data accumulation
"""

import threading

import h5py
import numpy as np
import pytest


# ──────────────────────────────────────────────────────────────────────────────
# Imports under test (will fail until implementation exists)
# ──────────────────────────────────────────────────────────────────────────────
from record_to_hdf5 import EpisodeRecorder, HDF5Writer, TopicBuffer


# ══════════════════════════════════════════════════════════════════════════════
# TopicBuffer
# ══════════════════════════════════════════════════════════════════════════════


class TestTopicBuffer:
    def test_get_returns_none_when_empty(self):
        buf = TopicBuffer()
        assert buf.get() is None

    def test_update_then_get_returns_value(self):
        buf = TopicBuffer()
        buf.update(np.array([1.0, 2.0, 3.0]))
        result = buf.get()
        assert result is not None
        np.testing.assert_array_equal(result, [1.0, 2.0, 3.0])

    def test_get_always_returns_latest(self):
        buf = TopicBuffer()
        buf.update(np.array([1.0]))
        buf.update(np.array([9.9]))
        np.testing.assert_array_equal(buf.get(), [9.9])

    def test_thread_safety_concurrent_writes(self):
        buf = TopicBuffer()
        errors = []

        def writer(val):
            try:
                for _ in range(200):
                    buf.update(np.array([float(val)]))
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [], f"Thread errors: {errors}"
        assert buf.get() is not None

    def test_get_is_non_destructive(self):
        buf = TopicBuffer()
        buf.update(np.array([7.0]))
        _ = buf.get()
        assert buf.get() is not None


# ══════════════════════════════════════════════════════════════════════════════
# HDF5Writer
# ══════════════════════════════════════════════════════════════════════════════


class TestHDF5Writer:
    @pytest.fixture
    def tmp_hdf5(self, tmp_path):
        return str(tmp_path / "test_dataset.hdf5")

    def test_creates_file_with_data_group(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5, env_name="TestEnv")
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert "data" in f

    def test_data_group_has_env_args(self, tmp_hdf5):
        import json
        writer = HDF5Writer(tmp_hdf5, env_name="OpenArmBilateral")
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            env_args = json.loads(f["data"].attrs["env_args"])
            assert env_args["env_name"] == "OpenArmBilateral"
            assert env_args["type"] == 2

    def test_write_episode_creates_demo_group(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=5), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert "data/demo_0" in f

    def test_write_episode_stores_actions(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=10, action_dim=34), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert f["data/demo_0/actions"][:].shape == (10, 34)

    def test_write_episode_stores_obs_keys(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=5), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            obs = f["data/demo_0/obs"]
            for key in ("left_joint_pos", "right_joint_pos", "tesollo_joint_states", "tesollo_sensor"):
                assert key in obs

    def test_write_episode_stores_leader_obs(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=5), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert "data/demo_0/leader_obs" in f

    def test_write_episode_sets_num_samples(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=7), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert f["data/demo_0"].attrs["num_samples"] == 7

    def test_write_episode_sets_success_flag(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=3), success=False)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert bool(f["data/demo_0"].attrs["success"]) is False

    def test_multiple_episodes_increment_demo_id(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        for _ in range(3):
            writer.write_episode(_make_episode_data(n_steps=2), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            for i in range(3):
                assert f"data/demo_{i}" in f

    def test_data_group_total_counts_all_steps(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=4), success=True)
        writer.write_episode(_make_episode_data(n_steps=6), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert f["data"].attrs["total"] == 10

    def test_skips_empty_episode(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode({}, success=False)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert len(list(f["data"].keys())) == 0

    def test_uses_gzip_compression(self, tmp_hdf5):
        writer = HDF5Writer(tmp_hdf5)
        writer.write_episode(_make_episode_data(n_steps=10), success=True)
        writer.close()
        with h5py.File(tmp_hdf5, "r") as f:
            assert f["data/demo_0/actions"].compression == "gzip"


# ══════════════════════════════════════════════════════════════════════════════
# EpisodeRecorder
# ══════════════════════════════════════════════════════════════════════════════


class TestEpisodeRecorder:
    def _make_buffers(self, left_empty=False):
        buffers = {
            "left_joint_states": TopicBuffer(),
            "right_joint_states": TopicBuffer(),
            "left_gripper": TopicBuffer(),
            "dg5f_reference": TopicBuffer(),
            "dg5f_joint_states": TopicBuffer(),
            "tesollo_sensor": TopicBuffer(),
        }
        if not left_empty:
            buffers["left_joint_states"].update(np.zeros(7))
        buffers["right_joint_states"].update(np.zeros(7))
        buffers["left_gripper"].update(np.zeros(1))
        buffers["dg5f_reference"].update(np.zeros(20))
        buffers["dg5f_joint_states"].update(np.zeros(20))
        buffers["tesollo_sensor"].update(np.zeros(30))
        return buffers

    def test_initial_state_is_idle(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        assert not rec.is_recording

    def test_start_sets_recording_true(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        assert rec.is_recording

    def test_stop_sets_recording_false(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.stop()
        assert not rec.is_recording

    def test_stop_before_start_raises(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        with pytest.raises(RuntimeError):
            rec.stop()

    def test_sample_when_recording_accumulates_data(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.sample()
        rec.sample()
        rec.sample()
        episode = rec.stop()
        assert episode["actions"].shape[0] == 3

    def test_sample_when_idle_is_noop(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.sample()  # no-op when idle

    def test_stop_returns_correct_action_dim(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.sample()
        episode = rec.stop()
        # actions = left(7) + right(7) + dg5f_reference(20) = 34
        assert episode["actions"].shape == (1, 34)

    def test_stop_returns_correct_obs_shapes(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.sample()
        episode = rec.stop()
        assert episode["obs"]["left_joint_pos"].shape == (1, 7)
        assert episode["obs"]["right_joint_pos"].shape == (1, 7)
        assert episode["obs"]["tesollo_joint_states"].shape == (1, 20)
        assert episode["obs"]["tesollo_sensor"].shape == (1, 30)

    def test_stop_returns_leader_obs(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.sample()
        episode = rec.stop()
        assert "leader_obs" in episode
        assert "left_joint_pos" in episode["leader_obs"]

    def test_stop_clears_buffer_for_next_episode(self):
        rec = EpisodeRecorder(buffers=self._make_buffers())
        rec.start()
        rec.sample()
        rec.stop()
        rec.start()
        rec.sample()
        rec.sample()
        episode2 = rec.stop()
        assert episode2["actions"].shape[0] == 2

    def test_sample_with_missing_buffer_uses_zeros(self):
        """Empty buffer yields zeros instead of crashing."""
        rec = EpisodeRecorder(buffers=self._make_buffers(left_empty=True))
        rec.start()
        rec.sample()
        episode = rec.stop()
        np.testing.assert_array_equal(episode["obs"]["left_joint_pos"][0], np.zeros(7))


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────


def _make_episode_data(n_steps: int = 5, action_dim: int = 34) -> dict:
    return {
        "actions": np.random.randn(n_steps, action_dim).astype(np.float32),
        "obs": {
            "left_joint_pos": np.random.randn(n_steps, 7).astype(np.float32),
            "right_joint_pos": np.random.randn(n_steps, 7).astype(np.float32),
            "left_gripper": np.random.randn(n_steps, 1).astype(np.float32),
            "tesollo_joint_states": np.random.randn(n_steps, 20).astype(np.float32),
            "tesollo_sensor": np.random.randn(n_steps, 30).astype(np.float32),
        },
        "leader_obs": {
            "left_joint_pos": np.random.randn(n_steps, 7).astype(np.float32),
            "right_joint_pos": np.random.randn(n_steps, 7).astype(np.float32),
            "left_gripper": np.random.randn(n_steps, 1).astype(np.float32),
        },
    }
