"""Contract tests for ReconstructionBackend (ARCHITECTURE.md §4.4).

Uses a tiny fake backend so the contract is tested without Open3D or ROS.
Every real backend must also pass these behaviours.
"""

import threading

import numpy as np
import pytest

from cr_tsdf_mapping.reconstruction_backend import (
    CameraIntrinsics,
    MeshSnapshot,
    Observation,
    ReconstructionBackend,
    make_backend,
    register_backend,
    registered_backends,
)

INTR = CameraIntrinsics(width=8, height=6, fx=5.0, fy=5.0, cx=3.5, cy=2.5)


def make_obs(stamp_ns=1, frame_index=0, depth_value=1.0, T=None, **overrides):
    depth = np.full((INTR.height, INTR.width), depth_value, dtype=np.float32)
    kwargs = dict(
        stamp_ns=stamp_ns,
        frame_index=frame_index,
        depth=depth,
        intrinsics=INTR,
        T_world_opt=np.eye(4) if T is None else T,
    )
    kwargs.update(overrides)
    return Observation(**kwargs)


@register_backend("fake_test")
class FakeBackend(ReconstructionBackend):
    """Stores one vertex per integrated frame: the world point at the image centre."""

    def __init__(self, scale=1.0):
        self.scale = scale
        self.points = []
        super().__init__()

    def _integrate_impl(self, obs):
        z = float(obs.depth[INTR.height // 2, INTR.width // 2]) * self.scale
        self.points.append((obs.T_world_opt @ np.array([0.0, 0.0, z, 1.0]))[:3])

    def _extract_impl(self):
        v = np.array(self.points, dtype=np.float64).reshape(-1, 3)
        tris = np.array([[0, 1, 2]]) if len(v) >= 3 else np.empty((0, 3))
        return v, tris

    def _reset_impl(self):
        self.points = []


# -- validation ---------------------------------------------------------------


def test_valid_observation_passes():
    make_obs().validate()


@pytest.mark.parametrize(
    "overrides, message",
    [
        (dict(depth=np.full((6, 8), np.nan, dtype=np.float32)), "NaN"),
        (dict(depth=np.ones((6, 8), dtype=np.float64)), "float32"),
        (dict(depth=np.ones((5, 8), dtype=np.float32)), "shape"),
        (dict(depth=-np.ones((6, 8), dtype=np.float32)), "negative"),
        (dict(rgb=np.zeros((6, 8, 3), dtype=np.float32)), "uint8"),
        (dict(stamp_ns=-1), "non-negative"),
    ],
)
def test_invalid_observation_is_rejected(overrides, message):
    with pytest.raises(ValueError, match=message):
        make_obs(**overrides).validate()


def test_reflection_pose_is_rejected():
    T = np.diag([1.0, 1.0, -1.0, 1.0])  # orthonormal but det = -1
    with pytest.raises(ValueError, match="determinant"):
        make_obs(T=T).validate()


def test_non_orthonormal_pose_is_rejected():
    T = np.eye(4)
    T[0, 0] = 2.0
    with pytest.raises(ValueError, match="orthonormal"):
        make_obs(T=T).validate()


def test_principal_point_outside_image_is_rejected():
    bad = CameraIntrinsics(width=8, height=6, fx=5.0, fy=5.0, cx=10.0, cy=2.5)
    with pytest.raises(ValueError, match="principal point"):
        make_obs(intrinsics=bad).validate()


def test_T_opt_world_is_rigid_inverse():
    rng = np.random.default_rng(0)
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    T = np.eye(4)
    T[:3, :3] = q
    T[:3, 3] = rng.normal(size=3)
    obs = make_obs(T=T)
    np.testing.assert_allclose(obs.T_opt_world @ T, np.eye(4), atol=1e-12)


# -- integrate / ordering ----------------------------------------------------


def test_integrate_accepts_and_counts():
    b = FakeBackend()
    assert b.integrate(make_obs(stamp_ns=10))
    assert b.integrate(make_obs(stamp_ns=20))
    s = b.get_status()
    assert s.frames_integrated == 2
    assert s.frames_rejected == 0
    assert s.last_stamp_ns == 20


@pytest.mark.parametrize("second_stamp", [10, 5])
def test_out_of_order_or_duplicate_stamp_rejected(second_stamp):
    b = FakeBackend()
    assert b.integrate(make_obs(stamp_ns=10))
    assert not b.integrate(make_obs(stamp_ns=second_stamp))
    s = b.get_status()
    assert s.frames_integrated == 1
    assert s.frames_rejected == 1
    assert "out-of-order" in s.last_rejection_reason
    assert len(b.points) == 1  # backend never saw the rejected frame


def test_invalid_observation_rejected_without_raising():
    b = FakeBackend()
    bad = make_obs(depth=np.full((6, 8), np.nan, dtype=np.float32))
    assert b.integrate(bad) is False
    assert b.get_status().frames_rejected == 1
    assert b.points == []


# -- extract_mesh / versioning -------------------------------------------------


def test_empty_mesh_before_any_integration():
    snap = FakeBackend().extract_mesh()
    assert isinstance(snap, MeshSnapshot)
    assert snap.version == 1
    assert snap.is_empty
    assert snap.stamp_ns == 0
    assert snap.frame_id == "world"


def test_versions_strictly_increase_and_stamp_tracks_last_frame():
    b = FakeBackend()
    for i, stamp in enumerate([100, 200, 300]):
        b.integrate(make_obs(stamp_ns=stamp, frame_index=i))
    s1 = b.extract_mesh()
    b.integrate(make_obs(stamp_ns=400, frame_index=3))
    s2 = b.extract_mesh()
    assert (s1.version, s2.version) == (1, 2)
    assert (s1.stamp_ns, s2.stamp_ns) == (300, 400)
    assert (s1.frames_integrated, s2.frames_integrated) == (3, 4)
    assert s1.vertices.dtype == np.float64 and s1.triangles.dtype == np.int32
    assert s1.num_triangles == 1


def test_extract_uses_pose():
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 3.0]
    b = FakeBackend()
    b.integrate(make_obs(T=T, depth_value=2.0))
    np.testing.assert_allclose(b.extract_mesh().vertices[0], [1.0, 2.0, 5.0])


def test_bad_triangle_indices_raise():
    class Broken(FakeBackend):
        def _extract_impl(self):
            return np.zeros((2, 3)), np.array([[0, 1, 5]])

    with pytest.raises(RuntimeError, match="out of range"):
        Broken().extract_mesh()


# -- reset --------------------------------------------------------------------


def test_reset_clears_state_and_restarts_versions():
    b = FakeBackend()
    b.integrate(make_obs(stamp_ns=100))
    b.extract_mesh()
    b.integrate(make_obs(stamp_ns=50))  # rejected
    b.reset()
    s = b.get_status()
    assert (s.frames_integrated, s.frames_rejected, s.last_mesh_version) == (0, 0, 0)
    assert s.last_stamp_ns is None
    # After reset, an earlier stamp is acceptable again (new mission).
    assert b.integrate(make_obs(stamp_ns=50))
    assert b.extract_mesh().version == 1


# -- concurrency ----------------------------------------------------------------


def test_concurrent_integrate_and_extract_are_consistent():
    b = FakeBackend()
    n = 200

    def producer():
        for i in range(n):
            b.integrate(make_obs(stamp_ns=i + 1, frame_index=i))

    snaps = []

    def consumer():
        for _ in range(50):
            snaps.append(b.extract_mesh())

    threads = [threading.Thread(target=producer), threading.Thread(target=consumer)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert b.get_status().frames_integrated == n
    versions = [s.version for s in snaps]
    assert versions == sorted(versions) and len(set(versions)) == len(versions)
    for s in snaps:  # every snapshot is internally consistent
        assert s.num_vertices == s.frames_integrated


# -- registry -------------------------------------------------------------------


def test_registry_constructs_by_name_with_config():
    assert "fake_test" in registered_backends()
    b = make_backend("fake_test", scale=2.0)
    assert isinstance(b, FakeBackend) and b.scale == 2.0
    assert b.get_status().backend_name == "fake_test"


def test_unknown_backend_name_lists_known():
    with pytest.raises(ValueError, match="fake_test"):
        make_backend("does_not_exist")


def test_abstract_base_cannot_be_instantiated():
    with pytest.raises(TypeError):
        ReconstructionBackend()
