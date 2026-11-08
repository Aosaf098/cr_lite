"""Backend-agnostic reconstruction interface.

ARCHITECTURE.md §4.4: the rest of the system talks to reconstruction only
through ``ReconstructionBackend``. Phase 1 has one implementation
(Open3D TSDF); a NeuralRecon backend could be added in Phase 2 without
touching the mapper node, mesh feedback, or the controller.

Design: template method. The public methods (``integrate``, ``extract_mesh``,
``reset``, ``get_status``) live in this base class and enforce the contract
for every backend:

* inputs are validated against docs/conventions.md before a backend sees them;
* observations must arrive in strictly increasing stamp order;
* mesh versions start at 1 and strictly increase until ``reset``;
* a snapshot's stamp is the stamp of the last integrated observation;
* integration and extraction never run at the same time (one lock).

Subclasses implement only the ``_*_impl`` hooks.

This module is deliberately ROS-free (NumPy only) so it can be unit-tested
without a ROS installation.
"""

from __future__ import annotations

import abc
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple, Type

import numpy as np

WORLD_FRAME = "world"


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics in the OpenCV/optical convention (conventions.md §4.1)."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"image size must be positive, got {self.width}x{self.height}")
        if not (self.fx > 0 and self.fy > 0):
            raise ValueError(f"focal lengths must be positive, got fx={self.fx}, fy={self.fy}")
        if not (0 <= self.cx < self.width and 0 <= self.cy < self.height):
            raise ValueError(
                f"principal point ({self.cx}, {self.cy}) lies outside the "
                f"{self.width}x{self.height} image"
            )

    def as_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class Observation:
    """One calibrated, time-consistent RGB-D frame.

    Attributes:
        stamp_ns: Simulation time of acquisition, nanoseconds.
        frame_index: Monotonic counter since the last reset.
        depth: (H, W) float32 planar z-depth in metres. Invalid pixels are 0.0;
            NaN is not allowed here (conventions.md §5).
        intrinsics: Camera intrinsics matching ``depth``.
        T_world_opt: (4, 4) pose of the optical frame in ``world``
            (maps optical-frame points into world, conventions.md §6.1).
        rgb: Optional (H, W, 3) uint8 colour image.
    """

    stamp_ns: int
    frame_index: int
    depth: np.ndarray
    intrinsics: CameraIntrinsics
    T_world_opt: np.ndarray
    rgb: Optional[np.ndarray] = None

    def validate(self, rotation_tol: float = 1e-6) -> None:
        """Raise ``ValueError`` if this observation violates conventions.md."""
        if self.stamp_ns < 0:
            raise ValueError(f"stamp_ns must be non-negative, got {self.stamp_ns}")

        self.intrinsics.validate()
        h, w = self.intrinsics.height, self.intrinsics.width

        if self.depth.shape != (h, w):
            raise ValueError(f"depth shape {self.depth.shape} does not match intrinsics ({h}, {w})")
        if self.depth.dtype != np.float32:
            raise ValueError(f"depth must be float32 metres, got {self.depth.dtype}")
        if not np.all(np.isfinite(self.depth)):
            raise ValueError("depth contains NaN/inf; invalid pixels must be 0.0 (conventions.md §5)")
        if np.any(self.depth < 0):
            raise ValueError("depth contains negative values")

        if self.rgb is not None:
            if self.rgb.shape != (h, w, 3):
                raise ValueError(f"rgb shape {self.rgb.shape} does not match ({h}, {w}, 3)")
            if self.rgb.dtype != np.uint8:
                raise ValueError(f"rgb must be uint8, got {self.rgb.dtype}")

        T = self.T_world_opt
        if T.shape != (4, 4):
            raise ValueError(f"T_world_opt must be 4x4, got {T.shape}")
        if not np.all(np.isfinite(T)):
            raise ValueError("T_world_opt contains NaN/inf")
        if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=rotation_tol):
            raise ValueError("T_world_opt bottom row must be [0, 0, 0, 1]")
        R = T[:3, :3]
        if not np.allclose(R.T @ R, np.eye(3), atol=rotation_tol):
            raise ValueError("T_world_opt rotation is not orthonormal")
        if abs(np.linalg.det(R) - 1.0) > rotation_tol:
            raise ValueError("T_world_opt rotation has determinant != +1 (reflection?)")

    @property
    def T_opt_world(self) -> np.ndarray:
        """World-to-camera transform, the form Open3D's ``integrate`` expects
        (conventions.md §6.3). Rigid inverse, not ``np.linalg.inv``."""
        R = self.T_world_opt[:3, :3]
        t = self.T_world_opt[:3, 3]
        T = np.eye(4)
        T[:3, :3] = R.T
        T[:3, 3] = -R.T @ t
        return T


@dataclass(frozen=True)
class MeshSnapshot:
    """A versioned triangle mesh in the ``world`` frame (ARCHITECTURE.md §4.5).

    Writing it to disk (atomic PLY, conventions.md §8) is the mapper node's
    job, not the backend's.
    """

    version: int
    stamp_ns: int
    vertices: np.ndarray            # (N, 3) float64, metres, world frame
    triangles: np.ndarray           # (M, 3) int32 vertex indices
    frames_integrated: int
    extraction_time_s: float        # wall-clock
    frame_id: str = WORLD_FRAME

    @property
    def num_vertices(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def num_triangles(self) -> int:
        return int(self.triangles.shape[0])

    @property
    def is_empty(self) -> bool:
        return self.num_vertices == 0


@dataclass(frozen=True)
class BackendStatus:
    """Runtime and mapping status for logging and ``get_status``."""

    backend_name: str
    frames_integrated: int
    frames_rejected: int
    last_stamp_ns: Optional[int]
    last_mesh_version: int
    mean_integration_time_s: float  # wall-clock, over accepted frames since reset
    last_rejection_reason: Optional[str]


# ---------------------------------------------------------------------------
# Interface
# ---------------------------------------------------------------------------


class ReconstructionBackend(abc.ABC):
    """Base class every reconstruction backend derives from.

    Subclasses implement:
        ``_integrate_impl(obs)``  — fuse one already-validated observation.
        ``_extract_impl()``       — return ``(vertices, triangles)``.
        ``_reset_impl()``         — clear all reconstruction state.

    Subclasses must not override the public methods.
    """

    #: Short identifier used in logs and the backend registry.
    name: str = "abstract"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clear_bookkeeping()

    # -- public API (ARCHITECTURE.md §4.4) ---------------------------------

    def integrate(self, observation: Observation) -> bool:
        """Validate and fuse one observation.

        Returns ``True`` if integrated. Returns ``False`` (and counts the
        rejection) for an invalid or out-of-order observation; never raises
        for bad input, so one bad frame cannot stop a mission.
        """
        try:
            observation.validate()
        except ValueError as exc:
            return self._reject(f"invalid observation: {exc}")

        with self._lock:
            if self._last_stamp_ns is not None and observation.stamp_ns <= self._last_stamp_ns:
                return self._reject_locked(
                    f"out-of-order stamp {observation.stamp_ns} <= last {self._last_stamp_ns}"
                )
            t0 = time.perf_counter()
            self._integrate_impl(observation)
            self._integration_time_total += time.perf_counter() - t0
            self._frames_integrated += 1
            self._last_stamp_ns = observation.stamp_ns
        return True

    def extract_mesh(self) -> MeshSnapshot:
        """Return a new versioned mesh snapshot.

        Before any frame is integrated this returns an empty mesh with
        ``stamp_ns == 0``; callers decide whether empty meshes are useful.
        """
        with self._lock:
            t0 = time.perf_counter()
            vertices, triangles = self._extract_impl()
            elapsed = time.perf_counter() - t0

            vertices = np.ascontiguousarray(vertices, dtype=np.float64).reshape(-1, 3)
            triangles = np.ascontiguousarray(triangles, dtype=np.int32).reshape(-1, 3)
            if triangles.size and (triangles.min() < 0 or triangles.max() >= len(vertices)):
                raise RuntimeError(f"{self.name}: triangle indices out of range")

            self._mesh_version += 1
            return MeshSnapshot(
                version=self._mesh_version,
                stamp_ns=self._last_stamp_ns if self._last_stamp_ns is not None else 0,
                vertices=vertices,
                triangles=triangles,
                frames_integrated=self._frames_integrated,
                extraction_time_s=elapsed,
            )

    def reset(self) -> None:
        """Clear reconstruction state. The next snapshot is version 1."""
        with self._lock:
            self._reset_impl()
            self._clear_bookkeeping()

    def get_status(self) -> BackendStatus:
        with self._lock:
            n = self._frames_integrated
            return BackendStatus(
                backend_name=self.name,
                frames_integrated=n,
                frames_rejected=self._frames_rejected,
                last_stamp_ns=self._last_stamp_ns,
                last_mesh_version=self._mesh_version,
                mean_integration_time_s=self._integration_time_total / n if n else 0.0,
                last_rejection_reason=self._last_rejection_reason,
            )

    # -- hooks for subclasses -----------------------------------------------

    @abc.abstractmethod
    def _integrate_impl(self, observation: Observation) -> None:
        """Fuse one observation. Called with the lock held, input already validated."""

    @abc.abstractmethod
    def _extract_impl(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return ``(vertices (N,3), triangles (M,3))`` in the world frame.

        Must not modify the reconstruction state. Called with the lock held.
        """

    @abc.abstractmethod
    def _reset_impl(self) -> None:
        """Discard all reconstruction state. Called with the lock held."""

    # -- internals -----------------------------------------------------------

    def _clear_bookkeeping(self) -> None:
        self._frames_integrated = 0
        self._frames_rejected = 0
        self._last_stamp_ns: Optional[int] = None
        self._mesh_version = 0
        self._integration_time_total = 0.0
        self._last_rejection_reason: Optional[str] = None

    def _reject(self, reason: str) -> bool:
        with self._lock:
            return self._reject_locked(reason)

    def _reject_locked(self, reason: str) -> bool:
        self._frames_rejected += 1
        self._last_rejection_reason = reason
        return False


# ---------------------------------------------------------------------------
# Registry: select a backend by name from config (e.g. mapping.yaml: backend: open3d_tsdf)
# ---------------------------------------------------------------------------

_REGISTRY: Dict[str, Type[ReconstructionBackend]] = {}


def register_backend(name: str) -> Callable[[Type[ReconstructionBackend]], Type[ReconstructionBackend]]:
    """Class decorator that makes a backend constructible by ``make_backend``."""

    def decorator(cls: Type[ReconstructionBackend]) -> Type[ReconstructionBackend]:
        if not issubclass(cls, ReconstructionBackend):
            raise TypeError(f"{cls.__name__} is not a ReconstructionBackend")
        if name in _REGISTRY and _REGISTRY[name] is not cls:
            raise ValueError(f"backend name '{name}' is already registered")
        cls.name = name
        _REGISTRY[name] = cls
        return cls

    return decorator


def make_backend(name: str, **config) -> ReconstructionBackend:
    """Construct a registered backend, passing ``config`` to its constructor."""
    try:
        cls = _REGISTRY[name]
    except KeyError:
        known = ", ".join(sorted(_REGISTRY)) or "(none)"
        raise ValueError(f"unknown reconstruction backend '{name}'; registered: {known}") from None
    return cls(**config)


def registered_backends() -> Tuple[str, ...]:
    return tuple(sorted(_REGISTRY))
