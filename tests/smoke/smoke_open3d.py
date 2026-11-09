#!/usr/bin/env python3
"""Smoke test: MuJoCo RGB-D -> Open3D VoxelBlockGrid (tensor TSDF) -> mesh extraction.

Uses the tensor API (o3d.t.geometry.VoxelBlockGrid), not the legacy
pipelines.integration.ScalableTSDFVolume: on Open3D 0.20 the legacy volume
allocated nothing for correct input (see diag_open3d.py), and the tensor
VoxelBlockGrid is the voxel-hashed implementation Open3D now recommends.

This exercises the full camera-convention chain from docs/conventions.md:
  MuJoCo camera (looks along -z, y up)
    -> ROS/Open3D optical frame (looks along +z, y down)
    -> Open3D extrinsic = world-to-camera (the INVERSE of the camera pose)

Scene: a 0.6 m cube on a floor, observed by 12 cameras on a circle.

Checks:
  O1  Projection consistency: a known world point projects to a pixel whose
      depth equals the point's camera-frame z (validates intrinsics + extrinsics)
  O2  TSDF integration latency fits the 2-5 Hz mapping budget
  O3  Mesh extraction produces a non-empty mesh
  O4  Mesh accuracy: vertices lie on the true surfaces (floor + cube)
  O5  The cube is actually reconstructed (top face and a side face present)

Run from the repo root with the venv active:
    python tests/smoke/smoke_open3d.py
The mesh is written to /tmp/cr_smoke/tsdf_mesh.ply.
"""
import os
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

# Settings from ARCHITECTURE.md §4.5
WIDTH, HEIGHT = 320, 240
FOVY_DEG = 60.0
DEPTH_MIN, DEPTH_MAX = 0.2, 5.0
VOXEL = 0.03
TRUNC = 0.09
TRUNC_MULT = TRUNC / VOXEL      # VoxelBlockGrid expresses truncation in voxels
BLOCK_RES = 8                   # 8^3 voxels per block (0.24 m blocks)
BLOCK_COUNT = 20000             # initial hash-map capacity (~200 MB on CPU)
WEIGHT_THRESHOLD = 1.0          # extraction default is 3.0 (needs >=3 observations)
DEVICE = "CPU:0"                # Phase 1: CPU integration (ARCHITECTURE.md §3)

BOX_CENTRE = np.array([0.0, 0.0, 0.3])
BOX_HALF = np.array([0.3, 0.3, 0.3])
N_VIEWS = 12
RADIUS, CAM_Z = 2.0, 1.5
OUT_DIR = Path("/tmp/cr_smoke")

XML = f"""
<mujoco model="smoke_tsdf">
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1"/>
    <light pos="2 2 3" dir="-1 -1 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.6 0.6 0.6 1"/>
    <geom name="cube" type="box" pos="{BOX_CENTRE[0]} {BOX_CENTRE[1]} {BOX_CENTRE[2]}"
          size="{BOX_HALF[0]} {BOX_HALF[1]} {BOX_HALF[2]}" rgba="0.2 0.4 0.8 1"/>
    <camera name="cam" pos="0 0 1" fovy="{FOVY_DEG}"/>
  </worldbody>
</mujoco>
"""

results = []


def check(name, ok, detail):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def mujoco_lookat_quat(pos, target):
    """Quaternion (w, x, y, z) for a MuJoCo camera at pos looking at target.

    MuJoCo camera axes: x right, y up, z BACKWARD (it looks along -z).
    """
    f = target - pos
    f /= np.linalg.norm(f)
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    R = np.column_stack([right, up, -f])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


def optical_pose_from_mujoco(data, cam_id):
    """T_world_optical (4x4) from MuJoCo's camera pose.

    Optical frame = MuJoCo camera frame rotated 180 deg about x
    (flip y and z): looks along +z, y down.
    """
    R_world_mj = data.cam_xmat[cam_id].reshape(3, 3)
    T = np.eye(4)
    T[:3, :3] = R_world_mj @ np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = data.cam_xpos[cam_id]
    return T


def box_sdf(p, centre, half):
    q = np.abs(p - centre) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return outside + inside


def main():
    import open3d.core as o3c
    print(f"Open3D {o3d.__version__}, MuJoCo {mujoco.__version__}, "
          f"device={DEVICE}, CUDA available={o3c.cuda.is_available()}")
    dev = o3c.Device(DEVICE)
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    # Pinhole intrinsics from MuJoCo's vertical FOV (square pixels).
    # Principal point (W-1)/2: integer pixel indices refer to pixel centres.
    fy = (HEIGHT / 2.0) / np.tan(np.deg2rad(FOVY_DEG) / 2.0)
    fx = fy
    cx, cy = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
    K_np = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    K = o3c.Tensor(K_np, o3c.float64)

    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1,), (1,), (3,)),
        voxel_size=VOXEL,
        block_resolution=BLOCK_RES,
        block_count=BLOCK_COUNT,
        device=dev,
    )

    probe_world = np.array([0.0, 0.0, 0.6, 1.0])  # centre of the cube's top face
    proj_errors = []
    integrate_ms = []
    target = BOX_CENTRE.copy()

    for k in range(N_VIEWS):
        a = 2.0 * np.pi * k / N_VIEWS
        pos = np.array([RADIUS * np.cos(a), RADIUS * np.sin(a), CAM_Z])
        model.cam_pos[cam_id] = pos
        model.cam_quat[cam_id] = mujoco_lookat_quat(pos, target)
        mujoco.mj_forward(model, data)

        renderer.update_scene(data, camera=cam_id)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=cam_id)
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()

        T_world_cam = optical_pose_from_mujoco(data, cam_id)
        extrinsic = np.linalg.inv(T_world_cam)  # Open3D wants world-to-camera

        # O1: project the probe point and compare with the rendered depth
        pc = extrinsic @ probe_world
        u = int(round(fx * pc[0] / pc[2] + cx))
        v = int(round(fy * pc[1] / pc[2] + cy))
        proj_errors.append(abs(float(depth[v, u]) - pc[2]))

        # Invalid / out-of-range depth -> 0 (Open3D ignores zeros)
        depth[~np.isfinite(depth) | (depth < DEPTH_MIN) | (depth > DEPTH_MAX)] = 0.0
        depth_t = o3d.t.geometry.Image(np.ascontiguousarray(depth, dtype=np.float32)).to(dev)
        color_t = o3d.t.geometry.Image(np.ascontiguousarray(rgb, dtype=np.float32) / 255.0).to(dev)
        E = o3c.Tensor(extrinsic, o3c.float64)

        t0 = time.perf_counter()
        coords = vbg.compute_unique_block_coordinates(depth_t, K, E, 1.0, DEPTH_MAX, TRUNC_MULT)
        vbg.integrate(coords, depth_t, color_t, K, K, E, 1.0, DEPTH_MAX, TRUNC_MULT)
        integrate_ms.append(1000.0 * (time.perf_counter() - t0))

    renderer.close()

    check("O1 projection consistency", max(proj_errors) < 0.02,
          f"max |depth - z_cam| over {N_VIEWS} views = {max(proj_errors) * 1000:.1f} mm")
    check("O2 integration latency", np.mean(integrate_ms) < 200.0,
          f"mean {np.mean(integrate_ms):.1f} ms, max {np.max(integrate_ms):.1f} ms per frame")

    t0 = time.perf_counter()
    mesh = vbg.extract_triangle_mesh(weight_threshold=WEIGHT_THRESHOLD).to_legacy()
    extract_ms = 1000.0 * (time.perf_counter() - t0)
    V = np.asarray(mesh.vertices)
    check("O3 mesh extraction", len(V) > 1000 and len(mesh.triangles) > 1000,
          f"{len(V)} vertices, {len(mesh.triangles)} triangles in {extract_ms:.0f} ms")
    if len(V) == 0:
        print("\nOPEN3D SMOKE TEST: FAILED (empty mesh)")
        return

    # O4: distance from each vertex to the nearest true surface (floor z=0, or cube)
    dist = np.minimum(np.abs(V[:, 2]), np.abs(box_sdf(V, BOX_CENTRE, BOX_HALF)))
    med, p95 = np.median(dist), np.percentile(dist, 95)
    check("O4 mesh accuracy", med < VOXEL / 2 and p95 < VOXEL,
          f"median {med * 1000:.1f} mm, 95th pct {p95 * 1000:.1f} mm "
          f"(limits {VOXEL / 2 * 1000:.0f} / {VOXEL * 1000:.0f} mm)")

    # O5: is the cube there?
    top = np.sum((np.abs(V[:, 2] - 0.6) < VOXEL) & (np.abs(V[:, 0]) < 0.25) & (np.abs(V[:, 1]) < 0.25))
    side = np.sum((np.abs(V[:, 0] - 0.3) < VOXEL) & (np.abs(V[:, 1]) < 0.25) & (V[:, 2] > 0.1) & (V[:, 2] < 0.5))
    check("O5 cube reconstructed", top > 50 and side > 50,
          f"{top} vertices on top face, {side} on +x side face")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(OUT_DIR / "tsdf_mesh.ply"), mesh)
    print(f"Mesh saved to {OUT_DIR / 'tsdf_mesh.ply'}")

    print("\nOPEN3D SMOKE TEST:", "PASSED" if all(results) else "FAILED")


if __name__ == "__main__":
    main()
