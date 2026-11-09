#!/usr/bin/env python3
"""Diagnose the empty-mesh failure in smoke_open3d.py, one stage at a time.

Run from the repo root:  python tests/smoke/diag_open3d.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import smoke_open3d as s  # noqa: E402  (sets MUJOCO_GL=egl)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

model = mujoco.MjModel.from_xml_string(s.XML)
data = mujoco.MjData(model)
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")
renderer = mujoco.Renderer(model, height=s.HEIGHT, width=s.WIDTH)

fy = (s.HEIGHT / 2.0) / np.tan(np.deg2rad(s.FOVY_DEG) / 2.0)
fx, cx, cy = fy, (s.WIDTH - 1) / 2.0, (s.HEIGHT - 1) / 2.0
intrinsic = o3d.camera.PinholeCameraIntrinsic(s.WIDTH, s.HEIGHT, fx, fy, cx, cy)

pos = np.array([s.RADIUS, 0.0, s.CAM_Z])
model.cam_pos[cam_id] = pos
model.cam_quat[cam_id] = s.mujoco_lookat_quat(pos, s.BOX_CENTRE.copy())
mujoco.mj_forward(model, data)

renderer.update_scene(data, camera=cam_id)
rgb = renderer.render().copy()
renderer.enable_depth_rendering()
renderer.update_scene(data, camera=cam_id)
depth = renderer.render().copy()
renderer.disable_depth_rendering()

print("\n--- 1. Raw MuJoCo arrays ---")
print(f"rgb   : {rgb.dtype} {rgb.shape} contiguous={rgb.flags['C_CONTIGUOUS']}")
print(f"depth : {depth.dtype} {depth.shape} min={np.nanmin(depth):.3f} max={np.nanmax(depth):.3f}")

depth[~np.isfinite(depth) | (depth < s.DEPTH_MIN) | (depth > s.DEPTH_MAX)] = 0.0
print(f"valid depth pixels after filtering: {np.count_nonzero(depth)} / {depth.size}")

T_world_cam = s.optical_pose_from_mujoco(data, cam_id)
extrinsic = np.linalg.inv(T_world_cam)
print("\nT_world_cam (camera pose):\n", np.round(T_world_cam, 3))

print("\n--- 2. Open3D RGBDImage ---")
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(np.ascontiguousarray(rgb)),
    o3d.geometry.Image(depth.astype(np.float32)),
    depth_scale=1.0, depth_trunc=s.DEPTH_MAX, convert_rgb_to_intensity=False)
d = np.asarray(rgbd.depth)
c = np.asarray(rgbd.color)
print(f"rgbd.depth: {d.dtype} {d.shape} nonzero={np.count_nonzero(d)} max={d.max():.3f}")
print(f"rgbd.color: {c.dtype} {c.shape}")

print("\n--- 3. Back-projected point cloud (no TSDF) ---")
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic)
P = np.asarray(pcd.points)
print(f"points: {len(P)}")
if len(P):
    print(f"bounds min {np.round(P.min(0), 2)}  max {np.round(P.max(0), 2)}")
    print("(expected: z from ~0 to ~0.6, x/y within a few metres of the origin)")

print("\n--- 4. Legacy ScalableTSDFVolume ---")
vol = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=s.VOXEL, sdf_trunc=s.TRUNC,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
vol.integrate(rgbd, intrinsic, extrinsic)
print(f"voxel points : {len(vol.extract_voxel_point_cloud().points)}")
print(f"mesh vertices: {len(vol.extract_triangle_mesh().vertices)}")

vol_nc = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=s.VOXEL, sdf_trunc=s.TRUNC,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
vol_nc.integrate(rgbd, intrinsic, extrinsic)
print(f"(no colour) mesh vertices: {len(vol_nc.extract_triangle_mesh().vertices)}")

print("\n--- 5. Tensor VoxelBlockGrid (newer API) ---")
try:
    import open3d.core as o3c
    dev = o3c.Device("CPU:0")
    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1), (1), (3)),
        voxel_size=s.VOXEL, block_resolution=16, block_count=50000, device=dev)
    K = o3c.Tensor(intrinsic.intrinsic_matrix, o3c.float64)
    E = o3c.Tensor(extrinsic, o3c.float64)
    dt = o3d.t.geometry.Image(depth.astype(np.float32)).to(dev)
    ct = o3d.t.geometry.Image(rgb.astype(np.float32) / 255.0).to(dev)
    coords = vbg.compute_unique_block_coordinates(dt, K, E, 1.0, s.DEPTH_MAX, s.TRUNC / s.VOXEL)
    vbg.integrate(coords, dt, ct, K, K, E, 1.0, s.DEPTH_MAX, s.TRUNC / s.VOXEL)
    m = vbg.extract_triangle_mesh()
    print(f"blocks: {coords.shape[0]}, mesh vertices: {m.vertex.positions.shape[0]}")
except Exception as exc:  # noqa: BLE001
    print(f"VoxelBlockGrid failed: {type(exc).__name__}: {exc}")

renderer.close()
