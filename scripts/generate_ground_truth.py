#!/usr/bin/env python3
"""Export an evaluation-only furniture reference point cloud from MuJoCo MJCF.

Run from the repository root:
    python scripts/generate_ground_truth.py

The cloud samples each face of every selected box geom at <= 1 cm spacing.
It intentionally excludes room walls, floor, and the drone, in accordance
with the furniture-only target region in Stage 3.2. It is an oracle used
ONLY for evaluation, never for mapping, mesh feedback, or control.
"""

import json
from pathlib import Path

import mujoco
import numpy as np
import open3d as o3d

REPO = Path(__file__).resolve().parents[1]
SCENE = REPO / "environments" / "mjcf" / "indoor_room.xml"
OUTPUT_DIR = REPO / "experiments" / "evaluation"
SPACING_M = 0.01
TARGET_B = np.array([-1.75, -1.75, 0.0, 1.75, 1.75, 1.5])
GEOM_NAMES = [
    "table_top",
    "table_leg_1",
    "table_leg_2",
    "table_leg_3",
    "table_leg_4",
    "cabinet",
    "bench",
]


def sample_box_faces(center, rotation, half_extents, spacing):
    """Return box surface points in world coordinates on six regular grids."""
    faces = []
    for fixed_axis in range(3):
        ax0, ax1 = [a for a in range(3) if a != fixed_axis]
        n0 = int(np.ceil(2 * half_extents[ax0] / spacing)) + 1
        n1 = int(np.ceil(2 * half_extents[ax1] / spacing)) + 1
        a = np.linspace(-half_extents[ax0], half_extents[ax0], n0)
        b = np.linspace(-half_extents[ax1], half_extents[ax1], n1)
        A, B = np.meshgrid(a, b, indexing="ij")
        for sign in (-1.0, 1.0):
            local = np.zeros((A.size, 3), dtype=np.float64)
            local[:, fixed_axis] = sign * half_extents[fixed_axis]
            local[:, ax0] = A.ravel()
            local[:, ax1] = B.ravel()
            # MuJoCo geom_xmat is local->world; points are row vectors.
            faces.append(local @ rotation.T + center)
    return np.concatenate(faces, axis=0)


def main():
    assert SCENE.is_file(), f"Missing scene: {SCENE}"
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    all_points, all_colors = [], []
    raw_counts = {}
    for name in GEOM_NAMES:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            raise RuntimeError(f"Expected furniture geometry is missing: {name}")
        if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_BOX:
            raise RuntimeError(f"Expected box geometry for {name}")

        xyz = sample_box_faces(
            np.asarray(data.geom_xpos[gid]),
            np.asarray(data.geom_xmat[gid]).reshape(3, 3),
            np.asarray(model.geom_size[gid])[:3],
            SPACING_M,
        )
        rgb = np.asarray(model.geom_rgba[gid, :3], dtype=np.float64)
        all_points.append(xyz)
        all_colors.append(np.tile(rgb, (len(xyz), 1)))
        raw_counts[name] = int(len(xyz))

    points = np.concatenate(all_points)
    colors = np.concatenate(all_colors)

    # Deduplicate shared face edges and corners to avoid overweighting them.
    _, unique_indices = np.unique(
        np.round(points, decimals=6), axis=0, return_index=True
    )
    unique_indices.sort()
    points, colors = points[unique_indices], colors[unique_indices]

    mins, maxs = points.min(axis=0), points.max(axis=0)
    if np.any(mins < TARGET_B[:3] - 1e-6) or np.any(maxs > TARGET_B[3:] + 1e-6):
        raise RuntimeError("Furniture geometry is outside target region B")
    if not np.isfinite(points).all() or len(points) < 1000:
        raise RuntimeError("Generated point cloud is empty or invalid")

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    ply_path = OUTPUT_DIR / "ground_truth_furniture.ply"
    json_path = OUTPUT_DIR / "ground_truth_furniture.json"
    if not o3d.io.write_point_cloud(str(ply_path), cloud, write_ascii=False):
        raise RuntimeError(f"Could not write point cloud: {ply_path}")

    metadata = {
        "scene": str(SCENE.relative_to(REPO)),
        "purpose": "evaluation_only",
        "geometry": "furniture_only_no_floor_walls_drone",
        "spacing_m_max": SPACING_M,
        "target_region_B": TARGET_B.tolist(),
        "point_count": int(len(points)),
        "bounds_min_m": mins.tolist(),
        "bounds_max_m": maxs.tolist(),
        "source_geom_names": GEOM_NAMES,
        "per_geom_count_before_dedup": raw_counts,
        "known_limitation": "Includes every box face, even surfaces that may be hidden or physically unobservable.",
    }
    json_path.write_text(json.dumps(metadata, indent=2) + "\n")

    print(f"[PASS] Furniture geoms found: {len(GEOM_NAMES)}")
    print(f"[PASS] 1 cm surface sampling: {len(points):,} points")
    print(f"[PASS] Within target region B: min={np.round(mins, 3)}, max={np.round(maxs, 3)}")
    print(f"Saved point cloud: {ply_path}")
    print(f"Saved metadata:    {json_path}")
    print("GROUND-TRUTH GEOMETRY TEST: PASSED")


if __name__ == "__main__":
    main()
