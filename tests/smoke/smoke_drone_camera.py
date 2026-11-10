import os
os.environ["MUJOCO_GL"] = "egl"

from pathlib import Path
import mujoco
import numpy as np
import cv2

model = mujoco.MjModel.from_xml_path(
    "environments/mjcf/indoor_room.xml"
)
data = mujoco.MjData(model)

camera = "drone_1_rgbd"
cam_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_CAMERA, camera
)
assert cam_id >= 0

# Initial camera: yaw=0, pitch=60 degrees.
yaw_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "drone_1_yaw_joint"
)
pitch_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "drone_1_pitch_joint"
)
data.qpos[model.jnt_qposadr[yaw_id]] = 0.0
data.qpos[model.jnt_qposadr[pitch_id]] = np.pi / 3
mujoco.mj_forward(model, data)

# Hide the drone's three visual geometries.
for name in ("drone_1_visual", "gimbal_yaw_mount", "camera_mount"):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert gid >= 0 and model.geom_group[gid] == 1, name

options = mujoco.MjvOption()
options.geomgroup[1] = 0

renderer = mujoco.Renderer(model, height=240, width=320)

renderer.update_scene(data, camera=camera, scene_option=options)
rgb = renderer.render().copy()

renderer.enable_depth_rendering()
renderer.update_scene(data, camera=camera, scene_option=options)
depth = renderer.render().copy()
renderer.disable_depth_rendering()
renderer.close()

assert rgb.shape == (240, 320, 3)
assert rgb.dtype == np.uint8
assert depth.shape == (240, 320)
assert depth.dtype == np.float32

valid = np.isfinite(depth) & (depth >= 0.2) & (depth <= 5.0)
assert valid.any(), "No valid depth pixels"

out = Path("/tmp/cr_smoke")
out.mkdir(parents=True, exist_ok=True)

cv2.imwrite(str(out / "drone_rgb.png"),
            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

preview = np.zeros(depth.shape, dtype=np.uint8)
preview[valid] = (
    255 * (5.0 - depth[valid]) / 4.8
).clip(0, 255).astype(np.uint8)

cv2.imwrite(str(out / "drone_depth.png"), preview)
np.save(out / "drone_depth.npy", depth)

print("[PASS] RGB image:", rgb.shape, rgb.dtype)
print("[PASS] Depth image:", depth.shape, depth.dtype)
print(f"Valid depth pixels: {valid.mean()*100:.1f}%")
print(f"Depth range: {depth[valid].min():.3f} to "
      f"{depth[valid].max():.3f} m")
print("Images saved to /tmp/cr_smoke/")
print("DRONE RGB-D RENDER TEST: PASSED")
