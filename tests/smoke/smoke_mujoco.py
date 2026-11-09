#!/usr/bin/env python3
"""Smoke test: MuJoCo offscreen RGB-D rendering on the GPU (EGL).

Checks:
  S1  EGL context is created on the NVIDIA GPU (not llvmpipe software rendering)
  S2  RGB image has the expected shape/dtype and is not blank
  S3  Depth is metric and is z-depth (planar), not ray distance:
      a camera 2 m above a flat floor, looking straight down, must read
      ~2.0 m at the centre AND at the corners.
  S4  Rendering throughput is far above the 5 Hz camera rate

Run from the repo root with the venv active:
    python tests/smoke/smoke_mujoco.py
Images are written to /tmp/cr_smoke/ for visual inspection.
"""
import os
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")  # headless GPU rendering; must be set before import

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

WIDTH, HEIGHT = 320, 240
CAM_HEIGHT_M = 2.0
OUT_DIR = Path("/tmp/cr_smoke")

XML = f"""
<mujoco model="smoke">
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.6 0.6 0.6 1"/>
    <geom name="marker" type="box" pos="1.0 0.6 0.05" size="0.1 0.1 0.05" rgba="1 0 0 1"/>
    <!-- MuJoCo cameras look along their local -z; identity orientation = straight down -->
    <camera name="down" pos="0 0 {CAM_HEIGHT_M}" fovy="60"/>
  </worldbody>
</mujoco>
"""

results = []


def check(name, ok, detail):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def main():
    print(f"MuJoCo {mujoco.__version__}, MUJOCO_GL={os.environ['MUJOCO_GL']}")
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    # --- RGB ---
    renderer.update_scene(data, camera="down")
    rgb = renderer.render()

    # S1: which GPU is the GL context on?
    try:
        from OpenGL import GL
        gl_renderer = GL.glGetString(GL.GL_RENDERER).decode()
        check("S1 GPU context", "llvmpipe" not in gl_renderer.lower()
              and "software" not in gl_renderer.lower(), gl_renderer)
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] S1 could not query GL_RENDERER ({exc}); check nvidia-smi during S4")

    # S2
    check("S2 RGB image",
          rgb.shape == (HEIGHT, WIDTH, 3) and rgb.dtype == np.uint8 and rgb.std() > 1.0,
          f"shape={rgb.shape}, dtype={rgb.dtype}, mean={rgb.mean():.1f}, std={rgb.std():.1f}")

    # --- Depth ---
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="down")
    depth = renderer.render()
    renderer.disable_depth_rendering()

    centre = float(depth[HEIGHT // 2, WIDTH // 2])
    corner = float(depth[2, 2])  # top-left corner: floor, away from the red marker
    # Ray distance at this corner would be ~2.7 m; z-depth stays ~2.0 m.
    check("S3 depth is metric z-depth",
          abs(centre - CAM_HEIGHT_M) < 0.01 and abs(corner - CAM_HEIGHT_M) < 0.01,
          f"dtype={depth.dtype}, centre={centre:.4f} m, corner={corner:.4f} m "
          f"(expected {CAM_HEIGHT_M:.1f} for both)")

    # --- Throughput ---
    n = 200
    t0 = time.perf_counter()
    for _ in range(n):
        renderer.update_scene(data, camera="down")
        renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="down")
        renderer.render()
        renderer.disable_depth_rendering()
    fps = n / (time.perf_counter() - t0)
    check("S4 RGB-D throughput", fps > 50, f"{fps:.0f} RGB-D frames/s at {WIDTH}x{HEIGHT}")

    # --- Save images for a visual look ---
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    import cv2
    cv2.imwrite(str(OUT_DIR / "rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    d = np.clip(depth / depth.max(), 0, 1)
    cv2.imwrite(str(OUT_DIR / "depth.png"), (d * 255).astype(np.uint8))
    print(f"Images saved to {OUT_DIR}/rgb.png and depth.png")

    renderer.close()
    print("\nMUJOCO SMOKE TEST:", "PASSED" if all(results) else "FAILED")


if __name__ == "__main__":
    main()
