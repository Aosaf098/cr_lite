# Conventions: Frames, Units, and Camera Models

**Purpose:** MuJoCo, ROS 2, Open3D, and the paper's math each use different conventions for cameras, quaternions, and depth. This document fixes one convention for each, and says exactly where conversions happen. Every conversion has a unit test listed in Section 9.

**Rule:** Conversions happen **only at the boundaries** listed here. Inside the project, everything uses the internal conventions in Sections 1–3.

---

## 1. Units

| Quantity | Unit | Notes |
|---|---|---|
| Length | metre (m) | Everywhere, including depth images |
| Angle | radian (rad) | Degrees appear only in human-facing config comments and MJCF `fovy` |
| Time | second (s), simulation time | Wall-clock time is used only for profiling |
| Linear velocity | m/s | |
| Angular velocity | rad/s | |
| Depth image | `32FC1`, metres | Invalid pixels are `NaN` in ROS and `0.0` inside Open3D (Section 5) |

Parameters in YAML configs are in SI units. Any key holding a non-SI value carries a suffix such as `fovy_deg`.

---

## 2. World Frame

- **Frame ID:** `world`.
- Right-handed, **z-up**, with gravity along −z. This is MuJoCo's default and matches ROS REP 103, so the MuJoCo world frame **is** the ROS `world` frame, with no transform between them.
- **Origin:** the centre of the room floor.
  - Room extent: $x, y \in [-2, 2]$ m, $z \in [0, 3]$ m.
- Every mesh, observation point $q_j$, importance field, keep-out box, and ground-truth surface is expressed in `world`.

---

## 3. Drone and Camera Frames

### 3.1 Frame Tree

```text
world
 └── drone_1/base_link              position only (no rotation), from simulator
      └── drone_1/camera_link       gimbal: yaw θh, then pitch θv
           └── drone_1/camera_optical_frame   fixed rotation (REP 103 optical)
```

- **`drone_1/base_link`:** translation $(x, y, z)$ only, with identity rotation. The paper's state has no body yaw, because yaw lives entirely in the gimbal. Since Phase 1 uses ground-truth poses directly, there is **no `odom` frame**.
- **`drone_1/camera_link`:** ROS body convention, with x forward, y left, z up. Its rotation relative to `base_link` is
  $$R = R_z(\theta^h)\,R_y(\theta^v)$$
- **`drone_1/camera_optical_frame`:** OpenCV / ROS optical convention, with **z forward, x right, y down**. All image data and all Open3D extrinsics refer to this frame.

### 3.2 Gimbal Angles (Paper State)

- $\theta^h$ is yaw: rotation about world +z, measured from world +x, counter-clockwise when viewed from above. It is wrapped to $[-\pi, \pi)$.
- $\theta^v$ is pitch, **positive downward**. $\theta^v = \pi/2$ points the camera straight down. The allowed range is $[\pi/3, \pi/2]$ (deviations D11).
- With this convention, the forward axis of `camera_link` is
  $$R_z(\theta^h)R_y(\theta^v)\,\hat e_x = [\cos\theta^h\cos\theta^v,\ \sin\theta^h\cos\theta^v,\ -\sin\theta^v]^\top = \mathrm{Dir}(\theta^h,\theta^v)$$
  This is exactly the `Dir` function of deviations D5. Positive rotation about +y tilts +x toward −z, which is why positive pitch looks down.
- **Singularity:** at $\theta^v = \pi/2$, `Dir` does not depend on $\theta^h$, so $\partial h_1/\partial\theta^h = 0$. This is expected behaviour. The QP simply sees no yaw gradient while looking straight down.

### 3.3 Fixed Transform `camera_link → camera_optical_frame`

The rotation maps optical axes onto link axes as follows:

| Optical axis | Link axis |
|---|---|
| $z_{\mathrm{opt}}$ | $x_{\mathrm{link}}$ |
| $x_{\mathrm{opt}}$ | $-y_{\mathrm{link}}$ |
| $y_{\mathrm{opt}}$ | $-z_{\mathrm{link}}$ |

Quaternion in ROS order $(x, y, z, w)$: $(-0.5,\ 0.5,\ -0.5,\ 0.5)$, equivalent to roll-pitch-yaw $(-\pi/2,\ 0,\ -\pi/2)$. The translation is zero, since the camera sits at the gimbal centre in Phase 1.

---

## 4. Camera Models — The Three Conventions

| System | Forward | Right | Down | Used where |
|---|---|---|---|---|
| MuJoCo camera | −z | +x | −y | Inside the simulator only |
| OpenCV / ROS optical / Open3D | +z | +x | +y | Images, `CameraInfo`, Open3D, all project code |
| ROS body (`camera_link`) | +x | −y | −z | TF tree, gimbal math |

**MuJoCo → optical conversion** is a rotation of π about the camera's x axis:

$$R_{\mathrm{opt}\leftarrow\mathrm{mj}} = \mathrm{diag}(1, -1, -1)$$

It is applied **once, in `cr_mujoco_bridge`**. Nothing downstream ever sees MuJoCo's camera convention.

### 4.1 Intrinsics (Pinhole, No Distortion)

MuJoCo specifies only the vertical field of view `fovy`, and pixels are square:

$$f_y = \frac{H/2}{\tan(\mathrm{fovy}/2)}, \qquad f_x = f_y, \qquad c_x = \frac{W-1}{2}, \qquad c_y = \frac{H-1}{2}$$

The principal point uses OpenCV's convention, where the first pixel's centre is at (0, 0).

**Phase 1 camera (deviations D16):**

| Quantity | Value |
|---|---|
| Resolution $W \times H$ | 320 × 240 |
| `fovy` | 55.2° |
| Horizontal FOV | ≈ 69.7° |
| Diagonal FOV | ≈ 82° (matches the DJI Mini 3) |
| $f_x = f_y$ | ≈ 229.7 px |
| $(c_x, c_y)$ | (159.5, 119.5) |
| `distortion_model` | `plumb_bob`, $D = [0,0,0,0,0]$ |

### 4.2 Pixel Coordinates

- $u$ is the column, increasing to the **right**. $v$ is the row, increasing **downward**. Row 0 is the top of the image.
- NumPy arrays have shape `(H, W)` for depth and `(H, W, 3)` for RGB, indexed `[v, u]`.
- OpenGL stores images bottom-up. The bridge must flip vertically if the MuJoCo API in use returns rows bottom-up. Test T4 catches this either way.

### 4.3 Projection (Optical Frame)

$$u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y$$

**Back-projection** from depth $Z$ at pixel $(u, v)$:

$$X = \frac{(u - c_x)Z}{f_x}, \qquad Y = \frac{(v - c_y)Z}{f_y}$$

---

## 5. Depth

- **Depth means planar z-depth** along the optical axis. It is *not* Euclidean ray length. This is what Open3D's TSDF expects, and what ROS REP 118 specifies.
- **MuJoCo:** the raw OpenGL depth buffer is non-linear. If the low-level API is used, it must be linearized with
  $$z = \frac{z_n}{1 - d\,(1 - z_n/z_f)}$$
  where $z_n$ and $z_f$ are `model.vis.map.znear` and `zfar` **multiplied by `model.stat.extent`**. The high-level Python `mujoco.Renderer` already returns metric depth. Whichever path is used, test T3 is the arbiter.
- **ROS message:** `sensor_msgs/Image`, encoding `32FC1`, metres. Pixels with no return, or outside $[0.2, 5.0]$ m, are set to `NaN`.
- **Open3D input:** a `float32` image with `depth_scale = 1.0` and `depth_trunc = 5.0`. Open3D treats **0 as invalid** and does not reliably handle `NaN`, so the preprocessing node replaces `NaN` with `0.0` before integration.

---

## 6. Transforms and Rotations

### 6.1 Notation

$T_{a\_b}$ is the 4×4 homogeneous transform that maps points **from frame b into frame a**:

$$p_a = T_{a\_b}\,p_b$$

In code, the variable `T_world_cam` is the camera's pose in the world, and its inverse is `T_cam_world`. Transforms are composed by matching inner frames:

$$T_{\mathrm{world\_opt}} = T_{\mathrm{world\_base}}\,T_{\mathrm{base\_link}}\,T_{\mathrm{link\_opt}}$$

### 6.2 Quaternion Order — A Common Bug Source

| Library | Order |
|---|---|
| ROS 2 messages, `tf2` | $(x, y, z, w)$ |
| SciPy `Rotation.from_quat` / `as_quat` (default) | $(x, y, z, w)$ |
| **MuJoCo** (`qpos`, `xquat`, `body_quat`) | **$(w, x, y, z)$** |
| Open3D | Uses rotation matrices; avoid passing quaternions |

All quaternion handling goes through two helper functions in a shared module, `mj_quat_to_ros()` and `ros_quat_to_mj()`. No inline reordering is allowed anywhere else.

### 6.3 Open3D TSDF Extrinsic

`ScalableTSDFVolume.integrate(rgbd, intrinsic, extrinsic)` expects the **world-to-camera** transform in the **optical** convention:

$$\texttt{extrinsic} = T_{\mathrm{opt\_world}} = T_{\mathrm{world\_opt}}^{-1}$$

Passing the camera pose instead of its inverse is the most common TSDF bug. It produces a smeared, mirrored, or empty mesh, and test T6 catches it.

---

## 7. Time and Synchronization

- All nodes run with `use_sim_time: true`. The bridge is the only publisher of `/clock`.
- The bridge renders RGB and depth **at the same simulation step** as the pose it publishes. The RGB image, depth image, `CameraInfo`, and the TF for that step all carry **identical** `header.stamp` values.
- Consumers look up the camera pose with `tf2` **at the image stamp**, never using "latest". If the lookup fails or the stamps differ, the frame is dropped and counted (Reliability metrics).
- **Mesh snapshots** carry the simulation stamp of the last frame integrated before extraction.

---

## 8. Data Layouts and Files

| Item | Layout |
|---|---|
| Observation points | `np.ndarray`, shape `(m, 5)`, `float64`, columns $[x, y, z, \theta^h, \theta^v]$ |
| Importance field $\phi$ | `np.ndarray`, shape `(m,)`, `float64` |
| Drone state $p$ | `np.ndarray`, shape `(5,)`, same column order as above |
| Control input $u$ | `np.ndarray`, shape `(5,)`, $[v_x, v_y, v_z, \dot\theta^h, \dot\theta^v]$ in world frame |
| Keep-out boxes | Axis-aligned in `world`, as `[xmin, ymin, zmin, xmax, ymax, zmax]` |
| Mesh snapshots | Binary PLY in `world` frame, metres |

**Mesh snapshot filenames:** `mesh_{version:06d}_{stamp_ns}.ply`, for example `mesh_000012_0000060000000000.ply`.

Snapshots are written **atomically**: first to `*.ply.tmp`, then renamed with `os.replace`. Readers therefore never see a half-written mesh.

---

## 9. Required Convention Tests

Every boundary conversion gets a test. These live in `tests/` and are implemented with the relevant component.

| ID | Test | Pass condition |
|---|---|---|
| T1 | `Dir` / gimbal | $R_z(\theta^h)R_y(\theta^v)\hat e_x$ equals `Dir` for random angles; $\theta^v = \pi/2$ gives $(0, 0, -1)$ |
| T2 | Optical static transform | Optical $+z$ maps to link $+x$, optical $+x$ to link $-y$, optical $+y$ to link $-z$ |
| T3 | Metric depth | Camera facing a plane at a known distance $d$: median depth $= d \pm 1$ mm, and constant across the image (z-depth, not ray length) |
| T4 | Image orientation | A marker placed *above* the optical axis appears in the **top** half of the image |
| T5 | Projection round trip | Back-project a depth image, then re-project it: pixel error < 0.01 px |
| T6 | TSDF extrinsic | Integrate synthetic views of a known box; the extracted mesh lies within one voxel of the box surface |
| T7 | Quaternion helpers | `ros_quat_to_mj(mj_quat_to_ros(q)) == q`; identity maps to identity |
| T8 | Sensing optimum | The D5 acceptance test: $h_1 = 1$ at the ideal pose |
| T9 | Stamp consistency | RGB, depth, `CameraInfo`, and TF stamps are identical for every published frame |
