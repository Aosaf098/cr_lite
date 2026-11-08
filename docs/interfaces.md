# ROS 2 Interfaces

**Purpose:** Defines every node, topic, message, service, and QoS profile in Phase 1, and which node owns which piece of state.

**Related documents:** Frames, units, and stamps follow `docs/conventions.md`. Algorithm references such as "Eq. (4)" and "D5" point to `docs/paper_spec.md` and `docs/deviations.md`.

---

## 1. Node Graph

```text
                         /clock, /tf, /tf_static
 ┌──────────────────┐   /drone_1/camera/{rgb,depth,camera_info}
 │  mujoco_bridge   │──────────────────────────────┐
 │ (cr_mujoco_bridge)│  /drone_1/state, /drone_1/odom│
 └────────▲─────────┘                              ▼
          │ /drone_1/cmd_vel          ┌────────────────────────┐
 ┌────────┴─────────┐                 │   rgbd_preprocessor    │
 │  motion_executor │                 │  (cr_rgbd_processing)  │
 │(cr_motion_control)│                └───────────┬────────────┘
 └────────▲─────────┘                             │ /drone_1/rgbd/observation
          │ /drone_1/cmd_desired                  ▼
 ┌────────┴─────────────┐            ┌────────────────────────┐
 │ coverage_controller  │            │      tsdf_mapper       │
 │ (cr_coverage_control)│            │   (cr_tsdf_mapping)    │
 │  owns φ, J, QP       │            └───────────┬────────────┘
 └────────▲─────────────┘                        │ /mapping/mesh_snapshot
          │ /feedback/mesh_change                ▼
          │                          ┌────────────────────────┐
          └──────────────────────────│     mesh_feedback      │
                                     │   (cr_mesh_feedback)   │
                                     └────────────────────────┘

 experiment_runner (cr_evaluation): calls services, records rosbag2, writes CSV
```

---

## 2. Naming

- **Per-drone topics** use the prefix `/drone_1/`. Every node that handles drone data takes a `drone_ns` parameter, so adding drones later needs no renaming.
- **System-wide topics** use the prefixes `/mapping/`, `/feedback/`, and `/coverage/`.
- **Node names:**

| Node | Package |
|---|---|
| `mujoco_bridge` | `cr_mujoco_bridge` |
| `rgbd_preprocessor` | `cr_rgbd_processing` |
| `tsdf_mapper` | `cr_tsdf_mapping` |
| `mesh_feedback` | `cr_mesh_feedback` |
| `coverage_controller` | `cr_coverage_control` |
| `motion_executor` | `cr_motion_control` |
| `experiment_runner` | `cr_evaluation` |

---

## 3. Topics

| Topic | Type | Publisher → Subscribers | Rate | QoS |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | bridge → all | every physics step | default clock |
| `/tf` | `tf2_msgs/TFMessage` | bridge → all | 20–50 Hz | standard tf |
| `/tf_static` | `tf2_msgs/TFMessage` | bridge → all | once | standard tf_static |
| `/drone_1/state` | `cr_interfaces/DroneState5D` | bridge → controller, motion, runner | 20–50 Hz | `CONTROL` |
| `/drone_1/odom` | `nav_msgs/Odometry` | bridge → RViz, runner | 20–50 Hz | `SENSOR` |
| `/drone_1/camera/rgb` | `sensor_msgs/Image` (`rgb8`) | bridge → preprocessor | 5 Hz | `SENSOR` |
| `/drone_1/camera/depth` | `sensor_msgs/Image` (`32FC1`, m) | bridge → preprocessor | 5 Hz | `SENSOR` |
| `/drone_1/camera/camera_info` | `sensor_msgs/CameraInfo` | bridge → preprocessor, RViz | 5 Hz | `SENSOR` |
| `/drone_1/rgbd/observation` | `cr_interfaces/RGBDObservation` | preprocessor → mapper | 2–5 Hz | `SENSOR` |
| `/mapping/mesh_snapshot` | `cr_interfaces/MeshSnapshot` | mapper → feedback, runner | every 5 s | `LATCHED` |
| `/feedback/mesh_change` | `cr_interfaces/MeshFeedback` | feedback → controller, runner | per snapshot | `EVENT` |
| `/drone_1/cmd_desired` | `cr_interfaces/VelocityCommand5D` | controller → motion | 5 Hz | `CONTROL` |
| `/drone_1/cmd_vel` | `cr_interfaces/VelocityCommand5D` | motion → bridge | 5 Hz (held) | `CONTROL` |
| `/coverage/status` | `cr_interfaces/CoverageStatus` | controller → runner | 5 Hz | `EVENT` |
| `/drone_1/motion/status` | `cr_interfaces/MotionStatus` | motion → runner | 5 Hz | `EVENT` |
| `/coverage/importance_cloud` | `sensor_msgs/PointCloud2` | controller → RViz | 1 Hz | `SENSOR` |
| `/mapping/mesh_marker` | `visualization_msgs/Marker` | mapper → RViz | per snapshot | `LATCHED` |

**Visualization:** `importance_cloud` holds one point per unique observation **position**. Each point has two fields, `phi_mean` and `phi_max`, aggregated over that position's viewing angles.

---

## 4. Custom Messages (`cr_interfaces/msg`)

### `DroneState5D.msg`

```text
# Paper state p = [x, y, z, θh, θv]. Frame: world. See conventions.md §3.2.
std_msgs/Header header
float64 x
float64 y
float64 z
float64 yaw      # θh, rad, wrapped to [-pi, pi)
float64 pitch    # θv, rad, positive = camera tilted down
```

### `VelocityCommand5D.msg`

```text
# Paper input u = d/dt p. Linear part in world frame; angular parts are gimbal rates.
std_msgs/Header header
float64 vx           # m/s
float64 vy           # m/s
float64 vz           # m/s
float64 yaw_rate     # rad/s, d(θh)/dt
float64 pitch_rate   # rad/s, d(θv)/dt (positive = tilt down)
```

### `RGBDObservation.msg`

```text
# A validated, time-consistent frame ready for TSDF integration.
std_msgs/Header header             # stamp = acquisition sim time; frame_id = camera optical frame
sensor_msgs/Image rgb              # rgb8
sensor_msgs/Image depth            # 32FC1, metres; invalid pixels = 0.0 (Open3D convention, conventions.md §5)
sensor_msgs/CameraInfo camera_info
geometry_msgs/Pose camera_pose     # T_world_opt at header.stamp
uint32 frame_index                 # monotonic since reset
float32 valid_depth_fraction
```

### `MeshSnapshot.msg`

```text
# Metadata only. The mesh itself is a PLY file on disk (conventions.md §8).
std_msgs/Header header             # stamp = sim time of last integrated frame; frame_id = world
uint32 version                     # strictly increasing since reset, starts at 1
string path                        # absolute path to the PLY file
uint32 num_vertices
uint32 num_triangles
uint32 frames_integrated           # total frames in the volume
float32 extraction_time_s          # wall-clock
```

### `MeshFeedback.msg`

```text
# h2 from the 3D Grid method, Eq. (6). h2 depends only on Pos(q_j), so it is sent once per unique position.
std_msgs/Header header             # stamp = stamp of the newer mesh
uint32 mesh_version
uint32 prev_mesh_version
string field_id                    # hash of the observation-field config, must match the controller's
float32[] h2_per_position          # length = number of unique observation positions
float32 h2_max
float32 h2_sum
uint32 grid_points_changed         # count of r_k with ΔV != 0
float32 compute_time_s             # wall-clock
```

### `CoverageStatus.msg`

```text
uint8 SOLVER_OK=0
uint8 SOLVER_INACCURATE=1
uint8 SOLVER_INFEASIBLE=2
uint8 SOLVER_ERROR=3
uint8 SOLVER_SKIPPED_STALE_STATE=4

std_msgs/Header header
float64 objective_j                # J, Eq. (3)
bool feedback_enabled              # experiment setting (on/off)
bool feedback_active               # δ2 > 0 right now, Eq. (23)
uint32 last_applied_mesh_version
float64 last_feedback_age_s        # sim time since last applied feedback
uint8 solver_status
float64 solve_time_s               # wall-clock
float64 slack_w                    # w in QP (19)
float64 performance_barrier        # b_I, Eq. (12)
float64[] active_safety_margins    # min barrier value per hard-constraint group
```

### `MotionStatus.msg`

```text
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_TIMEOUT_HOLD=2
uint8 STATE_ESTOP=3
uint8 STATE_COLLISION_STOP=4

std_msgs/Header header
uint8 state
bool command_clamped               # last command exceeded limits and was clamped
float64 command_age_s              # sim time since last received command
```

---

## 5. Services

All services use `std_srvs/srv/Trigger`, with failures reported in `message`.

| Service | Server | Effect |
|---|---|---|
| `/mujoco_bridge/reset` | bridge | Restores the initial pose and gimbal from config, with the trial seed |
| `/tsdf_mapper/reset` | mapper | Clears the TSDF volume and resets `version` to 0 |
| `/mesh_feedback/reset` | feedback | Forgets the previous mesh |
| `/coverage_controller/reset` | controller | Sets $\phi_j = 1$ and $J = 1$; clears feedback history |
| `/coverage_controller/start` | controller | Begins publishing commands |
| `/coverage_controller/stop` | controller | Publishes zero commands and stops |
| `/drone_1/motion/estop` | motion | Latches zero velocity until reset |
| `/drone_1/motion/reset` | motion | Clears estop and collision state |

**Reset order** (`experiment_runner`):

1. controller stop
2. motion reset
3. bridge reset
4. mapper reset
5. feedback reset
6. controller reset
7. controller start

**Feedback on/off** is a **launch parameter** (`feedback_enabled`) of the controller, not a runtime service. An experiment's condition is therefore fixed at launch and recorded in its config.

---

## 6. QoS Profiles

| Profile | Reliability | Durability | History | Used for |
|---|---|---|---|---|
| `SENSOR` | reliable | volatile | keep last 5 | Images, observations, odom |
| `CONTROL` | reliable | volatile | keep last 1 | State and commands; only the newest value matters |
| `EVENT` | reliable | volatile | keep last 20 | Feedback and status; must not be dropped |
| `LATCHED` | reliable | transient local | keep last 1 | Mesh snapshot and marker; late joiners get the latest |

Sensors are reliable rather than best-effort, because everything runs on one machine and silently dropped frames would bias the experiments. Dropped frames are counted explicitly instead.

---

## 7. Ownership Rules

1. **The importance field $\phi$ has exactly one writer: `coverage_controller`.**
   - It integrates decay (Eq. 2) every tick.
   - It applies jumps (Eq. 4) when a `MeshFeedback` arrives.
   - The feedback-activation gate ($\delta_2$, Eq. 23) lives here, because only the controller knows $J$.
   - `mesh_feedback` only computes $h_2$; it never touches $\phi$.
2. **The controller rejects a `MeshFeedback`** if any of these hold:
   - `field_id` does not match its own;
   - `mesh_version` ≤ `last_applied_mesh_version` (out of order or duplicate);
   - `prev_mesh_version` ≠ `mesh_version − 1`. In this case it is applied **only** if a config flag allows gaps, and the gap is counted.

   Rejections are logged and counted.
3. **Feedback arriving while $J \ge J_{\mathrm{th}}$** is acknowledged (its version is recorded) but contributes $\delta_2 = 0$, as the paper does.
4. **The observation field $\mathcal{Q}$ is generated in one place.** A shared function `make_observation_field(config)` builds it, and both `mesh_feedback` and `coverage_controller` call it with the same config file. `field_id` is a hash of that config.
   - Index order is position-major: $j = \text{pos} \cdot (n_{\text{yaw}} n_{\text{pitch}}) + \text{yaw} \cdot n_{\text{pitch}} + \text{pitch}$.
   - Expanding $h_2$ from positions to all points is therefore `np.repeat(h2_per_position, n_yaw * n_pitch)`.
5. **`motion_executor` is the only path to the bridge.** It clamps commands to $\mathcal{U}$ (D14) and enforces the command timeout. On timeout it holds position with zero velocity; it never extrapolates the last command.
6. **The controller skips a tick** (`SOLVER_SKIPPED_STALE_STATE`, zero command) if the newest `DroneState5D` is older than a configured age.

---

## 8. Changes Relative to `ARCHITECTURE.md`

| Change | Reason |
|---|---|
| `/drone_1/cmd_vel` uses `cr_interfaces/VelocityCommand5D` instead of `geometry_msgs/Twist` | `Twist` has no clean slot for gimbal yaw and pitch rates. Overloading `angular.y` and `angular.z` would silently break any standard tool that reads `Twist`. |
| New topic `/drone_1/cmd_desired` | Separates what the controller wants from what the motion layer allows, so both appear in logs. |
| New package `cr_common` (ament_python) | Shared, ROS-free code: quaternion helpers, `Dir`, `make_observation_field`, and config loading. Avoids two packages importing each other. |

---

## 9. Reproducibility Note

Matched seeds and identical configs make trials *comparable*, but not *bit-identical*. ROS 2 callback timing varies from run to run, so the exact simulation step at which each image or command lands can differ. This is why the architecture requires repeated trials.

If exact replay becomes necessary later, the bridge can be switched to a lockstep mode, where it advances the simulation only after the controller has answered. This is not required in Phase 1.
