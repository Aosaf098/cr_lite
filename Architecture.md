# Coverage-Recon Lite
## Phase 1 — System Architecture

**Version:** 2.0  
**Status:** Planned  
**Project Type:** Independent robotics simulation project  
**Reference:** Coverage-Recon research paper (arXiv:2510.18347)  
**Scope:** Phase 1 — Open3D TSDF-based active 3D reconstruction

---

## 1. Project Overview

Coverage-Recon Lite is a ROS 2-based robotics simulation project inspired by the Coverage-Recon research paper.

The objective is to develop an autonomous drone simulation that incrementally reconstructs a 3D environment and uses online reconstruction feedback to improve its observation strategy.

The system combines:

- **MuJoCo:** Physics simulation, drone motion, and virtual RGB-D sensing.
- **ROS 2 Humble:** Communication, synchronization, and system orchestration.
- **Open3D TSDF Fusion:** Incremental 3D reconstruction using depth observations.
- **3D Grid-Based Mesh Feedback:** Detection of changes between successive reconstructed meshes.
- **5D Coverage Planning:** Position- and viewing-angle-aware observation planning.
- **Quadratic Programming (QP):** Optimization of drone motion under coverage and safety constraints.
- **RViz2 and Open3D:** Visualization and evaluation.

Unlike the original paper's neural reconstruction pipeline, Phase 1 uses geometric depth fusion to reduce computational requirements and implementation complexity.

The project is divided into two phases:

| Phase | Description | Status |
|---|---|---|
| Phase 1 | Implement and validate a closed-loop active 3D reconstruction system using Open3D TSDF | Current |
| Phase 2 | Benchmark Open3D TSDF against NeuralRecon | Future |

Phase 2 is explicitly excluded from the current implementation scope.

---

## 2. High-Level System Architecture

```text
+------------------------------------------------------+
|                 MuJoCo Simulation                    |
|                                                      |
|  +----------------+     +-------------------------+  |
|  | Indoor World   |     | Simulated Drone         |  |
|  |                |     |                         |  |
|  | Walls, objects |     | Pose, motion, camera    |  |
|  | Ground truth   |     | RGB-D sensor + gimbal   |  |
|  +----------------+     +-------------------------+  |
+---------------------------+--------------------------+
                            |
                            | RGB, Depth, Pose, Time
                            v
+------------------------------------------------------+
|                  ROS 2 MuJoCo Bridge                 |
|                                                      |
|  /clock                                              |
|  /drone_1/camera/rgb                                 |
|  /drone_1/camera/depth                               |
|  /drone_1/camera/camera_info                         |
|  /drone_1/odom                                       |
|  /tf                                                 |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                 RGB-D Preprocessing                  |
|                                                      |
|  - Depth conversion to meters                        |
|  - Invalid-depth filtering                           |
|  - Image/pose synchronization                        |
|  - Camera intrinsics and extrinsics validation       |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                  Open3D TSDF Mapper                  |
|                                                      |
|  - Persistent volumetric reconstruction              |
|  - Incremental depth integration                     |
|  - Periodic triangle-mesh extraction                 |
|  - Versioned mesh snapshots                          |
+---------------------------+--------------------------+
                            |
                            | Mesh(t), Mesh(t-1)
                            v
+------------------------------------------------------+
|                 Mesh-Change Feedback                 |
|                                                      |
|  - 3D Grid-Based mesh comparison                     |
|  - Spatial change estimation                         |
|  - Reconstruction feedback h2                        |
|  - Coverage importance updates                       |
+---------------------------+--------------------------+
                            |
                            | Importance field
                            v
+------------------------------------------------------+
|                   5D Coverage Planner                |
|                                                      |
|  q = (x, y, z, theta_h, theta_v)                     |
|                                                      |
|  - Angle-aware sensing model h1                      |
|  - Coverage objective                                |
|  - Importance-weighted observation planning          |
|  - Coverage gradients                                |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                  QP / CBF Controller                 |
|                                                      |
|  - Coverage-performance constraints                  |
|  - Workspace and obstacle constraints                |
|  - Viewing-angle and velocity constraints            |
|  - OSQP optimization                                 |
|  - Safe fallback behavior                            |
+---------------------------+--------------------------+
                            |
                            | Velocity / camera commands
                            v
+------------------------------------------------------+
|                 Motion Execution Layer               |
|                                                      |
|  - Command validation                                |
|  - Kinematic motion / dynamic tracking               |
|  - Command timeouts                                  |
|  - Collision monitoring                              |
+---------------------------+--------------------------+
                            |
                            +-------> MuJoCo Drone
                                      (Feedback Loop)


        +--------------------------------------+
        |       Evaluation & Visualization     |
        |                                      |
        | RViz2 / Open3D / rosbag2 / Matplotlib|
        | Ground-truth geometry comparisons    |
        | Runtime and memory profiling         |
        +--------------------------------------+
```

**Architectural principle:** Mapping, mesh feedback, coverage planning, and motion control operate as separate components. Reconstruction updates do not block the motion-control loop.

---

## 3. Technology Stack

| Layer | Technology | Responsibility |
|---|---|---|
| Operating system | Ubuntu 22.04 LTS | Development environment |
| Middleware | ROS 2 Humble | Inter-process communication |
| Programming language | Python 3 | Primary implementation |
| Simulator | MuJoCo | Physics, scene, and drone simulation |
| Environment format | MJCF/XML | Scene and robot descriptions |
| Camera | MuJoCo RGB-D rendering | Color and depth observations |
| Image processing | OpenCV, NumPy | Depth conversion and preprocessing |
| 3D reconstruction | Open3D | TSDF integration and mesh extraction |
| Mesh feedback | Open3D, NumPy, SciPy | Mesh-change estimation |
| Coverage planning | NumPy, SciPy | 5D coverage calculations |
| Optimization | OSQP | Quadratic programming |
| Coordinate transforms | tf2_ros | Robot and camera frames |
| Visualization | RViz2, Open3D | Mapping and robot visualization |
| Experiment analysis | Pandas, Matplotlib | Quantitative evaluation |
| Data recording | rosbag2, CSV, PLY | Reproducible experiments |
| Testing | pytest, launch_testing | Unit and integration testing |

### Hardware Target

- NVIDIA RTX 3060 Desktop — 12 GB VRAM
- CPU-based Open3D TSDF integration initially
- GPU primarily used for MuJoCo rendering
- No pretrained reconstruction model required

---

## 4. Component Specifications

### 4.1 MuJoCo Simulation

**Responsibility:** Simulate the environment, drone, and camera.

Initial environment:

- Indoor room approximately 4 × 4 × 3 meters.
- Walls, floor, and several geometric objects.
- Known ground-truth geometry.
- One simulated drone.
- RGB-D camera with configurable orientation.
- Workspace boundaries and collision geometry.

Implementation stages:

1. Kinematic drone movement.
2. Validated RGB-D sensor simulation.
3. Collision and workspace constraints.
4. Optional physical quadrotor dynamics and low-level tracking.

The initial kinematic model is a deliberate simplification and must not be presented as a validated physical quadrotor controller.

### 4.2 ROS 2–MuJoCo Bridge

**Responsibility:** Connect simulation state and sensors to ROS 2.

Required outputs:

```text
/clock
/tf
/drone_1/odom
/drone_1/camera/rgb
/drone_1/camera/depth
/drone_1/camera/camera_info
```

Required control input:

```text
/drone_1/cmd_vel
```

The bridge must:

- Publish consistent simulation timestamps.
- Convert MuJoCo depth values into metric depth.
- Publish valid camera intrinsics.
- Transform MuJoCo camera coordinates into ROS optical-frame conventions.
- Publish synchronized robot and camera poses.
- Handle command timeouts and simulation resets.

### 4.3 RGB-D Preprocessing

**Responsibility:** Prepare observations for TSDF integration.

Operations:

1. Receive RGB and depth images.
2. Associate images with acquisition-time camera poses.
3. Validate camera intrinsics and extrinsics.
4. Convert depth into Open3D-compatible units.
5. Filter invalid and out-of-range depth values.
6. Optionally introduce realistic depth noise.
7. Forward validated observations to the reconstruction interface.

Ground-truth poses are permitted for initial mapping validation.

Pose estimation is not a mandatory Phase 1 component.

### 4.4 Reconstruction Interface

**Responsibility:** Decouple the mapping backend from the rest of the system.

Conceptual interface:

```python
class ReconstructionBackend:

    def integrate(self, observation):
        """Integrate a calibrated sensor observation."""
        raise NotImplementedError

    def extract_mesh(self):
        """Return a versioned mesh snapshot."""
        raise NotImplementedError

    def reset(self):
        """Clear reconstruction state."""
        raise NotImplementedError

    def get_status(self):
        """Return runtime and mapping status."""
        raise NotImplementedError
```

Phase 1 implementation:

```text
ReconstructionBackend
        |
        v
Open3DTSDFBackend
```

The interface is retained for future extensibility, but no NeuralRecon code is implemented in Phase 1.

### 4.5 Open3D TSDF Mapping

**Responsibility:** Build a persistent 3D surface representation.

Pipeline:

```text
RGB-D Observation
        |
        v
Camera Pose + Intrinsics
        |
        v
TSDF Volume Integration
        |
        v
Persistent TSDF Volume
        |
        v
Periodic Mesh Extraction
        |
        v
Versioned Triangle Mesh
```

Initial configuration:

| Parameter | Starting value |
|---|---|
| Camera resolution | 320 × 240 |
| Depth range | 0.2–5.0 m |
| TSDF voxel size | 0.03 m |
| TSDF truncation distance | 0.09 m |
| Integration frequency | 2–5 Hz |
| Mesh extraction interval | 5 seconds |

Use a scalable or voxel-hashed TSDF implementation.

Mesh extraction should occur asynchronously where practical, without corrupting the active TSDF volume.

Each mesh snapshot should include:

- Unique version identifier.
- Simulation timestamp.
- Fixed world-frame identifier.
- Mesh file path or mesh reference.
- Triangle and vertex counts.
- Extraction duration.

### 4.6 Mesh-Change Feedback

**Responsibility:** Convert changes in reconstructed geometry into spatial feedback.

Inputs:

```text
Mesh(t-1)
Mesh(t)
```

Processing:

1. Align both meshes in the same fixed world coordinate frame.
2. Partition the environment into a spatial grid.
3. Calculate geometric changes using the paper's 3D Grid-Based method.
4. Compute the spatial feedback signal h₂.
5. Update the coverage importance field.
6. Publish feedback for the coverage controller.

Importance update:

\[
\phi_j(t^+) = \phi_j(t^-) + \delta_2 h_2(q_j,t)
\]

The exact spatial grid statistic and importance dynamics should be implemented according to the paper's definitions.

Validation requirements:

- Identical meshes produce approximately zero change.
- Local mesh changes produce spatially localized feedback.
- Out-of-order snapshots are rejected.
- Invalid or empty meshes are handled safely.
- Mesh changes are not automatically interpreted as calibrated reconstruction uncertainty.

### 4.7 5D Coverage Planner

**Responsibility:** Compute observation priorities using spatial position and viewing angles.

Coverage sample:

\[
q=(x,y,z,\theta_h,\theta_v)
\]

Where:

- \(x,y,z\): Spatial coordinates.
- \(\theta_h\): Horizontal viewing angle.
- \(\theta_v\): Vertical viewing angle.

The planner implements:

- Angle-aware sensing-performance function \(h_1\).
- Coverage importance dynamics.
- Coverage objective.
- Importance-weighted sampling.
- Required objective derivatives.
- Workspace and field-of-view constraints.

Initial configuration:

```text
Coverage samples: 2,000–5,000
Controller update: 5 Hz
Drones: 1
```

These are reduced-scale implementation settings rather than the original paper's full discretization.

Coverage gradients must be validated using finite-difference tests.

### 4.8 QP / CBF Controller

**Responsibility:** Calculate constrained drone motion commands.

Optimization backend:

```text
OSQP
```

Controller inputs:

- Drone pose and state.
- Coverage objective and gradients.
- Importance field.
- Workspace constraints.
- Obstacle information.
- Camera viewing-angle constraints.

Controller outputs:

- Desired translational velocity.
- Desired rotational or camera-angle commands.
- Solver status.
- Constraint diagnostics.

The controller should implement the paper's relevant sampling-performance CBF formulation, subject to documented adaptations for the selected kinematic model.

Safety requirements:

- Workspace boundary enforcement.
- Obstacle avoidance.
- Velocity limits.
- Viewing-angle limits.
- Safe behavior on solver failure.
- Stale-data handling.

Performance constraints may use slack variables, but hard safety constraints must not be silently relaxed.

### 4.9 Motion Execution

**Responsibility:** Execute controller commands in MuJoCo.

Initial implementation:

- Kinematic pose updates.
- Velocity and angular-rate limits.
- Collision checks.
- Command timeout.
- Emergency stop.
- Simulation reset.

Optional extension:

- Physical quadrotor dynamics.
- Motor or thrust-level control.
- Low-level velocity tracking.

### 4.10 Evaluation and Visualization

**Responsibility:** Measure system behavior and reconstruction performance.

Tools:

- RViz2
- Open3D
- rosbag2
- Pandas
- Matplotlib

Metrics:

**Reconstruction**
- Mesh precision.
- Mesh recall.
- Mesh F-score.
- Surface completeness.
- Geometric error against ground truth.

**Coverage**
- Coverage objective over time.
- Observation count.
- Trajectory length.
- Importance-field evolution.

**Computation**
- TSDF integration latency.
- Mesh extraction latency.
- Mesh-feedback latency.
- Coverage-planning latency.
- QP solve time.
- CPU and RAM utilization.
- GPU utilization where relevant.

**Reliability**
- Mission completion.
- Collision count.
- QP solver failures.
- Stale mapping updates.
- Dropped sensor frames.

---

## 5. ROS 2 Package Structure

```text
coverage_recon_lite/
│
├── ros2_ws/
│   └── src/
│       │
│       ├── cr_interfaces/
│       │   └── Custom messages and services
│       │
│       ├── cr_description/
│       │   └── Drone descriptions and frames
│       │
│       ├── cr_mujoco_bridge/
│       │   └── Simulation and sensor bridge
│       │
│       ├── cr_rgbd_processing/
│       │   └── Sensor preprocessing
│       │
│       ├── cr_tsdf_mapping/
│       │   └── Reconstruction interface
│       │   └── Open3D TSDF backend
│       │
│       ├── cr_mesh_feedback/
│       │   └── 3D Grid-Based mesh comparison
│       │
│       ├── cr_coverage_control/
│       │   └── 5D coverage objective
│       │   └── QP / CBF controller
│       │
│       ├── cr_motion_control/
│       │   └── Motion execution and safety
│       │
│       ├── cr_evaluation/
│       │   └── Metrics and experiment logging
│       │
│       └── cr_bringup/
│           └── ROS 2 launch files
│
├── environments/
│   ├── mjcf/
│   ├── meshes/
│   └── textures/
│
├── configs/
│   ├── simulation.yaml
│   ├── camera.yaml
│   ├── mapping.yaml
│   ├── feedback.yaml
│   └── controller.yaml
│
├── experiments/
│   ├── configs/
│   ├── feedback_on/
│   ├── feedback_off/
│   ├── evaluation/
│   └── plots/
│
├── tests/
│
├── docs/
│
├── README.md
└── ARCHITECTURE.md
```

---

## 6. Runtime Scheduling

Different components operate at independent frequencies.

| Component | Initial frequency |
|---|---|
| MuJoCo physics | 200 Hz |
| Motion execution | 20–50 Hz |
| RGB-D camera | 5 Hz |
| TSDF integration | 2–5 Hz |
| Mesh extraction | Every 5 seconds |
| Mesh-change calculation | On new mesh snapshot |
| Importance update | On new valid feedback |
| Coverage controller | 5 Hz |
| QP optimization | 5 Hz |

The TSDF integration loop and mesh extraction should not block the controller.

If no new mesh is available, the controller may continue using the latest valid importance field, subject to a configurable staleness policy.

---

## 7. Experimental Design

### Experiment A — Feedback Disabled

Run autonomous coverage using the baseline coverage model without online mesh-change importance updates.

```text
Coverage Planner
       |
       v
QP Controller
       |
       v
Drone Motion
       |
       v
TSDF Reconstruction
```

Reconstruction is still performed and evaluated, but mesh-change feedback does not modify coverage importance.

### Experiment B — Feedback Enabled

Run the same system with mesh-change-based importance updates.

```text
TSDF Reconstruction
       |
       v
Mesh-Change Feedback
       |
       v
Importance Update
       |
       v
Coverage Planner
       |
       v
QP Controller
       |
       v
Drone Motion
       |
       +----> TSDF Reconstruction
```

### Comparison Requirements

Both configurations must use:

- Identical simulation environments.
- Identical camera parameters.
- Identical TSDF settings.
- Identical initial conditions.
- Identical observation budgets.
- Matched random seeds.
- Consistent evaluation thresholds.

Run at least five repeated trials per configuration initially.

The primary evaluation question is:

**Does online mesh-change feedback improve reconstruction quality or observation efficiency compared with the same coverage controller operating without feedback?**

A negative or mixed result is valid if the implementation and evaluation are reproducible.

---

## 8. Development Milestones

| Milestone | Deliverable |
|---|---|
| M1 | MuJoCo drone simulation with ROS 2 RGB-D sensor bridge |
| M2 | Working incremental Open3D TSDF reconstruction |
| M3 | Validated 3D Grid-Based mesh-change feedback |
| M4 | Working 5D coverage planner and QP controller |
| M5 | Fully integrated autonomous closed-loop system |
| M6 | Controlled experiments, visualization, tests, and documentation |

### Phase 1 Completion Criteria

Phase 1 is complete when:

1. The drone can autonomously navigate the simulated environment.
2. RGB-D observations are incrementally integrated into a TSDF map.
3. The system generates versioned 3D mesh snapshots.
4. Mesh changes update the spatial importance field.
5. The coverage controller responds to importance updates.
6. The controller respects implemented workspace and safety constraints.
7. Complete missions run without unhandled failures.
8. Feedback-enabled and feedback-disabled experiments can be reproduced.
9. Reconstruction quality and computational performance are quantitatively evaluated.
10. The repository contains working launch instructions, tests, experiment configurations, and a demonstration.

---

## 9. Scope Boundaries

### Included in Phase 1

- Single-drone simulation.
- MuJoCo environment.
- ROS 2 Humble integration.
- RGB-D sensing.
- Open3D TSDF reconstruction.
- 3D Grid-Based mesh feedback.
- 5D coverage planning.
- QP-based motion control.
- Controlled experimental evaluation.

### Excluded from Phase 1

- NeuralRecon integration.
- NeuralRecon versus Open3D benchmarking.
- Real-world drone hardware.
- LiDAR or autonomous surface vehicles.
- Mandatory multi-drone coordination.
- Neural network training.
- Mandatory SLAM or camera-pose estimation.

---

## 10. Architectural Design Principles

1. **Modularity:** Reconstruction, feedback, planning, and control remain separate modules.
2. **Reproducibility:** Configurations, seeds, dependencies, and experimental results are recorded.
3. **Asynchronous execution:** Reconstruction latency must not block motion control.
4. **Scientific traceability:** Paper-derived algorithms and deliberate adaptations are documented separately.
5. **Safety:** Motion commands must respect constraints and fail safely.
6. **Observability:** Important runtime states, errors, and performance metrics are logged.
7. **Extensibility:** The reconstruction backend can be replaced without rewriting the coverage controller.
8. **Ground-truth isolation:** Ground-truth geometry is used for evaluation, not as an input to the coverage controller.
9. **Computational efficiency:** The initial system targets a single consumer-grade computer.
10. **Incremental validation:** Each subsystem is tested independently before full integration.

---

## 11. Future Work — Phase 2

After completing Phase 1, the project may be extended to benchmark:

- Open3D TSDF Fusion.
- NeuralRecon.

Potential comparison dimensions:

- Reconstruction accuracy.
- Mesh completeness.
- Computational latency.
- GPU memory consumption.
- CPU and RAM consumption.
- Reconstruction stability.
- Feedback behavior.
- Coverage efficiency.

Because Open3D TSDF and NeuralRecon use different sensing assumptions, the benchmark must distinguish differences caused by the reconstruction algorithm from differences caused by input modalities.

**No Phase 2 implementation is required for Phase 1 completion.**

---

## 12. Final System Definition

**Coverage-Recon Lite Phase 1** is a modular, ROS 2-based autonomous drone simulation that performs incremental RGB-D reconstruction using Open3D TSDF Fusion and uses online mesh-change feedback to influence 5D coverage planning and constrained drone motion.

The final deliverable is a working, reproducible, experimentally evaluated closed-loop reconstruction system.

The project is an independent implementation inspired by the Coverage-Recon paper, not an exact reproduction of its complete system.