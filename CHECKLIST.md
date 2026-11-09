# Coverage-Recon Lite — Phase 1 Checklist

Items marked **(new)** come from decisions made in `docs/deviations.md`, `docs/conventions.md`, `docs/interfaces.md`, and `docs/evaluation_plan.md`. The document references on each item show where the decision lives.

## 1. Project setup and specifications

- [x] Create the GitHub repository and project directory structure.
- [x] Document the original paper's coverage objective, sensing model, importance update, mesh-change method, and QP/CBF constraints. → `docs/paper_spec.md`
- [x] Identify which algorithms will be reproduced faithfully and which will be simplified. → `docs/deviations.md`
- [x] Define coordinate frames, units, camera conventions, and ROS 2 message interfaces. → `docs/conventions.md`, `docs/interfaces.md`
- [x] Define measurable success criteria and initial experiment configurations. → `docs/evaluation_plan.md`, `experiments/configs/phase1_main.yaml`
- [x] Create a modular reconstruction interface for future backends. → `cr_tsdf_mapping/reconstruction_backend.py`
- [x] **(new)** Amend Principle 8 in `ARCHITECTURE.md` (safety-only keep-out map, D13).

**Completion:** A documented architecture, implementation specification, and initialized repository. ✅ Done

## 2. Development environment

- [x] Verify Ubuntu 22.04, Python, and ROS 2 Humble. → Ubuntu 22.04.5, Python 3.10.12, ROS 2 Humble, RTX 3060 (driver 595.99)
- [x] Create the ROS 2 colcon workspace and Python environment. → `.venv` with `--system-site-packages`, `env.sh`; always build with `python -m colcon build --symlink-install`
- [x] Install MuJoCo, Open3D, NumPy, SciPy, OpenCV, OSQP, and plotting tools. → pip into `.venv`; OpenCV from apt (`python3-opencv`)
- [x] Install ROS 2 image, TF, visualization, and testing dependencies. → `environment/apt-packages.txt`
- [x] **(new)** Generate the ROS 2 packages with `ros2 pkg create`. `cr_interfaces` uses ament_cmake; all others use ament_python, including the new `cr_common`. → 11 packages; packaging added around the existing `cr_tsdf_mapping` backend interface
- [x] Verify MuJoCo offscreen rendering, Open3D mesh extraction, and OSQP solving. → `tests/smoke/run_all.sh`
- [x] Record and pin dependency versions. → `requirements.txt`, `requirements.lock.txt`, `environment/system_versions.txt`; lock file verified by rebuilding a clean venv
- [x] **(new)** TSDF backend uses Open3D's tensor `VoxelBlockGrid` instead of the legacy `ScalableTSDFVolume`. → `docs/deviations.md` D15

**Completion:** All core libraries pass isolated smoke tests. ✅ Done (all 12 checks pass)

**Carry-forward notes:**
- Keep NumPy below 2.0: ROS 2 Humble's `cv_bridge` is built against NumPy 1.x.
- Don't upgrade `setuptools` in `.venv`; newer versions break colcon `ament_python` builds on Humble.
- Rendering uses `MUJOCO_GL=egl`, which gives headless GPU rendering for the bridge.
- Section 5: `VoxelBlockGrid` truncation is given in voxels (`trunc_voxel_multiplier = 3.0`), and mesh extraction needs an explicit `weight_threshold` (default 3.0). Make it a `mapping.yaml` parameter.
- Section 8: OSQP 1.x reports infeasibility as a status but raises an exception for invalid data. The controller fallback must handle both.

## 3. MuJoCo environment and drone

- [ ] Create a small indoor MJCF environment with walls, floor, and objects. Origin at the floor centre, room spanning $x, y \in [-2, 2]$ m and $z \in [0, 3]$ m.
- [ ] **(new)** Define the target region $\mathcal{B}$ around the furniture (D11) and record it in `phase1_main.yaml`.
- [ ] **(new)** Write the keep-out box map (furniture plus margin) to `configs/controller.yaml` (D13).
- [ ] Add a simulated drone body with configurable initial pose (position-only base, gimbal yaw and pitch).
- [ ] Implement basic kinematic translation and rotation ($\dot p = u$, D4).
- [ ] Add a camera with configurable gimbal (yaw then pitch, pitch positive down). Use `fovy` = 55.2° at 320×240.
- [ ] Implement RGB and depth rendering from the camera.
- [ ] Convert MuJoCo depth-buffer output into metric z-depth.
- [ ] Generate ground-truth scene geometry for evaluation (1 cm surface sampling, D19).
- [ ] Implement collision detection, workspace boundaries, and simulation reset.
- [ ] **(new)** Generate `experiments/configs/initial_poses.yaml` for the evaluation and pilot seeds.

**Completion:** A drone can move through the environment and capture geometrically correct RGB-D images.

## 4. ROS 2–MuJoCo integration

- [ ] **(new)** Define the 7 custom messages in `cr_interfaces` (`interfaces.md` §4).
- [ ] **(new)** Add quaternion helpers and `Dir` to `cr_common`, with tests T1 and T7.
- [ ] Implement the MuJoCo bridge ROS 2 node.
- [ ] Publish `/clock` and configure simulation time.
- [ ] Publish RGB images, depth images (`32FC1`, NaN for invalid), and CameraInfo.
- [ ] Publish drone odometry, `DroneState5D`, and TF transforms (including the static optical transform).
- [ ] Implement the velocity-command interface (`VelocityCommand5D` on `/drone_1/cmd_vel`).
- [ ] Verify optical-frame conventions and camera-to-world transforms (tests T2, T3, T4).
- [ ] Synchronize camera frames with acquisition-time poses, using identical stamps (test T9).
- [ ] Visualize camera data, TF, and robot state in RViz2.
- [ ] Implement command timeouts and safe stopping in `motion_executor`.
- [ ] **(new)** Implement the reset, estop, start, and stop services (`interfaces.md` §5).

**Completion:** ROS 2 can observe and control the simulated drone with validated sensor geometry.

## 5. Open3D TSDF reconstruction

- [ ] Implement a standalone TSDF mapper using recorded RGB-D frames.
- [ ] Configure voxel size, truncation distance, depth range, and integration weights (`VoxelBlockGrid`, D15: truncation in voxels, explicit `weight_threshold`).
- [ ] Validate camera intrinsics and world-to-camera extrinsics (test T6; extrinsic $= T_{\mathrm{opt\_world}}$).
- [ ] **(new)** Replace NaN with 0 before integration (`conventions.md` §5).
- [ ] Integrate sequential depth frames into a persistent TSDF volume.
- [ ] Extract triangle meshes at configurable intervals.
- [ ] Save atomic, versioned PLY mesh snapshots with timestamps, and publish `MeshSnapshot`.
- [ ] Integrate the mapper as a ROS 2 component.
- [ ] Measure integration time, mesh extraction time, CPU usage, and RAM.
- [ ] Verify the reconstructed mesh against known scene geometry (criterion E2; test T5).

**Completion:** A predefined drone trajectory produces a coherent, progressively reconstructed 3D mesh.

## 6. Mesh-change feedback

- [ ] Implement a fixed spatial grid for comparing successive meshes (0.3 m spacing, clipped to $\mathcal{B}$).
- [ ] Implement the paper's 3D Grid-Based mesh-change calculation (Eqs. 5–6).
- [ ] Map mesh changes to the spatial feedback signal $h_2$, sent per unique position.
- [ ] **(new)** Implement the $\kappa$ calibration procedure (D9).
- [ ] Implement importance-field updates using the feedback signal. These live in the controller (`interfaces.md` §7).
- [ ] Verify that identical meshes produce approximately zero change (criterion E3a).
- [ ] Test mesh additions, removals, and local geometry changes (criterion E3b).
- [ ] Verify spatial consistency, mesh version ordering, and `field_id` checks (criterion E3c).
- [ ] Visualize the resulting feedback and importance fields.

**Completion:** Mesh changes generate correct, testable updates to the coverage importance field.

## 7. 5D coverage planning

- [ ] Represent coverage samples as (x, y, z, horizontal angle, vertical angle).
- [ ] Generate the reduced sample set with `make_observation_field` in `cr_common` (D10, position-major order).
- [ ] Implement the paper's angle-aware sensing-performance function $h_1$, with the D5 convention.
- [ ] **(new)** D5 acceptance test: $h_1 = 1$ at the ideal pose (test T8).
- [ ] Implement the coverage objective and importance evolution (exact exponential decay between ticks).
- [ ] Compute the coverage objective's required spatial and angular derivatives.
- [ ] Validate analytical derivatives using finite differences (criterion E4).
- [ ] Implement workspace and camera field-of-view restrictions.
- [ ] Profile coverage computations and optimize sampling if necessary.
- [ ] Visualize coverage scores and preferred observation directions.

**Completion:** The planner produces meaningful position and viewing-direction preferences.

## 8. QP-based drone controller

- [ ] Formulate the coverage-control quadratic program (Eq. 19).
- [ ] Implement the sampling-performance CBF constraint (soft, with slack $w$).
- [ ] Implement workspace (D12) and keep-out obstacle (D13) safety constraints.
- [ ] Add camera pitch (two linear barriers, D6) and input-bound constraints (D14).
- [ ] Implement OSQP solving and `CoverageStatus` reporting.
- [ ] Separate soft performance constraints from hard safety constraints.
- [ ] Handle infeasible QPs, stale inputs, and solver failures safely (OSQP 1.x: infeasibility is a status, invalid data raises an exception).
- [ ] **(new)** Finite-difference tests for all CBF gradients (criterion E4).
- [ ] Validate commands in simple analytical and simulated scenarios.
- [ ] Connect controller outputs to the MuJoCo motion interface (via `/drone_1/cmd_desired`).

**Completion:** The drone follows coverage-driven commands while respecting implemented safety constraints.

## 9. Complete closed-loop integration

- [ ] Connect RGB-D acquisition to TSDF integration.
- [ ] Connect mesh snapshots to the mesh-change detector.
- [ ] Connect mesh-change feedback to importance updates, gated by $J_{\mathrm{th}}$.
- [ ] Connect the updated importance field to the coverage planner.
- [ ] Connect the QP controller to drone motion.
- [ ] Run physics, camera, mapping, and control at independent rates.
- [ ] Implement asynchronous mesh processing and stale-map handling.
- [ ] Implement autonomous mission start, stop, and reset (`experiment_runner`, reset order from `interfaces.md` §5).
- [ ] Run a complete autonomous reconstruction mission.
- [ ] Log all important states, timings, commands, and reconstruction versions (criterion E9 run directory; include `scripts/record_env.sh` output).

**Completion:** The drone autonomously reconstructs the scene and adapts its observations based on mesh feedback.

## 10. Baselines and controlled experiments

- [ ] Implement the four conditions C0–C3 (feedback on/off × depth noise on/off).
- [ ] **(new)** Implement the depth noise model (`evaluation_plan.md` §4.5).
- [ ] Ensure all conditions use identical sensors, mapping settings, and observation budgets (fixed mission time).
- [ ] Create fixed initial conditions and reproducible random seeds.
- [ ] Generate ground-truth reference surfaces for geometric evaluation.
- [ ] **(new)** Run the pilot on seeds 100–102 (feedback-off only), set $\gamma$, $\kappa$, $T$, and $\sigma_2$, then commit and tag `experiments-frozen`.
- [ ] Compute mesh precision, recall, F-score (5 cm and 2.5 cm), Chamfer distance, and F-score AUC.
- [ ] Record trajectory length, observation count, and coverage metrics.
- [ ] Measure TSDF integration, mesh extraction, feedback, and QP latency separately.
- [ ] Run five paired trials per condition (seeds 0–4).
- [ ] Compare performance at all four checkpoints.
- [ ] **(new)** Apply the pre-registered decision rule; extend to 15 seeds if inconclusive.
- [ ] Investigate cases where mesh feedback helps or hurts (mechanism check, `evaluation_plan.md` §6).

**Completion:** Controlled experiments establish whether feedback changes reconstruction quality or efficiency.

## 11. Testing and reliability

- [ ] Write unit tests for depth conversion, projection, and frame transforms (T1–T9).
- [ ] Write TSDF integration and mesh snapshot tests.
- [ ] Write synthetic mesh-change tests.
- [ ] Write coverage gradient and QP constraint tests.
- [ ] Test empty meshes, missing frames, invalid depth, and stale feedback.
- [ ] Test reproducible simulation resets and repeated missions.
- [ ] Run an end-to-end automated integration test.
- [ ] Document known limitations and remaining failures.

**Completion:** Core components pass tests and the system reliably completes repeatable missions (criteria E1–E9).

## 12. Demo, documentation, and release

- [ ] Create a single-command or clearly documented launch procedure.
- [ ] Prepare an RViz2 visualization of drone pose, trajectories, and mapping.
- [ ] Produce a time-lapse or video showing incremental 3D reconstruction.
- [ ] Visualize feedback and importance-field changes.
- [ ] Generate the six deliverable figures (`evaluation_plan.md` §7).
- [ ] Document setup, architecture, algorithms, configuration, and usage.
- [ ] Document deviations from the original paper and experimental limitations.
- [ ] Clean the repository and publish reproducible example configurations.
- [ ] Tag the Phase 1 release and archive experimental results.

**Completion:** A reproducible project with a working demo, test suite, quantitative results, and documentation.
