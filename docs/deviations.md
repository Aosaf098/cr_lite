# Deviations from the Paper

**Reference:** `docs/paper_spec.md`, a transcription of arXiv:2510.18347. Section and issue numbers below refer to that document.

**Purpose:** Every place this project differs from the paper is recorded here, along with the reason and its expected effect on results. Anything *not* listed here is implemented as written in the paper.

**Categories:**

- **Faithful:** implemented as in the paper.
- **Corrected:** the paper appears to contain an error, and we implement what it evidently intends.
- **Adapted:** changed deliberately for scope or compute reasons.
- **Added:** not in the paper.
- **Omitted:** in the paper, but not implemented in Phase 1.

Status of each entry is either **Decided** or **Proposed**. Proposed entries need sign-off before the code that depends on them is written.

---

## 1. Summary

| ID | Topic | Category | Status |
|---|---|---|---|
| D1 | Reconstruction: Open3D TSDF instead of NeuralRecon | Adapted | Decided |
| D2 | Simulator: MuJoCo instead of Unity | Adapted | Decided |
| D3 | Single drone; collision CBF and Voronoi partition dropped | Adapted | Decided |
| D4 | Single-integrator kinematics $\dot p = u$ | Faithful | Decided |
| D5 | `Dir` convention and $\varphi_2$ sign (Issue 9.1) | Corrected | Decided |
| D6 | Gimbal pitch constraint (Issues 9.2, 9.3) | Corrected | Decided |
| D7 | $\gamma$ value (Issue 9.4) | Adapted | Proposed |
| D8 | $\sigma_2$ value (Issue 9.7) | Adapted | Proposed |
| D9 | $\kappa$ and grid $\mathcal{R}$ re-tuned for TSDF meshes (Issue 9.8) | Adapted | Proposed |
| D10 | Reduced virtual field: ~4,000 samples | Adapted | Proposed |
| D11 | Target region $\mathcal{B}$ and pitch range | Faithful (Option A) | Decided |
| D12 | Workspace boundary CBF | Added (paper mentions, no equation) | Proposed |
| D13 | Static-obstacle avoidance CBF (known keep-out map) | Added | Decided |
| D14 | Input bounds $\mathcal{U}$ | Added (paper mentions, no values) | Proposed |
| D15 | Rates: 5 Hz control, mesh every 5 s | Adapted | Proposed |
| D16 | Camera: 320×240 RGB-D, ground-truth poses | Adapted | Decided |
| D17 | No occlusion in $h_1$ (Issue 9.9) | Faithful | Decided |
| D18 | M3C2 feedback method | Omitted | Decided |
| D19 | Ground truth from MJCF primitives, not resampled meshes | Adapted | Decided |
| D20 | Feedback activation $J_{\mathrm{th}} = 0.5$ | Faithful | Decided |

---

## 2. Pipeline-Level Deviations

### D1 — Open3D TSDF instead of NeuralRecon (Adapted)

| | |
|---|---|
| **Paper** | NeuralRecon takes monocular RGB plus poses and produces a learned, incrementally fused mesh. |
| **Ours** | Open3D `ScalableTSDFVolume` (voxel-hashed) takes metric depth plus ground-truth poses. Marching-cubes mesh extraction every 5 s. |
| **Reason** | No GPU model or training needed; deterministic; runs on a single RTX 3060. |
| **Expected effect** | This is the most important scientific difference. In NeuralRecon, a region's mesh keeps changing as the network refines its prediction. In a TSDF with perfect depth and poses, a surface patch mostly changes when it is **first observed**, and repeat observations only average out small differences. The $h_2$ signal may therefore act mainly as a "newly seen surface" detector rather than a "still-refining" detector, which could weaken the benefit of feedback. |
| **Mitigation** | Experiments are run both with and without depth noise (Sec. 4.3 of the architecture), since noise makes the TSDF behave more like a refining estimator. A negative result here is a legitimate finding, as the architecture document already allows. |

### D2 — MuJoCo instead of Unity (Adapted)

The scene is simple geometric primitives, not photorealistic. This has no effect on TSDF reconstruction, which uses depth only, but it means RGB texture is irrelevant in Phase 1.

### D3 — Single drone (Adapted)

With $n = 1$:

- $\mathcal{V}_1 = \mathcal{M}$ (all observation points), so $|\mathcal{V}_1| = m$.
- The $\max$ in Eq. (2) reduces to $h_1(p_1, q_j)$.
- The collision CBF (Eqs. 16–17, 22) is not implemented.
- Issue 9.6 (Voronoi inequality direction) is moot.

The code keeps a drone-index dimension in its arrays so multi-drone support is not blocked later.

### D4 — Single-integrator kinematics (Faithful)

The paper's model is $\dot p_i = u_i$ with all five state components directly actuated. Our kinematic MuJoCo drone (a mocap body plus gimbal joints) matches this exactly. Physical quadrotor dynamics would be an **addition** to the paper, not a correction.

---

## 3. Corrections to the Paper's Equations

### D5 — Direction convention (Corrected, Issue 9.1)

The world frame is z-up. Pitch $\theta^v > 0$ tilts the camera *down*, so $\theta^v = \pi/2$ points straight down. A single direction function is used for both drone and observation point:

$$\mathrm{Dir}(\theta^h,\theta^v) = [\cos\theta^h\cos\theta^v,\ \sin\theta^h\cos\theta^v,\ -\sin\theta^v]^\top$$

- $\mathrm{Dir}(p_i)$: the camera optical axis.
- $\mathrm{Dir}(q_j)$: the direction a camera should be **looking** when it observes $q_j$.

With that meaning, the paper's minus sign in $\varphi_2$ must be dropped:

$$\cos\varphi_2 = \frac{\mathrm{Dir}(q_j)\cdot(\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i))}{\|\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i)\|}$$

This is equivalent to the paper with $\mathrm{Dir}(q_j)$ redefined as the outward direction toward the viewer. We chose the form where both directions mean "look direction", because it is easier to visualize and test.

**Acceptance test** (unit test, required): For any $q$, the pose $p$ with $\mathrm{Pos}(p) = \mathrm{Pos}(q) - D\,\mathrm{Dir}(q)$ and $\mathrm{Ang}(p) = \mathrm{Ang}(q)$ must give $\varphi_1 = \varphi_2 = 0$, $\rho = 0$, and $h_1 = 1$. Rotating the camera away from that pose, or moving the drone sideways, must strictly reduce $h_1$.

### D6 — Gimbal pitch constraint (Corrected, Issues 9.2 and 9.3)

The paper's quadratic barrier has a doubled range and a wrong-signed gradient. We replace it with **two linear barriers**, which are exact, simpler, and give a constant gradient:

$$b_{\min} = \theta^v - \theta^v_{\min} \ge 0, \qquad b_{\max} = \theta^v_{\max} - \theta^v \ge 0$$

The corresponding QP rows are:

$$u_{\theta^v} + a_2\,(\theta^v - \theta^v_{\min}) \ge 0$$

$$-u_{\theta^v} + a_2\,(\theta^v_{\max} - \theta^v) \ge 0$$

These enforce the same safe set the paper intends. Every CBF gradient in the code is verified with a finite-difference unit test.

---

## 4. Parameters That Cannot Be Copied

### D7 — $\gamma$ (Adapted, Proposed)

The paper never states $\gamma$. The duplicated "$\epsilon = 0.012$" is probably $\gamma$, but this is unconfirmed (Issue 9.4).

**Proposal:** Treat $\gamma$ as a tuned parameter. Sweep $\gamma \in \{0.001, 0.003, 0.01\}$ in a short tuning study, then fix one value for all experiments. Log the slack $w$ every control step to see how often the performance constraint is actually active.

### D8 — $\sigma_2$ (Adapted, Proposed)

**Proposal:** Compute $\sigma_2$ from the paper's own formula using our angular spacing $\Delta\theta$, since our spacing differs from the paper's anyway (see D10). Record the paper's 0.095 as an alternative for a one-off sensitivity check.

### D9 — $\kappa$ and grid $\mathcal{R}$ (Adapted, Proposed)

**Proposal:** Keep Eq. (6) **unchanged in structure** (vertex-count change, $\tanh^2$ normalization, Gaussian spreading) and recalibrate only its constants:

- Grid $\mathcal{R}$: 0.3 m spacing, matching the paper's positional cell size, clipped to $\mathcal{B}$.
- $\kappa$: calibrated empirically. Record $|\Delta V^t(r_k)|$ over a feedback-disabled run at the 5 s snapshot interval, then set $\kappa$ to roughly the median non-zero value. This makes typical changes map to $\tanh^2(1) \approx 0.58$, so the term neither saturates nor vanishes. The calibration run and resulting value are saved in `configs/feedback.yaml`.
- $\sigma_4 = 0.4$ m (faithful).

### D10 — Reduced virtual field (Adapted, Proposed)

| | Paper | Ours (proposed) |
|---|---|---|
| Positional spacing | 0.3 m | 0.5 m |
| Yaw bins | ~21 (0.3 rad) | 8 ($\Delta\theta^h = \pi/4$) |
| Pitch bins | ~2 | 2 |
| Total $m$ | $1.5\times10^7$ (reported) | ~3,000–5,000, depending on D11 |

Observation points are placed at cell centres. We use the yaw spacing as $\Delta\theta$ for $\sigma_2$ (D8). Since the pitch spacing is finer than this, the yaw value is the conservative choice.

**Effect:** The angular diversity the paper rewards is coarser, so absolute J and F-score values are not comparable with the paper. Only the feedback-on vs feedback-off comparison within this project is meaningful, which is how the architecture document already frames the evaluation.

---

## 5. Scene and Safety Decisions

### D11 — Target region $\mathcal{B}$ and pitch range (Decided: Option A)

**Decision (2026-11-08):** Option A. $\mathcal{B}$ covers the furniture area only, from the floor to about 1.5 m, and $\Theta^v = [\pi/3, \pi/2]$ as in the paper. Walls are excluded from the F-score.

The paper's pitch range $\Theta^v = [\pi/3, \pi/2]$ means the camera always looks 60°–90° downward. That suits furniture seen from above, but it **cannot observe walls** well.

- **Option A (faithful):** $\mathcal{B}$ covers the furniture area only, not the walls, from floor to ~1.5 m. Keep $\Theta^v = [\pi/3, \pi/2]$. Walls are reconstructed incidentally but excluded from F-score.
- **Option B (adapted):** $\mathcal{B}$ includes walls, and $\Theta^v$ is widened to about $[\pi/12, \pi/2]$ (pitch bins re-spaced accordingly). This is closer to "reconstruct the room", but deviates from the paper's sensing geometry.

**Recommendation: A.** It stays faithful, keeps $m$ small, and matches the paper's experimental scene.

Geometry note: with $D = 1$ m and steep pitch, the drone has to fly about 1 m above surfaces. In a 3 m room the flight region $\mathcal{P}$ is roughly $z \in [0.8, 2.5]$ m, which fits.

### D12 — Workspace boundary CBF (Added, Proposed)

The paper mentions a flight-region CBF without giving it. We use six linear barriers, one per face of $\mathcal{P}$, for example $b = x - x_{\min} \ge 0$. They are hard constraints, giving QP rows of the form $u_x + a_w(x - x_{\min}) \ge 0$.

The bounds of $\mathcal{P}$ are inset from the walls by the drone radius plus a margin. Proposed values: radius 0.15 m, margin 0.2 m.

### D13 — Static-obstacle avoidance (Added, Decided: Option A)

**Decision (2026-11-08):** Option A, a known safety map. Keep-out boxes live in `configs/controller.yaml` and are used only by the safety CBFs. `ARCHITECTURE.md` Principle 8 must be amended to match.

The paper has no obstacle avoidance against scene geometry. A drone flying about 1 m above furniture still needs it.

**The conflict:** Design Principle 8 says ground truth must not be an input to the controller. The simplest obstacle CBF reads furniture boxes from the MJCF file, which *is* ground truth.

- **Option A:** A **known safety map**. A config file of keep-out boxes (furniture inflated by a margin) is passed **only** to the safety CBFs, never to the coverage objective or importance field. Principle 8 is amended to read: "ground truth is not used for coverage or feedback; a static keep-out map may be used for safety."
- **Option B:** Obstacle CBFs built from the live TSDF map. This is more realistic, but couples safety to reconstruction quality and adds significant work. It is better suited to Phase 2.

**Recommendation: A.** Since the coverage behaviour is unaffected, the feedback-on vs feedback-off comparison stays fair.

### D14 — Input bounds $\mathcal{U}$ (Added, Proposed)

Box bounds are added as QP variable bounds. Proposed starting values: $|u_{x,y,z}| \le 0.5$ m/s, $|u_{\theta^h}| \le 0.5$ rad/s, $|u_{\theta^v}| \le 0.3$ rad/s. These keep motion blur and TSDF pose spacing reasonable at a 5 Hz camera rate.

---

## 6. Runtime and Sensing

### D15 — Rates (Adapted, Proposed)

| | Paper | Ours |
|---|---|---|
| Control | 20 Hz | 5 Hz |
| Images | 20 fps | 5 Hz |
| Mesh update | NeuralRecon fragment updates | Every 5 s |

Importance decay is integrated exactly between control ticks with $\phi_j \leftarrow \phi_j \exp(-\delta_1 h_1 \Delta t)$, so the lower rate does not change the decay model. However, the snapshot interval directly scales $\Delta V$, which is why $\kappa$ is calibrated at the chosen interval (D9).

### D16 — Camera and poses (Adapted, Decided)

- Camera: 320×240 RGB-D, depth range 0.2–5.0 m. The field of view is set to approximate the DJI Mini 3's (about 82° diagonal) so the paper's $\sigma_1 = 0.07$ stays meaningful.
- Poses: ground truth from the simulator. This is equivalent to the paper's MoCap poses in the real experiment.
- $D = 1.0$ m and $\sigma_3 = 0.3$ m are kept (faithful).

### D17 — No occlusion in $h_1$ (Faithful, Decided)

Points hidden behind furniture still decay. This is kept for fidelity and listed as a known limitation. A depth-based visibility mask is a possible ablation, but is not part of Phase 1.

### D18 — M3C2 (Omitted, Decided)

Only the 3D Grid method is implemented. The paper itself adopts the grid method for all its later experiments.

### D19 — Ground truth (Adapted, Decided)

The ground-truth surface is sampled uniformly from the MJCF primitive shapes at a density comparable to the TSDF output. This replaces the paper's MeshLab resampling step and serves the same purpose. The F-score threshold is $\eta = 0.05$ m (faithful).

### D20 — Feedback activation (Faithful, Decided)

$J_{\mathrm{th}} = 0.5$ and $\delta_2' = 1.0$, as in the paper. The feedback-disabled baseline uses $\delta_2 = 0$ throughout, which matches the paper's baseline exactly.

---

## 7. Parameters Kept from the Paper

$D = 1.0$, $\sigma_1 = 0.07$, $\sigma_3 = 0.3$, $\sigma_4 = 0.4$, $\delta_1 = 3.0$, $\delta_2' = 1.0$, $a_1 = a_2 = 1.0$, $\epsilon = 0.01$, $J_{\mathrm{th}} = 0.5$, $\eta = 0.05$ m, $\phi_j(0) = 1$.
