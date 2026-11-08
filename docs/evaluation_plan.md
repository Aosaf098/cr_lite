# Evaluation Plan and Success Criteria

**Purpose:** This document fixes, *before any results exist*, what counts as a working system and what counts as "feedback helps". Fixing these rules in advance keeps the conclusion honest: thresholds, metrics, and the decision rule cannot be adjusted after the results are seen.

**Related documents:** `docs/deviations.md` (D-numbers) and `docs/conventions.md` (T-tests).

**Change control:** Any change to this file after main trials begin must be recorded in Section 8, with the date and the reason.

---

## 1. Two Kinds of Success

| Kind | Question | Where |
|---|---|---|
| **Engineering** | Does the system work correctly, safely, and on time? | Section 2. All criteria must pass for Phase 1 to be complete. |
| **Scientific** | Does mesh-change feedback improve reconstruction? | Sections 3–6. Any answer, including a negative or null one, is acceptable if it is reproducible. |

---

## 2. Engineering Acceptance Criteria

Thresholds are initial values. They may be tightened, but are only loosened with a recorded reason (Section 8).

| ID | Criterion | Measurement | Pass condition |
|---|---|---|---|
| E1 | Conventions correct | Tests T1–T9 (`conventions.md` §9) | All pass |
| E2 | TSDF reconstruction accurate | Scripted orbit around $\mathcal{B}$, noise-free depth, then F-score against ground truth (GT) within $\mathcal{B}$ | Precision ≥ 0.95, Recall ≥ 0.85 at $\eta = 0.05$ m |
| E3 | Mesh feedback correct | Synthetic mesh tests | (a) Identical meshes give $h_2 \equiv 0$; (b) a local change concentrates ≥ 90% of $\sum h_2$ within $2\sigma_4$ of the change; (c) out-of-order versions are rejected |
| E4 | Gradients correct | $\partial h_1/\partial p$, $\xi_1$, and all CBF gradients compared against central finite differences at 1,000 random states | Relative error < $10^{-4}$ |
| E5 | QP reliable and fast | All main-trial control ticks | Solver failure rate < 1%; p95 solve time < 20 ms |
| E6 | Safety | All main trials | Zero collisions; hard barriers ≥ $-10^{-3}$ at every tick; drone never leaves $\mathcal{P}$ |
| E7 | Real-time behaviour | All main trials | Controller period p99 < 1.5× nominal; dropped observations < 5%; snapshot-to-feedback latency p95 < 2 s |
| E8 | Missions complete | All main trials | No unhandled exceptions; the run reaches the time budget or the J-completion condition |
| E9 | Reproducible | Every run | Run directory contains the resolved config, seed, git commit hash, dependency versions, rosbag, and metrics CSV |

---

## 3. Scientific Question and Hypotheses

**Primary question** (from `ARCHITECTURE.md` §7): Does online mesh-change feedback improve reconstruction quality or observation efficiency compared with the same controller without feedback?

- **H1 (primary):** With noise-free depth, final F-score is higher with feedback (condition C1) than without (C0).
- **H2 (secondary):** With noisy depth, final F-score is higher with feedback (C3) than without (C2).
- **H3 (secondary):** Feedback reaches a given F-score with fewer frames. This is measured as the area under the F-score-vs-time curve.

**Prior expectation, stated honestly:** H1 may fail even if the implementation is correct. With perfect depth, a TSDF mesh changes mainly when a surface is first seen, not as it refines (D1). H2 tests whether depth noise restores the effect the paper reports with NeuralRecon. The paper's own single-drone gain was +0.084 F-score (0.354 → 0.438).

---

## 4. Experimental Design

### 4.1 Conditions

| Condition | `feedback_enabled` | Depth noise | Purpose |
|---|---|---|---|
| C0 | false | off | Baseline for H1 |
| C1 | true | off | H1 |
| C2 | false | on | Baseline for H2 |
| C3 | true | on | H2 |

Everything else is identical across all four conditions: scene, camera, TSDF settings, $\mathcal{Q}$, controller gains, initial poses, and time budget.

### 4.2 Observation Budget

The budget is a **fixed simulated mission time** $T$. Since the camera runs at a fixed 5 Hz, this also fixes the number of frames ($5T$) for every condition.

Missions are *not* ended when $J$ reaches a threshold. Feedback raises $J$ by design (Eq. 4), so a $J$-based stop would give the feedback condition more time and bias the comparison toward it.

- **Checkpoints:** $T/4$, $T/2$, $3T/4$, and $T$. A mesh is extracted at each checkpoint for evaluation. These four points are the "multiple observation budgets" in the checklist.
- **Choosing $T$:** set during the pilot (Section 4.4) as the smallest round value at which C0 reaches $J \le 0.05$ in all pilot seeds. The provisional value is 600 s.

### 4.3 Seeds and Initial Conditions

- Seeds: $\{0, 1, 2, 3, 4\}$. Each seed selects:
  - an initial drone pose, from a list generated once (inside $\mathcal{P}$, outside keep-out boxes) and saved to `experiments/configs/initial_poses.yaml`;
  - the depth-noise random stream (C2 and C3 only).
- **Pairing:** each seed is run in every condition, and comparisons are paired by seed.

### 4.4 Pilot Phase (Tuning)

The pilot uses separate seeds $\{100, 101, 102\}$, so tuning never sees the evaluation seeds. It sets:

| Parameter | Set by |
|---|---|
| $\gamma$ | D7 sweep $\{0.001, 0.003, 0.01\}$; pick the value with the lowest slack activity that still decreases $J$ steadily |
| $\kappa$ | D9 median-calibration procedure |
| $T$ | Section 4.2 rule |
| $\sigma_2$ | Formula vs paper value; keep the formula unless it clearly fails |

Pilot seeds are **feedback-off** runs only, so tuning cannot favour the feedback condition.

**Freeze:** after the pilot, write the values into `phase1_main.yaml`, commit, and tag the commit `experiments-frozen`. All main trials must run from that tag.

### 4.5 Depth Noise Model (C2, C3)

Axial Gaussian noise that grows with depth, using the commonly used structured-light model of Nguyen et al. (2012):

$$\sigma_z(z) = 0.0012 + 0.0019\,(z - 0.4)^2 \quad \text{(metres)}$$

Noise is applied per pixel in `rgbd_preprocessor`. Poses stay ground truth.

---

## 5. Metrics

All reconstruction metrics are computed **inside $\mathcal{B}$ only**. Ground truth is sampled uniformly from the MJCF primitives at 1 cm spacing (D19).

### 5.1 Reconstruction

| Metric | Definition |
|---|---|
| **F-score@5 cm (primary)** | Harmonic mean of Precision and Recall at $\eta = 0.05$ m (paper §V-B) |
| Precision | Fraction of reconstructed points within $\eta$ of ground truth |
| Recall | Fraction of ground-truth points within $\eta$ of the reconstruction. This is the "completeness" metric. |
| F-score@2.5 cm | Stricter check of fine detail |
| Chamfer distance (mean, m) | Threshold-free summary |
| F-score AUC | Trapezoidal area under F-score at the checkpoints, normalized by $T$ (H3) |

### 5.2 Coverage and Behaviour

- $J(t)$
- Number of feedback jumps applied
- Total $\sum h_2$
- Trajectory length (m)
- Total gimbal rotation (rad)
- Frames integrated
- Slack activity: fraction of ticks with $w > 10^{-6}$

### 5.3 Computation

For each of TSDF integration, mesh extraction, mesh feedback, coverage computation, and QP solve: median, p95, and max latency. Also mean and peak CPU and RAM per node.

### 5.4 Reliability

Collisions, solver failures, stale-state skips, rejected feedback messages, and dropped frames.

---

## 6. Analysis and Decision Rule

For each hypothesis, compute the paired difference per seed, for example $\Delta_s = F_{\mathrm{C1},s} - F_{\mathrm{C0},s}$. Report:

- every per-seed value;
- the mean $\bar\Delta$ and standard deviation;
- the 95% confidence interval: a $t$-interval, plus a bootstrap interval with 10,000 resamples.

**Decision rule**, fixed in advance:

| Outcome | Condition |
|---|---|
| **Feedback helps** | CI lower bound > 0 **and** $\bar\Delta \ge 0.02$ (smallest effect considered practically meaningful) |
| **Feedback hurts** | CI upper bound < 0 |
| **Inconclusive** | Otherwise. Increase seeds to 15 per condition, with seeds 5–14 added identically to all conditions, and re-analyse once. Report both analyses. |

**Not allowed:** dropping seeds, changing $\eta$, or changing the primary metric after seeing results.

**Invalid trials:** a run is invalid **only** for infrastructure failures: a crash, more than 5% dropped frames, or a missing log. Invalid runs are re-run with the same seed and counted in the report. Collisions and poor scores are **not** grounds for exclusion; they are results.

**Mechanism check:** for at least one seed, plot where $h_2$ was large and where feedback-on recall improved. If feedback helps, those places should overlap. If they don't, any improvement is not explained by the paper's mechanism, and the report says so.

---

## 7. Deliverable Figures

1. $J(t)$, all conditions, mean ± std across seeds.
2. F-score versus checkpoint time, all conditions.
3. Paired per-seed final F-score (C0 vs C1, C2 vs C3), drawn as connected dots.
4. Final meshes, coloured by error, for the median seed in each condition.
5. Importance field and $h_2$ snapshots at 3 time points (feedback-on).
6. Latency box plots per component.

---

## 8. Change Log

| Date | Change | Reason |
|---|---|---|
| 2026-11-08 | Initial version | — |
