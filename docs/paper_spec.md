# Paper Specification — Coverage-Recon

**Source:** M. Hanif, R. Terunuma, T. Sumino, K. Cheng, T. Hatanaka, *Coverage-Recon: Coordinated Multi-Drone Image Sampling with Online Map Feedback*, arXiv:2510.18347v1, Oct 2025.
**Project page:** https://htnk-lab.github.io/coverage-recon/

**Purpose of this document:** A faithful record of the paper's formulation, with the paper's equation numbers where they exist. It describes *what the paper says*. Decisions about what this project changes belong in `deviations.md`. Issues found while transcribing are collected in Section 9 and are not silently fixed here.

---

## 1. State, Virtual Field, and Operators (Sec. II-A)

**Drones:** $\mathcal{I} = \{1,\dots,n\}$. Phase 1 of this project uses $n = 1$.

**Drone state** (position + gimbal yaw and pitch):

$$p_i = [x_i, y_i, z_i, \theta_i^h, \theta_i^v]^\top \in \mathcal{X} = \mathbb{R}^3 \times \Theta^h \times \Theta^v$$

- $\theta^h \in \Theta^h = [-\pi, \pi]$: camera yaw
- $\theta^v \in \Theta^v = (0, \pi/2]$: camera pitch ($\pi/2$ = straight down, per the "fixed downward-facing camera at 90°" baselines)
- Body frame $\Sigma_i$ is attached to the camera, with its z-axis along the optical axis. Roll is not controlled (Remark 1).

**Dynamics** (single integrator, all five components directly actuated):

$$\dot p_i = u_i, \qquad u_i \in \mathcal{U} \subseteq \mathbb{R}^5$$

**Regions:**
- $\mathcal{P} \subset \mathbb{R}^3$: flight region
- $\mathcal{B} \subset \mathbb{R}^3$: reconstruction target region

**Virtual field** (5D):

$$\mathcal{Q}_c = \mathcal{B} \times \Theta^h \times \Theta^v$$

discretized into $m$ cells. The representative point of cell $j$ is the **observation point**
$q_j = [x_j, y_j, z_j, \theta_j^h, \theta_j^v]^\top$, and $\mathcal{Q} = \{q_j \mid j \in \mathcal{M} = \{1,\dots,m\}\}$.

**Operators:**

$$\mathrm{Pos}(\cdot) = [x,y,z]^\top, \qquad \mathrm{Ang}(\cdot) = [\theta^h,\theta^v]^\top, \qquad \mathrm{Dir}(\cdot) \in S^2$$

with $\mathrm{Dir}$ built from $\cos\theta^h\cos\theta^v$, $\sin\theta^h\cos\theta^v$ and a third (vertical) component. See Issue 9.1.

- $\mathrm{Dir}(p_i)$: camera optical axis in the world frame
- $\mathrm{Dir}(q_j)$: required viewing direction for point $q_j$

---

## 2. Sensing Performance $h_1$ (Sec. II-B)

Three measures for drone state $p_i$ observing point $q_j$:

**Optical-axis misalignment** (is the point centred in the image?):

$$\varphi_1(p_i,q_j) = \arccos\!\left(\frac{\mathrm{Dir}(p_i)\cdot(\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i))}{\|\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i)\|}\right)$$

**Viewing-direction mismatch** (is the point seen from its required direction?):

$$\varphi_2(p_i,q_j) = \arccos\!\left(-\frac{\mathrm{Dir}(q_j)\cdot(\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i))}{\|\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i)\|}\right)$$

**Distance error** from the ideal observation distance $D$:

$$\rho(p_i,q_j) = \|\mathrm{Pos}(q_j)-\mathrm{Pos}(p_i)\| - D$$

**Sensing performance** — Eq. (1):

$$h_1(p_i,q_j) = \exp\!\left(-\frac{(1-\cos\varphi_1)^2}{2\sigma_1^2}\right)\exp\!\left(-\frac{(1-\cos\varphi_2)^2}{2\sigma_2^2}\right)\exp\!\left(-\frac{\rho^2}{2\sigma_3^2}\right) \in (0,1]$$

Parameter guidance from the paper:
- $\sigma_1$: tuned to the camera's field of view.
- $\sigma_2$: chosen so performance is 0.5 halfway between adjacent angular samples:
  $$\sigma_2 = \sqrt{\frac{(1-\cos(\Delta\theta/2))^2}{2\log 2}}$$
  where $\Delta\theta$ is the angular spacing.
- $\sigma_3$: tolerance on distance, tighter for higher-resolution cameras.

Implementation note: $h_1$ uses only $\cos\varphi$, so it can be computed from dot products without calling `arccos`.

---

## 3. Importance Index and Objective (Sec. II-C)

Initial condition: $\phi_j(0) = 1$ for all $j$.

**Importance decay** — Eq. (2):

$$\dot\phi_j = -\delta_1 \max_{i\in\mathcal{I}} h_1(p_i,q_j)\,\phi_j$$

**Global objective** — Eq. (3):

$$J = \frac{1}{m}\sum_{j=1}^{m}\phi_j$$

So $J(0) = 1$, and the mission is complete when $J \to 0$.

---

## 4. Online Map Feedback (Sec. III-A)

The mesh $(V^t, F^t)$ is updated at discrete events $t \in \mathcal{T} = \{1,2,\dots\}$.

Feedback function: $h_2(q_j,t) : \mathcal{Q}\times\mathcal{T} \to \mathbb{R}_{\ge 0}$, which is large where the mesh changed a lot.

**Importance jump** at each update event — Eq. (4):

$$\phi_j(t^+) = \phi_j(t^-) + \delta_2\, h_2(q_j,t)$$

Interpretation (the paper's own framing): regions still changing after being observed need more observations; regions that have stabilized do not.

---

## 5. Mesh-Change Quantification (Sec. III-B)

Vertex set at event $t$: $V^t = \{v_\ell^t \in \mathbb{R}^3 \mid \ell = 1,\dots,N^t\}$.

### 5.1 3D Grid-Based Method (the one this project implements)

Uniform evaluation grid:

$$\mathcal{R} = \{r_k \in \mathbb{R}^3 \mid k = 1,\dots,s\}, \qquad s < |\mathcal{Q}|$$

Vertices assigned to their **nearest** grid point, counting only vertices inside $\mathcal{B}$:

$$V_k^t = \{\ell \mid r_k = \arg\min_{r\in\mathcal{R}}\|r - v_\ell^t\|,\; v_\ell^t \in \mathcal{B}\}$$

**Local change** = change in vertex count — Eq. (5):

$$\Delta V^t(r_k) = |V_k^t| - |V_k^{t-1}|$$

**Feedback** — Eq. (6):

$$h_2(q_j,t) = \sum_{r_k\in\mathcal{R}} \tanh^2\!\left(\left(\frac{\Delta V^t(r_k)}{\kappa}\right)^2\right)\exp\!\left(-\frac{\|\mathrm{Pos}(q_j)-r_k\|^2}{2\sigma_4^2}\right)$$

- $\kappa > 0$: sensitivity normalization
- $\sigma_4 > 0$: spatial influence radius
- The Gaussian weight matrix ($m \times s$) depends only on positions and can be **precomputed**.
- Note: this measures change in local vertex *density*, not geometric displacement.

### 5.2 M3C2-Based Method (reference only — not implemented in Phase 1)

Eq. (7) replaces the $\tanh^2$ term with the M3C2 distance $L_{\mathrm{M3C2}}(r_k,t) \ge 0$ between consecutive vertex clouds. The paper reports a slightly better F-score (0.442 vs 0.438) at roughly 9× the cost (1.86 s vs 0.21 s per update).

### 5.3 Feedback Activation (Sec. IV-C) — Eq. (23)

$$\delta_2 = \begin{cases} 0 & J \ge J_{\mathrm{th}} \\ \delta_2' & J < J_{\mathrm{th}} \end{cases}$$

This gives exploration first, then refinement. The paper uses $J_{\mathrm{th}} = 50\%$, i.e. $J < 0.5$ since $J(0) = 1$.

---

## 6. Controller (Sec. IV)

### 6.1 Sampling-Performance Constraint

Required decay rate between map updates — Eq. (8):

$$\dot J < -\gamma$$

Expanding $\dot J$ with Eq. (2) — Eq. (9):

$$\dot J = -\frac{1}{m}\sum_{i=1}^{n} I_i$$

with the Voronoi-like partition

$$\mathcal{V}_i(p) = \{j \mid h_1(p_i,q_j) \ge h_1(p_k,q_j)\ \forall k\}$$

(i.e. drone $i$ is the best observer of $q_j$; see Issue 9.6) and the per-drone contribution

$$I_i = \sum_{j\in\mathcal{V}_i}\delta_1 h_1(p_i,q_j)\phi_j \qquad \text{— Eq. (10)}$$

**Theorem IV.1:** if every drone satisfies $-I_i/|\mathcal{V}_i| \le -\gamma$ (Eq. 11), then Eq. (8) holds globally.

**CBF** — Eq. (12):

$$b_{i,I} = I_i - |\mathcal{V}_i|\gamma \ge 0$$

**For $n = 1$:** $\mathcal{V}_1 = \mathcal{M}$ (all points), $|\mathcal{V}_1| = m$.

### 6.2 Gimbal Pitch Constraint — Eq. (14)

$$b_{i,\theta^v} = (\theta^v_{\max}-\theta^v_{\min})^2 - \left(\theta_i^v - \tfrac{\theta^v_{\min}+\theta^v_{\max}}{2}\right)^2 \ge 0$$

See Issues 9.2 and 9.3.

### 6.3 Inter-Drone Collision Avoidance — Eq. (16)

$$b_{i,\mathrm{ca}} = \min_{j\ne i}\|\mathrm{Pos}(p_i)-\mathrm{Pos}(p_j)\|^2 - d^2 \ge 0$$

**Not applicable for $n = 1$.**

### 6.4 Other Constraints (Remark 2)

The paper adds a flight-region constraint to keep drones inside $\mathcal{P}$, and an input bound $u^{\max}$ through $\mathcal{U}$, but does not give their equations. Obstacle avoidance against scene geometry is **not** part of the paper.

### 6.5 QP — Eq. (19), closed form of Eq. (18) with linear class-K functions $\alpha_k(b) = a_k b$

$$(u_i^*, w_i^*) = \arg\min_{u_i\in\mathcal{U},\, w_i\in\mathbb{R}}\ \epsilon\|u_i\|^2 + w_i^2$$

subject to:

$$\xi_{1i}^\top u_i + \xi_{2i} \ge w_i \quad \text{(sampling performance, soft via slack } w_i\text{)}$$

$$\chi_{1i}^\top u_i + \chi_{2i} \ge 0 \quad \text{(gimbal pitch, hard)}$$

$$\rho_{1i}^\top u_i + \rho_{2i} \ge 0 \quad \text{(collision, hard)}$$

with coefficients — Eqs. (20)–(22):

$$\xi_{1i} = \delta_1\sum_{j\in\mathcal{V}_i}\frac{\partial h_1(p_i,q_j)}{\partial p_i}\phi_j \in \mathbb{R}^5$$

$$\xi_{2i} = -a_1|\mathcal{V}_i|\gamma + \sum_{j\in\mathcal{V}_i}\delta_1 h_1(p_i,q_j)\,\phi_j\,\bigl(a_1 - \delta_1 h_1(p_i,q_j)\bigr)$$

$$\chi_{1i} = \bigl[0,0,0,0,\ 2(\theta_i^v - \bar\theta^v)\bigr]^\top, \qquad \chi_{2i} = a_2\, b_{i,\theta^v}, \qquad \bar\theta^v = \tfrac{\theta^v_{\min}+\theta^v_{\max}}{2}$$

$$\rho_{1i} = \bigl[2(\mathrm{Pos}(p_i)-\mathrm{Pos}(p_{j^*}))^\top,\ 0,\ 0\bigr]^\top, \qquad \rho_{2i} = a_3\, b_{i,\mathrm{ca}}$$

where $j^*$ is the nearest neighbour.

I re-derived $\xi_{2i}$ from $\dot b_{i,I}$ using Eq. (2) and it is consistent. $\chi_{1i}$ is **not** consistent (Issue 9.3).

The core implementation work for the controller is the analytical gradient $\partial h_1/\partial p_i \in \mathbb{R}^5$, which the paper does not write out.

---

## 7. Algorithm 1 (per control step)

1. Compute $J$ (Eq. 3).
2. If $J \ge J_{\mathrm{th}}$: $\delta_2 = 0$, update $\phi$ by Eq. (2) only.
3. Else: $\delta_2 = \delta_2'$, update $\phi$ by Eq. (2) and apply jumps by Eq. (4) on each new mesh.
4. Solve QP (19) for $u_i$.
5. Apply $u_i$; continue capturing images and reconstructing.

Implementation note: between controller ticks, Eq. (2) integrates exactly as $\phi_j \leftarrow \phi_j\exp(-\delta_1 h_1 \Delta t)$ for fixed $p$.

---

## 8. Paper Experimental Parameters (Sec. V-A)

| Quantity | Paper value |
|---|---|
| Target region $\mathcal{B}$ | $[-3,3]\times[-3,3]\times[0,2]$ m |
| Angle ranges | $\Theta^h = [-\pi,\pi)$, $\Theta^v = [\pi/3, \pi/2]$ |
| Discretization | $0.3$ m × $0.3$ m × $0.3$ m × $0.3$ rad × $0.3$ rad |
| Reported $m$ | $1.5\times10^7$ (see Issue 9.5) |
| Flight region $\mathcal{P}$ | $x,y \in [-4,4]$ m, $z \in [0.5, 5.0]$ m |
| Min. inter-drone distance $d$ | 1.0 m |
| Camera | DJI Mini 3 model: 24 mm (35 mm equiv.), 1920×1080, 20 fps |
| $D$ | 1.0 m |
| $\sigma_1, \sigma_2, \sigma_3, \sigma_4$ | 0.07, 0.095, 0.3, 0.4 |
| $\delta_1, \delta_2'$ | 3.0, 1.0 |
| $\kappa$ | 0.7 |
| $a_1, a_2, a_3$ | 1.0, 1.0, 1.0 |
| $\epsilon$ | 0.01 (a second "$\epsilon = 0.012$" is listed, see Issue 9.4) |
| $J_{\mathrm{th}}$ | 50% |
| Control / data rate | 20 Hz |
| Initial position (single drone) | $(0, 1, 2)$ m |
| Mission length | ~1500 s |
| Grid spacing for $\mathcal{R}$ | not stated |
| Pitch limits $\theta^v_{\min}, \theta^v_{\max}$ | not stated |
| $u^{\max}$ | not stated |

**Evaluation (Sec. V-B):** F-score at $\eta = 0.05$ m, with Precision and Recall from closest-point distances. The ground-truth mesh was uniformly resampled to match the reconstructed vertex density.

**Reported single-drone F-scores:**

| Method | F-score |
|---|---|
| No feedback | 0.354 |
| 3D Grid feedback | 0.438 |
| M3C2 feedback | 0.442 |
| Lawnmower | 0.324 |
| Persistent coverage | 0.191 |
| Angle-aware, fixed camera | 0.348 |
| Angle-aware, active camera | 0.351 |

---

## 9. Issues and Ambiguities Found in the Paper

Each issue gets a resolution in `deviations.md`. **None are resolved here.**

**9.1 `Dir` is incomplete and the sign convention is unclear.** The definition shows only two components (the vertical one is missing in the arXiv text). With pitch positive-down in a z-up world, the camera axis should be $[\cos\theta^h\cos\theta^v,\ \sin\theta^h\cos\theta^v,\ -\sin\theta^v]$. However, if the *same* formula is used for $\mathrm{Dir}(q_j)$, then the minus sign in $\varphi_2$ gives $\varphi_2 = \pi$ for the ideal configuration instead of 0. One of $\mathrm{Dir}(q_j)$ or the sign in $\varphi_2$ must be flipped.

*Required test regardless of choice:* for $\mathrm{Pos}(p) = \mathrm{Pos}(q) - D\,\mathrm{Dir}_{\mathrm{view}}(q)$ and $\mathrm{Ang}(p) = \mathrm{Ang}(q)$, we must get $\varphi_1 = \varphi_2 = 0$, $\rho = 0$, $h_1 = 1$.

**9.2 Pitch CBF range is doubled.** As written, Eq. (14) allows $\theta^v$ within $\bar\theta^v \pm (\theta_{\max}-\theta_{\min})$, which is twice the intended range. The intended form is likely $\left(\frac{\theta_{\max}-\theta_{\min}}{2}\right)^2 - (\theta^v - \bar\theta^v)^2$.

**9.3 Pitch CBF gradient has the wrong sign.** $\partial b_{\theta^v}/\partial\theta^v = -2(\theta^v - \bar\theta^v)$, but Eq. (21a) gives $+2(\cdot)$. With the paper's sign the constraint pushes the pitch *toward* the limits. This must be checked with a finite-difference test.

**9.4 Duplicate $\epsilon$.** Two values are given (0.01 and 0.012). The second is probably $\gamma$, which is otherwise never given a value. This is unconfirmed.

**9.5 Reported $m$ doesn't match the stated discretization.** $20\times20\times\sim7$ positions × $\sim21$ yaw × $\sim2$ pitch cells ≈ $1.2\times10^5$, not $1.5\times10^7$. Either the grid or the count differs from what is stated.

**9.6 Voronoi inequality direction.** As written, $\mathcal{V}_i$ uses $h_1(p_i,q_j) \le h_1(p_k,q_j)$, which would assign each point to the *worst* observer. This contradicts the $\max$ in Eq. (2); it should be $\ge$. This is irrelevant for $n = 1$.

**9.7 $\sigma_2$ value vs formula.** With $\Delta\theta = 0.3$ rad the formula gives $\sigma_2 \approx 0.0095$, but the paper uses 0.095, a 10× difference. Either it's a typo or the value was hand-tuned.

**9.8 $\kappa$ is tied to NeuralRecon's mesh density.** With $\kappa = 0.7$, a change of a single vertex gives $\tanh^2((1/0.7)^2) \approx 0.93$, which is already near saturation. A TSDF mesh at 3 cm voxels has a very different vertex density per grid cell, so $\kappa$ (and the spacing of $\mathcal{R}$) **cannot be copied** and must be re-tuned for Open3D.

**9.9 No occlusion in $h_1$.** The sensing model is purely geometric. Points hidden behind objects still receive coverage credit and decay.

**9.10 Missing equations.** The flight-region CBF, the input bounds $\mathcal{U}$, the grid spacing of $\mathcal{R}$, and the pitch limits are mentioned but not specified.
