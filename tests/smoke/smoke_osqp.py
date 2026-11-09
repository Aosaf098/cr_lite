#!/usr/bin/env python3
"""Smoke test: OSQP 1.x on a QP shaped like the Phase 1 controller.

Decision variable u = [vx, vy, vz, yaw_rate, pitch_rate]  (VelocityCommand5D)
Objective:   min 0.5 * ||u - u_des||^2     (track a desired command)
Constraints: input bounds from controller.input_bounds, plus one extra
             half-space row standing in for a safety barrier (vx <= 0.2).

Because every constraint here is axis-aligned, the exact solution is simply
u* = clip(u_des, lower, upper), so OSQP's answer can be checked analytically.

Checks:
  Q1  Known answer: one solve matches the analytical solution
  Q2  Infeasibility is detected and reported (needed for the safe fallback)
  Q2b Invalid problem data raises an exception at setup (must be caught)
  Q3  Repeated solves with update() + warm start stay correct
  Q4  Solve time is far below the 200 ms budget of the 5 Hz controller

Run from the repo root with the venv active:
    python tests/smoke/smoke_osqp.py
"""
import time

import numpy as np
import osqp
from scipy import sparse

# controller.input_bounds from experiments/configs/phase1_main.yaml
LIN, YAW, PITCH = 0.5, 0.5, 0.3
BARRIER_VX_MAX = 0.2           # stand-in for a CBF / workspace row
TOL = 1e-3

N = 5
lower = np.array([-LIN, -LIN, -LIN, -YAW, -PITCH])
upper = np.array([LIN, LIN, LIN, YAW, PITCH])

# A = [I ; e_vx]   ->   lower <= u <= upper,   -inf <= vx <= 0.2
A = sparse.vstack([sparse.eye(N), sparse.csr_matrix(([1.0], ([0], [0])), shape=(1, N))]).tocsc()
l = np.concatenate([lower, [-np.inf]])
u = np.concatenate([upper, [BARRIER_VX_MAX]])
P = sparse.triu(sparse.eye(N), format="csc")   # OSQP uses the upper triangle of P

eff_upper = upper.copy()
eff_upper[0] = min(upper[0], BARRIER_VX_MAX)

results = []


def check(name, ok, detail):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def solved(res):
    return res.info.status.lower() == "solved"


def main():
    print(f"OSQP {osqp.__version__}")

    # --- Q1: known answer ---
    u_des = np.array([1.0, -0.2, 0.1, 0.8, -0.5])
    expected = np.clip(u_des, lower, eff_upper)          # [0.2, -0.2, 0.1, 0.5, -0.3]
    prob = osqp.OSQP()
    prob.setup(P, -u_des, A, l, u, verbose=False, eps_abs=1e-6, eps_rel=1e-6)
    res = prob.solve()
    err = np.max(np.abs(res.x - expected)) if solved(res) else np.inf
    check("Q1 known answer", solved(res) and err < TOL,
          f"status='{res.info.status}', x={np.round(res.x, 4)}, max error {err:.1e}")

    # --- Q2: infeasible problem must be reported, not silently 'solved' ---
    # Two rows that contradict each other: vx >= 1.0 and vx <= 0.5.
    # (A single row with l > u is rejected at setup instead; see Q2b.)
    A_bad = sparse.vstack([sparse.eye(N), sparse.csr_matrix(([1.0], ([0], [0])), shape=(1, N))]).tocsc()
    l_bad = np.concatenate([[1.0], lower[1:], [-np.inf]])
    u_bad = np.concatenate([[np.inf], upper[1:], [0.5]])
    bad = osqp.OSQP()
    bad.setup(P, -u_des, A_bad, l_bad, u_bad, verbose=False)
    res_bad = bad.solve()
    check("Q2 infeasibility detected", "infeasible" in res_bad.info.status.lower(),
          f"status='{res_bad.info.status}'")

    # --- Q2b: invalid data (l > u on one row) raises at setup ---
    # The controller must catch this and fall back safely, not crash.
    try:
        osqp.OSQP().setup(P, -u_des, sparse.eye(N, format="csc"),
                          np.full(N, 1.0), np.full(N, 0.5), verbose=False)
        check("Q2b invalid data raises", False, "setup accepted l > u")
    except Exception as exc:  # noqa: BLE001
        check("Q2b invalid data raises", True, f"{type(exc).__name__} raised as expected")

    # --- Q3 / Q4: 5 Hz-style loop with update() and warm start ---
    rng = np.random.default_rng(0)
    n_solves, max_err, n_fail, times = 1000, 0.0, 0, []
    x_prev = res.x.copy()
    for _ in range(n_solves):
        u_des = rng.uniform(-1.0, 1.0, N)
        t0 = time.perf_counter()
        prob.update(q=-u_des)
        prob.warm_start(x=x_prev)
        r = prob.solve()
        times.append(1000.0 * (time.perf_counter() - t0))
        if not solved(r):
            n_fail += 1
            continue
        x_prev = r.x.copy()
        max_err = max(max_err, np.max(np.abs(r.x - np.clip(u_des, lower, eff_upper))))
    check("Q3 update + warm start", n_fail == 0 and max_err < TOL,
          f"{n_solves} solves, {n_fail} failures, max error {max_err:.1e}")
    check("Q4 solve time", np.percentile(times, 99) < 5.0,
          f"mean {np.mean(times):.3f} ms, 99th pct {np.percentile(times, 99):.3f} ms "
          f"(controller budget 200 ms)")

    print("\nOSQP SMOKE TEST:", "PASSED" if all(results) else "FAILED")


if __name__ == "__main__":
    main()
