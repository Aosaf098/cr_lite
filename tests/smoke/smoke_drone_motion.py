#!/usr/bin/env python3
"""Stage 3.5: verify single-integrator translation of a MuJoCo mocap drone.

Run from the repository root with the project virtual environment active:
    python tests/smoke/smoke_drone_motion.py

This test deliberately does not implement collision avoidance, gimbal actuation,
or command reception. Those are separate future stages.
"""
from pathlib import Path

import mujoco
import numpy as np

SCENE = Path("environments/mjcf/indoor_room.xml")
INITIAL_POSITION = np.array([0.0, 1.0, 2.0])  # metres, world frame
VELOCITY = np.array([0.25, -0.20, 0.10])  # metres/second, world frame
DURATION_S = 2.0


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_1")
    assert drone_id >= 0, "drone_1 missing from MJCF"
    mocap_id = int(model.body_mocapid[drone_id])
    assert mocap_id >= 0, "drone_1 must be a mocap body"

    # Set the known initial position, independently of the previous simulation state.
    data.mocap_pos[mocap_id] = INITIAL_POSITION
    mujoco.mj_forward(model, data)
    assert np.allclose(data.xpos[drone_id], INITIAL_POSITION, atol=1e-9)

    dt = float(model.opt.timestep)
    n_steps = round(DURATION_S / dt)
    assert np.isclose(n_steps * dt, DURATION_S), "duration must align with timestep"

    # Single integrator: x_{k+1} = x_k + v * dt.
    for _ in range(n_steps):
        data.mocap_pos[mocap_id] += VELOCITY * dt
        mujoco.mj_step(model, data)

    expected = INITIAL_POSITION + VELOCITY * DURATION_S
    actual = data.xpos[drone_id].copy()
    assert np.allclose(actual, expected, atol=1e-7), (
        f"Position mismatch: got {actual}, expected {expected}"
    )

    print("[PASS] Mocap drone moved using velocity integration")
    print(f"Timestep: {dt:.3f} s; steps: {n_steps}")
    print("Initial position:", INITIAL_POSITION)
    print("Commanded velocity:", VELOCITY)
    print("Expected final position:", np.round(expected, 4))
    print("Actual final position:  ", np.round(actual, 4))

    # A zero command must preserve position.
    for _ in range(round(1.0 / dt)):
        data.mocap_pos[mocap_id] += np.zeros(3) * dt
        mujoco.mj_step(model, data)
    assert np.allclose(data.xpos[drone_id], actual, atol=1e-7)
    print("[PASS] Zero velocity holds position")
    print("DRONE MOTION TEST: PASSED")


if __name__ == "__main__":
    main()
