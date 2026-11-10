"""Reset MuJoCo's kinematic drone to a reproducible state."""

import numpy as np
import mujoco


def reset_drone(
    model,
    data,
    position=(0.0, 1.0, 2.0),
    yaw=0.0,
    pitch=np.pi / 3,
):
    """Reset simulation time, drone position, and gimbal state."""

    position = np.asarray(position, dtype=float)

    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Initial position must be a finite 3D vector")

    if not np.isfinite(yaw) or not np.isfinite(pitch):
        raise ValueError("Initial gimbal angles must be finite")

    body_id = model.body("drone_1").id
    mocap_id = model.body_mocapid[body_id]

    yaw_id = model.joint("drone_1_yaw_joint").id
    pitch_id = model.joint("drone_1_pitch_joint").id

    if mocap_id < 0:
        raise ValueError("drone_1 is not a mocap body")

    pitch_min, pitch_max = model.jnt_range[pitch_id]

    if pitch < pitch_min - 1e-8 or pitch > pitch_max + 1e-8:
        raise ValueError("Initial pitch is outside joint limits")

    # Clear simulation state and reset simulation time.
    mujoco.mj_resetData(model, data)

    # Restore drone pose.
    data.mocap_pos[mocap_id] = position
    data.mocap_quat[mocap_id] = [1.0, 0.0, 0.0, 0.0]

    # Restore yaw and pitch.
    yaw_idx = model.jnt_qposadr[yaw_id]
    pitch_idx = model.jnt_qposadr[pitch_id]

    data.qpos[yaw_idx] = (yaw + np.pi) % (2 * np.pi) - np.pi
    data.qpos[pitch_idx] = np.clip(
        pitch, pitch_min, pitch_max
    )

    # Ensure zero initial joint velocities.
    data.qvel[:] = 0.0

    # Recompute body and camera poses.
    mujoco.mj_forward(model, data)
