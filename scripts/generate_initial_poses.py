"""Generate fixed, collision-free initial poses for experiments."""

from pathlib import Path
import numpy as np
import yaml
import mujoco

from cr_motion_control.safety import load_safety, check_position
from cr_mujoco_bridge.sim_reset import reset_drone

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "experiments/configs/initial_poses.yaml"

if OUTPUT.exists():
    raise SystemExit(
        f"{OUTPUT} already exists. Refusing to overwrite fixed poses."
    )

with open(ROOT / "experiments/configs/phase1_main.yaml") as f:
    config = yaml.safe_load(f)

safety = load_safety(ROOT / "configs/controller.yaml")

main_seeds = config["experiment"]["seeds"]
pilot_seeds = config["experiment"]["pilot_seeds"]
reserved_seeds = list(range(5, 15))

seeds = sorted(set(main_seeds + pilot_seeds + reserved_seeds))

model = mujoco.MjModel.from_xml_path(
    str(ROOT / "environments/mjcf/indoor_room.xml")
)
data = mujoco.MjData(model)

mocap_id = model.body_mocapid[model.body("drone_1").id]

poses = {}

for seed in seeds:
    rng = np.random.default_rng(
        np.random.SeedSequence([20261110, seed])
    )

    for attempt in range(10000):
        position = [
            round(float(rng.uniform(-1.4, 1.4)), 6),
            round(float(rng.uniform(-1.4, 1.4)), 6),
            round(float(rng.uniform(1.8, 2.3)), 6),
        ]

        safe, _ = check_position(position, safety)

        if safe:
            break
    else:
        raise RuntimeError(f"No safe position for seed {seed}")

    yaw = round(float(rng.uniform(-np.pi, np.pi)), 6)
    pitch = round(
        float(rng.uniform(np.pi / 3 + 0.05, np.pi / 2 - 0.05)),
        6
    )

    reset_drone(model, data, position, yaw, pitch)

    assert np.allclose(data.mocap_pos[mocap_id], position)
    assert np.isclose(data.time, 0.0)

    poses[seed] = {
        "position_m": position,
        "yaw_rad": yaw,
        "pitch_rad": pitch,
    }

OUTPUT.parent.mkdir(parents=True, exist_ok=True)

with open(OUTPUT, "w") as f:
    yaml.safe_dump(
        {
            "generator_seed": 20261110,
            "poses": poses,
        },
        f,
        sort_keys=False
    )

print(f"[PASS] Generated {len(poses)} initial poses")
print("[PASS] All positions passed workspace and keep-out checks")
print("[PASS] All poses passed MuJoCo reset validation")
print(f"Saved: {OUTPUT}")
print("INITIAL POSES TEST: PASSED")
