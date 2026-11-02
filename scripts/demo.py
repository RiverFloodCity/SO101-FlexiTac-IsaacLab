# ruff: noqa: E402
# Thread settings and launcher parsing must precede simulator imports.
# SPDX-License-Identifier: Apache-2.0
import argparse
import json
import os
import time
from pathlib import Path

for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(k, "1")
from isaaclab.app import add_launcher_args, launch_simulation

p = argparse.ArgumentParser()
p.add_argument("--robot", type=Path, required=True)
p.add_argument("--rest", type=Path, required=True)
p.add_argument("--output_dir", type=Path, required=True)
p.add_argument("--seed", type=int, default=42)
p.add_argument("--camera", choices=["true", "false"], default="true")
p.add_argument("--render", choices=["true", "false"], default="true")
p.add_argument("--mode", choices=["hold", "grasp", "sweep"], default="grasp")
p.add_argument("--steps", type=int, default=580)
add_launcher_args(p)
a = p.parse_args()
if a.steps <= 30:
    p.error("--steps must exceed the 30-step timing warmup")
a.output_dir.mkdir(parents=True, exist_ok=True)
import subprocess
import sys

import numpy as np

from so101_flexitac.isaaclab.demo import DemoEnv, make_cfg
from so101_flexitac.kinematics import forward
from so101_flexitac.spec import JOINTS, LIMITS

rest = json.loads(a.rest.read_text())
if rest.get("operator_confirmed") is not True:
    raise ValueError("Explicit operator-confirmed rest input required")
q0 = rest["joint_positions_rad"]
subprocess.run(
    [
        sys.executable,
        str(Path(__file__).with_name("build_demo_scene.py")),
        "--output_dir",
        str(a.output_dir),
    ],
    check=True,
)
scene = a.output_dir / "objects.usda"
cfg = make_cfg(
    a.robot.resolve(),
    scene.resolve(),
    q0,
    a.output_dir,
    a.camera == "true",
    a.render == "true",
)
cfg.sim.device = a.device
cfg.seed = a.seed
with launch_simulation(cfg, a):
    import torch
    import warp as wp
    from PIL import Image
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    torch.set_num_threads(1)
    env = DemoEnv(cfg)
    env.reset()
    from so101_flexitac.isaaclab.inspection import check_initialization

    initialization = check_initialization(env, a.output_dir)
    (a.output_dir / "initialization.json").write_text(json.dumps(initialization, indent=2))
    if initialization["status"] != "PASS":
        raise RuntimeError("Illegal demo initialization; inspect initialization.json")
    # Capture t=0 once so 60/30 Hz sample phases coincide at every eighth physics step.
    env.scene["tactile"].normal()
    for name in ["wrist", "global_camera"]:
        if name in env.scene.sensors:
            _ = env.scene[name].data
    base = np.array([q0[k] for k in JOINTS])
    rows = []
    durations = []
    peak = np.zeros((12, 32))
    maxlift = 0.0
    low, high = np.deg2rad(LIMITS[:5]).T
    desired = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])

    def ik(target, guess):
        def residual(q):
            t = forward(dict(zip(JOINTS, q)))["gripper_link"]
            point = t @ [-0.0065, 0, -0.08, 1]
            return np.r_[
                (point[:3] - target) * 8,
                Rotation.from_matrix(desired.T @ t[:3, :3]).as_rotvec(),
            ]

        return least_squares(residual, guess, bounds=(low, high), max_nfev=300).x

    grasp = ik([0.22, -0.009, 0.04], [0, 0.5, 1.3, -1.7, 1.6])
    hover = ik([0.23, -0.009, 0.16], grasp)
    retreat = ik([0.22, -0.06, 0.04], grasp)
    retreat_hover = ik([0.22, -0.06, 0.16], retreat)
    waypoints = [
        (0, base),
        (30, base),
        (100, np.r_[hover, 1.1]),
        (155, np.r_[grasp, 1.1]),
        (195, np.r_[grasp, -0.17]),
        (250, np.r_[hover, -0.17]),
        (280, np.r_[hover, -0.17]),
        (335, np.r_[grasp, -0.17]),
        (390, np.r_[grasp, 1.1]),
        (450, np.r_[retreat, 1.1]),
        (510, np.r_[retreat_hover, 1.1]),
    ]

    if a.mode == "sweep":
        # Empty-gripper clearance motion in the demonstrated hover arm posture.
        extended = np.r_[hover, 1.1]
        left = extended.copy()
        left[4] = np.deg2rad(-170)
        left[5] = -0.17
        right = extended.copy()
        right[4] = np.deg2rad(170)
        right[5] = 1.4
        waypoints = [
            (0, base),
            (30, base),
            (110, extended),
            (260, left),
            (285, left),
            (535, right),
            (560, right),
            (650, extended),
        ]

    def command(frame):
        if a.mode == "hold":
            return base
        for (t0, q1), (t1, q2) in zip(waypoints, waypoints[1:]):
            if frame <= t1:
                u = np.clip((frame - t0) / (t1 - t0), 0, 1)
                u = u * u * (3 - 2 * u)
                return q1 * (1 - u) + q2 * u
        return waypoints[-1][1]

    for frame in range(a.steps):
        if a.mode == "grasp" and frame == 280:
            # Use measured grasp pose for placement, avoiding an unreachable fixed
            # orientation that previously left the block above the table.
            current = env.robot.data.joint_pos.torch[0, env.ids].cpu().numpy()
            held = env.block.data.root_link_pose_w.torch[0].cpu().numpy()
            grasp_frame = forward(dict(zip(JOINTS, current)))["gripper_link"]
            point = (grasp_frame @ [-0.0065, 0, -0.08, 1])[:3]
            point[2] -= held[2] - 0.0505

            def placement_ik(point, guess):
                def error(q):
                    t = forward(dict(zip(JOINTS, q)))["gripper_link"]
                    return np.r_[
                        ((t @ [-0.0065, 0, -0.08, 1])[:3] - point) * 50,
                        Rotation.from_matrix(grasp_frame[:3, :3].T @ t[:3, :3]).as_rotvec() * 0.5,
                    ]

                return least_squares(error, guess, bounds=(low, high), max_nfev=300).x

            placed = placement_ik(point, current[:5])
            away_point = point - grasp_frame[:3, 0] * 0.025
            away = placement_ik(away_point, placed)
            up_point = away_point.copy()
            up_point[2] += 0.12
            up = placement_ik(up_point, away)
            waypoints = [
                (280, current),
                (350, np.r_[placed, -0.17]),
                (380, np.r_[placed, -0.17]),
                (440, np.r_[placed, 1.4]),
                (490, np.r_[away, 1.4]),
                (550, np.r_[up, 1.4]),
            ]
        q = command(frame)
        start = time.perf_counter()
        env.step(torch.tensor(q, dtype=torch.float32, device=env.device)[None])
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - start
        tactile = env.scene["tactile"].normal()[0].cpu().numpy()
        peak = np.maximum(peak, tactile)
        pose = env.block.data.root_link_pose_w.torch[0].cpu().tolist()
        maxlift = max(maxlift, pose[2] - 0.05)
        rec = dict(
            frame=frame,
            physics_step=(frame + 1) * 8,
            sim_time_s=(frame + 1) / 30,
            block_pose=pose,
            tactile_peak=float(tactile.max()),
            tactile_time_s=float(wp.to_torch(env.scene["tactile"]._timestamp_last_update)[0]),
        )
        rec["contact_force_norm"] = float(
            torch.linalg.vector_norm(env.scene["contact"].data.net_forces_w.torch[0]).item()
        )
        rec["q"] = env.robot.data.joint_pos.torch[0, env.ids].cpu().tolist()
        rec["target_q"] = q.tolist()
        rec["body_poses"] = dict(
            zip(env.robot.body_names, env.robot.data.body_link_pose_w.torch[0].cpu().tolist())
        )
        if "wrist" in env.scene.sensors:
            rgb = env.scene["wrist"].data.output["rgb"].torch
            rec["rgb_time_s"] = float(wp.to_torch(env.scene["wrist"]._timestamp_last_update)[0])
        if frame >= 30:
            durations.append(elapsed)
        if frame in [29, 154, 194, 249, a.steps - 1] and a.render == "true":
            for name in ["global_camera", "wrist"]:
                if name in env.scene.sensors:
                    pixels = env.scene[name].data.output["rgb"].torch[0, ..., :3].cpu().numpy()
                    Image.fromarray(pixels.astype(np.uint8)).save(
                        a.output_dir / f"{name}_{frame}.png"
                    )
        rows.append(rec)
    np.save(a.output_dir / "tactile_peak.npy", peak)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 3))
    im = ax.imshow(peak, vmin=0, cmap="inferno")
    ax.set(
        xlabel="Column (+Z)",
        ylabel="Row (+Y)",
        title="Single-side FlexiTac peak — uncalibrated normal proxy",
    )
    fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(a.output_dir / "tactile_heatmap.png")
    plt.close(fig)
    report = dict(
        mode=a.mode,
        steps=a.steps,
        seed=a.seed,
        camera=a.camera,
        render=a.render,
        physics_hz=240,
        tactile_hz=60,
        rgb_hz=30,
        max_lift_m=maxlift,
        max_tactile=float(peak.max()),
        mean_control_step_s=float(np.mean(durations)),
        p95_control_step_s=float(np.quantile(durations, 0.95)),
        rows=rows,
        full_grasp_pass=bool(
            a.mode == "grasp"
            and a.steps >= 420
            and min(r["block_pose"][2] for r in rows[250:280]) > 0.09
            and max(r["tactile_peak"] for r in rows[250:280]) > 1e-6
            and abs(rows[-1]["block_pose"][2] - 0.05) < 0.005
            and max(r["tactile_peak"] for r in rows[-5:]) < 1e-6
        ),
    )
    (a.output_dir / "result.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))
    env.reset()
    reset_pose = env.block.data.root_link_pose_w.torch[0].cpu().tolist()
    reset_history_cleared = env.scene["tactile"].history is None
    reset_tactile = env.scene["tactile"].normal().abs().max().item()
    reset_q = env.robot.data.joint_pos.torch[0, env.ids].cpu().numpy()
    reset_result = dict(
        block_pose=reset_pose,
        tactile_max=reset_tactile,
        joint_error_rad=float(np.max(np.abs(reset_q - base))),
        history_cleared=reset_history_cleared,
    )
    reset_result["status"] = (
        "PASS"
        if reset_tactile == 0
        and reset_result["joint_error_rad"] < 1e-5
        and np.linalg.norm(np.array(reset_pose[:3]) - [0.23, 0, 0.05]) < 1e-5
        else "FAIL"
    )
    (a.output_dir / "reset.json").write_text(json.dumps(reset_result, indent=2))
    env.close()
