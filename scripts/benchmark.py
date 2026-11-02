# SPDX-License-Identifier: Apache-2.0
"""Sequential paired one-environment benchmark; no concurrent GPU workloads."""

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--robot_camera", type=Path, required=True)
p.add_argument("--robot_baseline", type=Path, required=True)
p.add_argument("--rest", type=Path, required=True)
p.add_argument("--output_dir", type=Path, required=True)
p.add_argument("--seed", type=int, default=42)
p.add_argument("--device", default="cuda:0")
p.add_argument("--repeats", type=int, default=3)
a = p.parse_args()
a.output_dir.mkdir(parents=True, exist_ok=True)
runs = []
for repeat in range(a.repeats):
    for label, robot, enabled in [
        ("baseline", a.robot_baseline, "false"),
        ("camera", a.robot_camera, "true"),
    ]:
        out = a.output_dir / f"{label}_{repeat}"
        command = [
            sys.executable,
            str(Path(__file__).with_name("demo.py")),
            "--robot",
            str(robot),
            "--rest",
            str(a.rest),
            "--output_dir",
            str(out),
            "--mode",
            "hold",
            "--steps",
            "150",
            "--camera",
            enabled,
            "--render",
            "true",
            "--seed",
            str(a.seed),
            "--device",
            a.device,
            "--viz",
            "none",
        ]
        with (a.output_dir / f"{label}_{repeat}.log").open("w") as log:
            subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, check=True, env=os.environ.copy()
            )
        d = json.loads((out / "result.json").read_text())
        runs.append(
            dict(
                label=label,
                repeat=repeat,
                mean_s=d["mean_control_step_s"],
                p95_s=d["p95_control_step_s"],
            )
        )
        print(json.dumps(runs[-1]), flush=True)
summary = {
    label: dict(
        median_control_step_s=statistics.median(v["mean_s"] for v in runs if v["label"] == label),
        run_means_s=[v["mean_s"] for v in runs if v["label"] == label],
    )
    for label in ["baseline", "camera"]
}
b = summary["baseline"]["median_control_step_s"]
c = summary["camera"]["median_control_step_s"]
report = dict(
    conditions="single environment, sequential paired runs; 150 control steps each, first 30 warmup; 240 Hz PhysX, 60 Hz 3456-sample tactile, 30 Hz global 960x720 RTX, optional 640x480 wrist RTX",
    summary=summary,
    relative_change_percent=100 * (c / b - 1),
    baseline_rtf=(1 / 30) / b,
    camera_rtf=(1 / 30) / c,
    runs=runs,
    scope="step wall time with CUDA synchronization; excludes readback/plotting outside env.step; not multi-environment throughput",
)
(a.output_dir / "performance.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
