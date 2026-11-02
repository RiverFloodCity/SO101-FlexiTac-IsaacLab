# SPDX-License-Identifier: Apache-2.0
"""Check added camera bodies against actual cooked collision solids at recorded poses."""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument("--output_dir", type=Path, required=True)
a = p.parse_args()
r = a.output_dir
d = json.loads((r / "result.json").read_text())
rows = d["rows"]
shapes = {}
for item in json.loads((r / "cooked_shapes/index.json").read_text()):
    if item["group"] in ["Table", "Block"]:
        continue
    arr = np.load(r / "cooked_shapes" / item["file"])
    shapes.setdefault(item["group"], []).append(
        trimesh.Trimesh(arr["vertices"], arr["faces"], process=False)
    )


def matrix(p):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    t[:3, 3] = p[:3]
    return t


def overlaps(a, b):
    return np.all(np.minimum(a[1], b[1]) - np.maximum(a[0], b[0]) > 0)


allowed = {frozenset(["CameraMount", "gripper_link"]), frozenset(["CameraBody", "CameraMount"])}
pairs = sorted(
    {
        tuple(sorted([left, r]))
        for left in ["CameraMount", "CameraBody"]
        for r in shapes
        if left != r and frozenset([left, r]) not in allowed
    }
)
peaks = {" / ".join(pair): 0.0 for pair in pairs}
failures = []
drift = []
for row in rows:
    world = {}
    bounds = {}
    for name, meshes in shapes.items():
        t = matrix(row["body_poses"][name])
        out = []
        for m in meshes:
            n = m.copy()
            n.apply_transform(t)
            out.append(n)
        world[name] = out
        bounds[name] = np.array(
            [np.min([x.bounds[0] for x in out], axis=0), np.max([x.bounds[1] for x in out], axis=0)]
        )
    for left, h in pairs:
        if not overlaps(bounds[left], bounds[h]):
            continue
        v = 0.0
        for x in world[left]:
            for y in world[h]:
                if overlaps(x.bounds, y.bounds):
                    v += abs(trimesh.boolean.intersection([x, y], engine="manifold").volume)
        key = " / ".join([left, h])
        peaks[key] = max(peaks[key], v * 1e9)
        if v > 1e-10:
            failures.append(dict(frame=row["frame"], pair=key, volume_mm3=v * 1e9))
    relative = np.linalg.inv(matrix(row["body_poses"]["gripper_link"])) @ matrix(
        row["body_poses"]["CameraBody"]
    )
    drift.append(
        [np.linalg.norm(relative[:3, 3]), Rotation.from_matrix(relative[:3, :3]).magnitude()]
    )
q = np.array([x["q"] for x in rows])
target = np.array([x["target_q"] for x in rows])
report = dict(
    status="PASS" if not failures else "FAIL",
    checked_frames=len(rows),
    sample_hz=30,
    collision_tolerance_mm3=0.1,
    pair_peak_mm3=peaks,
    failures=failures,
    allowed_fixed_mates=[sorted(x) for x in allowed],
    scope="Actual cooked camera/mount solids against other robot links at every recorded control frame. No claim of continuous-time or full 6-DOF workspace certification.",
    joint_min_deg=np.rad2deg(q.min(axis=0)).tolist(),
    joint_max_deg=np.rad2deg(q.max(axis=0)).tolist(),
    max_camera_to_gripper_translation_m=float(np.max(np.array(drift)[:, 0])),
    max_camera_to_gripper_rotation_rad=float(np.max(np.array(drift)[:, 1])),
    final_joint_target_error_deg=np.rad2deg(np.abs(q[-1] - target[-1])).tolist(),
    rgb_tactile_max_time_difference_s=max(abs(x["rgb_time_s"] - x["tactile_time_s"]) for x in rows),
)
(r / "motion_review.json").write_text(json.dumps(report, indent=2))
print(json.dumps({k: v for k, v in report.items() if k != "failures"}))
