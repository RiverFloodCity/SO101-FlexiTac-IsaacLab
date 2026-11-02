# SPDX-License-Identifier: Apache-2.0
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

from so101_flexitac.geometry import camera_parts, overlap_volume, parts
from so101_flexitac.kinematics import description, forward, origin
from so101_flexitac.spec import asset_dir

p = argparse.ArgumentParser()
p.add_argument("--local_assets", type=Path, required=True)
p.add_argument("--output_dir", type=Path, required=True)
a = p.parse_args()
a.output_dir.mkdir(parents=True, exist_ok=True)
links = parts(a.local_assets)
mount, cam = camera_parts()
cam = cam.convex_hull
records = []
for label, mesh in [("mount", mount), ("camera", cam)]:
    for name, other, _ in links["gripper_link"]:
        try:
            v = overlap_volume(mesh, other)
            error = None
        except Exception as e:
            v = None
            error = str(e)
        records.append(
            dict(
                pair=[label, name],
                volume_mm3=None if v is None else v * 1e9,
                error=error,
            )
        )
    for angle in np.linspace(-0.17, 1.4, 33):
        fk = forward({"gripper": float(angle)})
        for name, other, _ in links["moving_jaw_so101_v1_link"]:
            moved = other.copy()
            moved.apply_transform(
                np.linalg.inv(fk["gripper_link"]) @ fk["moving_jaw_so101_v1_link"]
            )
            records.append(
                dict(
                    pair=[label, name],
                    gripper_rad=float(angle),
                    volume_mm3=overlap_volume(mesh, moved) * 1e9,
                )
            )
# Verify the fastener passages, not merely external envelopes.
wrist = next(m for n, m, _ in links["gripper_link"] if n == "Wrist_Roll_08c.stl")
for link in description().findall("link"):
    for visual in link.findall("visual"):
        filename = visual.find("geometry/mesh").get("filename")
        if "wrist_roll_follower" in filename:
            stock = trimesh.load_mesh(asset_dir() / filename)
            stock.apply_transform(origin(visual))
passages = []
for x in [-0.005, 0.0031]:
    probe = trimesh.creation.cylinder(radius=0.0015, height=0.006, sections=64)
    probe.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    probe.apply_translation([x, 0.021, -0.0234])
    passages.append(
        dict(
            axis_origin_m=[x, 0.021, -0.0234],
            axis=[0, 1, 0],
            radius_m=0.0015,
            length_m=0.006,
            stock_obstructed_fraction=overlap_volume(probe, stock) / probe.volume,
            custom_obstructed_fraction=overlap_volume(probe, wrist) / probe.volume,
        )
    )
failed = any(v["custom_obstructed_fraction"] > 0.001 for v in passages)
report = dict(
    status="FAIL" if failed else "UNVERIFIED",
    method="closed triangle solid boolean (manifold), 33 jaw angles; not a continuous sweep proof",
    records=records,
    bolt_passage_checks=passages,
    failure="Both intended M3 passages are solid in local Wrist_Roll_08c" if failed else None,
    note="This checker evaluates the original 08c only. For the separately authorized modified copy use review_hex_revision.py. No result here is hardware installation approval.",
)
(a.output_dir / "mount_geometry.json").write_text(json.dumps(report, indent=2))
print(
    json.dumps(
        {
            "status": report["status"],
            "bolt_passages": passages,
            "report": str(a.output_dir / "mount_geometry.json"),
        }
    )
)
