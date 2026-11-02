# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E402
"""Numerical CAD comparison, assembly checks and dimensioned review figures."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon

from so101_flexitac.geometry import camera_parts, hex_mount_fasteners, parts
from so101_flexitac.kinematics import forward

p = argparse.ArgumentParser()
p.add_argument("--revision", type=Path, required=True)
p.add_argument("--local_assets", type=Path, required=True)
a = p.parse_args()
r = a.revision
stock = trimesh.load_mesh(r / "stock_aligned_mm.stl")
original = trimesh.load_mesh(r / "original_aligned_mm.stl")
modified = trimesh.load_mesh(r / "modified_aligned_mm.stl")
removed = trimesh.load_mesh(r / "removed_aligned_mm.stl")
links = parts(a.local_assets, wrist_override=r / "Wrist_Roll_08c_HexNut_M3_r1.stl")
servo = links["gripper_link"][0][1].copy()
servo.apply_scale(1000)
mount, camera = camera_parts()
mount.apply_scale(1000)
camera.apply_scale(1000)
fasteners = []
for name, m, mass in hex_mount_fasteners():
    m = m.copy()
    m.apply_scale(1000)
    fasteners.append((name, m, mass))


def volume(x, y):
    if np.any(np.minimum(x.bounds[1], y.bounds[1]) - np.maximum(x.bounds[0], y.bounds[0]) <= 0):
        return 0.0
    return float(abs(trimesh.boolean.intersection([x, y], engine="manifold").volume))


def swept(m, delta):
    return trimesh.convex.convex_hull(np.vstack([m.vertices, m.vertices + delta]))


checks = {
    "modified_vs_mount_mm3": volume(modified, mount),
    "camera_hull_vs_modified_mm3": volume(camera.convex_hull, modified),
    "removed_vs_servo_mm3": volume(removed, servo),
    "original_vs_servo_mm3": volume(original, servo),
    "modified_vs_servo_mm3": volume(modified, servo),
}
fastener_checks = []
for name, m, _ in fasteners:
    row = dict(
        name=name,
        wrist_mm3=volume(m, modified),
        servo_mm3=volume(m, servo),
        bracket_mm3=volume(m, mount),
    )
    if name.startswith("Nut"):
        # Remove motor 6 with moving jaw first, as specified by upstream assembly guide.
        # Continuous convex swept volumes: enter empty cavity from +X, then seat along +Y.
        staged = m.copy()
        staged.apply_translation([0, -18.1, 0])
        x = -5 if name.endswith("1") else 3.1
        row["entry_sweep_mm3"] = volume(swept(staged, [40 - x, 0, 0]), modified)
        row["seating_sweep_mm3"] = volume(swept(staged, [0, 18.1, 0]), modified)
    fastener_checks.append(row)
wall_rows = []
for label, m in [("stock", stock), ("modified", modified)]:
    for y in np.linspace(17.201, 20.499, 101):
        ps = [Polygon(curve[:, [0, 2]]) for curve in m.section([0, 1, 0], [0, y, 0]).discrete]
        for x in [-5, 3.1]:
            hole = min(
                (p for p in ps if p.area < 60),
                key=lambda p: np.linalg.norm(np.array(p.centroid.coords[0]) - [x, -23.4]),
            )
            distance = min(hole.boundary.distance(p.boundary) for p in ps if p is not hole)
            wall_rows.append(dict(model=label, y_mm=float(y), x_mm=x, ligament_mm=float(distance)))
wall = {
    label: min((x for x in wall_rows if x["model"] == label), key=lambda x: x["ligament_mm"])
    for label in ["stock", "modified"]
}
jaw_checks = []
for angle in np.linspace(-0.17, 1.4, 101):
    fk = forward({"gripper": float(angle)})
    t = np.linalg.inv(fk["gripper_link"]) @ fk["moving_jaw_so101_v1_link"]
    jaw = links["moving_jaw_so101_v1_link"][0][1].copy()
    jaw.apply_transform(t)
    jaw.apply_scale(1000)
    for label, m in [("mount", mount), ("camera_hull", camera.convex_hull)] + [
        (n, m) for n, m, _ in fasteners
    ]:
        v = volume(m, jaw)
        if v > 1e-5:
            jaw_checks.append(dict(angle=float(angle), part=label, intersection_mm3=v))
report = dict(
    status="LOCAL_MOUNT_GEOMETRY_PASS"
    if checks["modified_vs_mount_mm3"] < 1e-5
    and checks["removed_vs_servo_mm3"] < 1e-5
    and not jaw_checks
    and all(all(v < 1e-5 for k, v in row.items() if k.endswith("mm3")) for row in fastener_checks)
    else "FAIL",
    checks=checks,
    fasteners=fastener_checks,
    wall=wall,
    wall_scope="101 planes through nut pocket; shortest in-plane distance to another contour, not a global 3D wall-thickness or strength certificate",
    jaw_sample_count=101,
    jaw_collisions=jaw_checks,
    mesh={
        "watertight": modified.is_watertight,
        "winding_consistent": modified.is_winding_consistent,
        "positive_volume": bool(modified.volume > 0),
        "components": len(modified.split()),
        "degenerate_faces": int((modified.area_faces < 1e-12).sum()),
    },
    removed_mm3=removed.volume,
    removed_fraction=removed.volume / original.volume,
    removed_bounds_mm=removed.bounds.tolist(),
    pad_z_interval_mm=[-112, -48],
    unchanged_note="Subtraction only, cuts are remote from tactile face and servo solid; inherited nominal CAD servo/wrist overlap is unchanged, not newly introduced",
    nut=dict(
        across_flats_mm=5.5,
        height_mm=2.4,
        pocket_af_mm=5.6,
        clearance_per_flat_mm=0.05,
        seat_y_mm=[18.1, 20.5],
        servo_gap_mm=1.0,
    ),
    screw=dict(
        thread="M3",
        length_mm=8,
        head_diameter_mm=4.5,
        head_height_mm=3,
        reference="NBK SNS-M3-8-SD",
        under_head_y_mm=26,
        tip_y_mm=18,
        servo_tip_gap_mm=0.9,
        nut_full_height_engagement_mm=2.4,
        bracket_head_recess_diameter_mm=5.4,
        note="Generic 5.5 mm M3 cap heads are not guaranteed to fit the unchanged official bracket",
    ),
    hardware_validation="NOT_PERFORMED",
    strength="NO_FEA_NO_LOAD_TEST",
    cable="Existing channels untouched; camera cable route/strain relief and moving cable envelope not modeled; inspect on hardware",
)
(r / "geometry_review.json").write_text(json.dumps(report, indent=2))

colors = {"stock": "#2563a6", "original": "#e5a03a", "modified": "#16846f", "removed": "#d94343"}


def section(ax, m, axis, value, color, label, indices):
    n = np.eye(3)[axis]
    pt = n * value
    s = m.section(n, pt)
    if s:
        for i, line in enumerate(s.discrete):
            ax.plot(
                line[:, indices[0]],
                line[:, indices[1]],
                color=color,
                lw=1.35,
                label=label if i == 0 else None,
            )


fig, axs = plt.subplots(1, 3, figsize=(15, 5.6))
for ax, label, m in zip(
    axs, ["Official SO101", "LeFlexiTac original", "Restored copy"], [stock, original, modified]
):
    section(
        ax,
        m,
        1,
        18.5,
        colors["modified"]
        if label == "Restored copy"
        else colors["stock"]
        if label == "Official SO101"
        else colors["original"],
        label,
        [0, 2],
    )
    if label == "Restored copy":
        section(ax, original, 1, 18.5, colors["original"], "Original overlay", [0, 2])
    ax.set(
        xlim=(-12, 12),
        ylim=(-33, -14),
        aspect="equal",
        xlabel="X (mm)",
        ylabel="Z (mm)",
        title=label + " | Y=18.5 mm",
    )
    for x in [-5, 3.1]:
        ax.plot(x, -23.4001, "+", color="black", ms=7)
    ax.grid(alpha=0.2)
axs[2].annotate(
    "8.10 mm", xy=(-5, -18.5), xytext=(3.1, -18.5), ha="center", arrowprops=dict(arrowstyle="<->")
)
axs[2].text(
    -10,
    -30.5,
    "Bore Ø3.20 mm; hex AF 5.60 mm / depth 3.30 mm\nCenters: (-5,-23.4001), (3.1,-23.4001)",
    fontsize=8,
)
fig.suptitle("Aligned actual STL sections — missing features restored locally", fontsize=15)
fig.tight_layout()
fig.savefig(r / "geometry_comparison.png", dpi=170)
plt.close(fig)
fig, axs = plt.subplots(1, 2, figsize=(12, 6))
for ax, x in zip(axs, [-5, 3.1]):
    for m, col, label in [
        (original, colors["original"], "Original wrist"),
        (modified, colors["modified"], "Modified wrist"),
        (servo, "#555555", "Motor 6"),
        (mount, "#6576bb", "Official bracket"),
    ]:
        section(ax, m, 0, x, col, label, [1, 2])
    for name, m, _ in fasteners:
        if (x == -5 and name.endswith("1")) or (x == 3.1 and name.endswith("2")):
            section(ax, m, 0, x, "#111111", name, [1, 2])
    ax.set(
        xlim=(14, 31),
        ylim=(-29, -18),
        aspect="equal",
        xlabel="Y (mm)",
        ylabel="Z (mm)",
        title=f"Axial section X={x:.1f} mm",
    )
    ax.annotate(
        "3.3 pocket",
        xy=(17.2, -27.5),
        xytext=(20.5, -27.5),
        fontsize=8,
        ha="center",
        arrowprops=dict(arrowstyle="<->"),
    )
    ax.annotate(
        "3.5 land",
        xy=(20.5, -28.2),
        xytext=(24, -28.2),
        fontsize=8,
        ha="center",
        arrowprops=dict(arrowstyle="<->"),
    )
    ax.text(14.3, -18.5, "M3 x 8, head Ø4.5; full nut engagement 2.4 mm", fontsize=8)
    ax.grid(alpha=0.2)
axs[0].legend(fontsize=7, loc="lower right")
fig.suptitle("Bolt / nut / servo clearance — nominal CAD, dimensions in mm")
fig.tight_layout()
fig.savefig(r / "local_sections.png", dpi=180)
plt.close(fig)


# Explicit sections at each model's measured minimum pocket ligament.
from shapely.ops import nearest_points

fig, axs = plt.subplots(1, 2, figsize=(11, 6))
for ax, label, model in zip(axs, ["stock", "modified"], [stock, modified]):
    y = wall[label]["y_mm"]
    curves = model.section([0, 1, 0], [0, y, 0]).discrete
    polygons = [Polygon(curve[:, [0, 2]]) for curve in curves]
    hole = min(
        (poly for poly in polygons if poly.area < 60),
        key=lambda poly: np.linalg.norm(np.array(poly.centroid.coords[0]) - [3.1, -23.4]),
    )
    other = min(
        (poly for poly in polygons if poly is not hole),
        key=lambda poly: hole.boundary.distance(poly.boundary),
    )
    point_a, point_b = nearest_points(hole.boundary, other.boundary)
    first = np.array(point_a.coords[0])
    second = np.array(point_b.coords[0])
    section(ax, model, 1, y, colors[label], label, [0, 2])
    ax.plot([first[0], second[0]], [first[1], second[1]], color="#cc3333", lw=3)
    ax.annotate(
        f"{wall[label]['ligament_mm']:.3f} mm minimum",
        xy=(first + second) / 2,
        xytext=(0, -29),
        arrowprops=dict(arrowstyle="->", color="#cc3333"),
        color="#aa2222",
    )
    ax.set(
        xlim=(-1, 13),
        ylim=(-31, -16),
        aspect="equal",
        xlabel="X (mm)",
        ylabel="Z (mm)",
        title=f"{label.title()} | Y={y:.3f} mm",
    )
    ax.grid(alpha=0.2)
fig.suptitle("Remaining pocket ligament — minimum over 101 sections; no strength certification")
fig.tight_layout()
fig.savefig(r / "wall_sections.png", dpi=180)
plt.close(fig)


def surface(ax, m, color, alpha=1):
    faces = m.triangles
    poly = Poly3DCollection(faces, facecolor=color, edgecolor="none", alpha=alpha, rasterized=True)
    ax.add_collection3d(poly)


def frame(ax, lo, hi, elev, azim):
    ax.set(
        xlim=(lo[0], hi[0]),
        ylim=(lo[1], hi[1]),
        zlim=(lo[2], hi[2]),
        xlabel="X (mm)",
        ylabel="Y (mm)",
        zlabel="Z (mm)",
    )
    ax.set_box_aspect(np.array(hi) - lo)
    ax.view_init(elev=elev, azim=azim)


fig = plt.figure(figsize=(13, 6))
for i, (title, m) in enumerate(
    [("Original / modified overlay", original), ("Removed material only (red)", modified)]
):
    ax = fig.add_subplot(1, 2, i + 1, projection="3d", computed_zorder=False)
    surface(ax, m, "#b8bec5", 0.10)
    if i == 0:
        surface(ax, modified, "#16846f", 0.08)
    surface(ax, removed, "#d94343", 1)
    frame(ax, [-36, -25, -119], [31, 26, 2], 20, 55)
    ax.set_title(title)
fig.suptitle("Local subtraction: 239.206 mm³ (0.3534%); no reinforcement or joint changes")
fig.tight_layout()
fig.savefig(r / "overlay_3d.png", dpi=160)
plt.close(fig)
fig = plt.figure(figsize=(13, 7))
for i, exploded in enumerate([False, True]):
    ax = fig.add_subplot(1, 2, i + 1, projection="3d")
    surface(ax, modified, "#d9ad46", 1)
    if not exploded:
        surface(ax, servo, "#46494d", 1)
    for name, m, mass in fasteners:
        surface(ax, m, "#a4aab3", 1)
    shift = np.array([0, 28, 0]) if exploded else np.zeros(3)
    for m, col in [(mount, "#4c77aa"), (camera, "#343a40")]:
        c = m.copy()
        c.apply_translation(shift)
        surface(ax, c, col, 1)
    jaw = links["moving_jaw_so101_v1_link"][0][1].copy()
    fk = forward({"gripper": 0.5})
    jaw.apply_transform(np.linalg.inv(fk["gripper_link"]) @ fk["moving_jaw_so101_v1_link"])
    jaw.apply_scale(1000)
    if not exploded:
        surface(ax, jaw, "#dfba65")
    frame(ax, [-38, -30, -120], [38, 122 if exploded else 94, 22], 20, 135)
    ax.set_title(
        "Assembled: official bracket, camera and fasteners"
        if not exploded
        else "Review only: bracket/camera translated +28 mm Y"
    )
fig.suptitle(
    "Virtual assembly — original Moving Jaw and tactile region retained; hardware untested"
)
fig.tight_layout()
fig.savefig(r / "assembly.png", dpi=160)
plt.close(fig)
# Geometric slicing check; not machine-specific G-code or physical print approval.
raw = trimesh.load_mesh(r / "Wrist_Roll_08c_HexNut_M3_r1.stl")
zs = np.arange(raw.bounds[0, 2] + 0.1, raw.bounds[1, 2], 0.2)
bad = []
for z in zs:
    sec = raw.section([0, 0, 1], [0, 0, z])
    if sec is None or any(np.linalg.norm(c[0] - c[-1]) > 1e-6 for c in sec.discrete):
        bad.append(float(z))
(r / "printability.json").write_text(
    json.dumps(
        dict(
            status="PASS" if not bad else "FAIL",
            layer_height_mm=0.2,
            layers=len(zs),
            open_or_missing_layers_mm=bad,
            hardware_print_test="NOT_PERFORMED",
            note="Supports and printer-specific fit compensation required; not a strength certificate",
        ),
        indent=2,
    )
)
print(
    json.dumps(
        {
            k: report[k]
            for k in ["status", "checks", "wall", "mesh", "removed_mm3", "jaw_collisions"]
        }
    )
)
