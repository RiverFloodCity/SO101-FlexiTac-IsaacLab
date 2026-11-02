# SPDX-License-Identifier: Apache-2.0
"""Restore stock M3 features on an explicit local LeFlexiTac CAD COPY.

All booleans use millimetres. Output remains an uncleared LeFlexiTac derivative.
Motor, joint, tactile and moving-jaw geometry are not changed.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

from so101_flexitac.kinematics import description, origin
from so101_flexitac.spec import WRIST_REGISTRATION, asset_dir


def cylinder(radius, low, high, x, z, sections=96):
    m = trimesh.creation.cylinder(radius=radius, height=high - low, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    m.apply_translation([x, (low + high) / 2, z])
    return m


def box(bounds):
    bounds = np.array(bounds)
    m = trimesh.creation.box(bounds[1] - bounds[0])
    m.apply_translation(bounds.mean(axis=0))
    return m


def frames():
    for link in description().findall("link"):
        for visual in link.findall("visual"):
            if "wrist_roll_follower" in visual.find("geometry/mesh").get("filename"):
                t = origin(visual)
                t[:3, 3] *= 1000
    custom = t.copy()
    custom[:3, 3] += t[:3, :3] @ (np.array(WRIST_REGISTRATION) * 1000)
    return t, custom


def restore(wrist, output):
    output.mkdir(parents=True, exist_ok=True)
    target = output / "Wrist_Roll_08c_HexNut_M3_r1.stl"
    if target.exists():
        raise FileExistsError(target)
    data = wrist.read_bytes()
    if (
        hashlib.sha256(data).hexdigest()
        != "7e78b577cd03f5c3e1ec5c2a5c1f0fb71cbaed23e6dd2bf76dc166a3b0b7bdeb"
    ):
        raise ValueError("Unreviewed custom wrist input")
    stock_path = asset_dir() / "assets/official_Wrist_Roll_Follower_SO101_mm.stl"
    stock_t, custom_t = frames()
    stock = trimesh.load_mesh(stock_path)
    stock.apply_transform(stock_t)
    custom = trimesh.load_mesh(wrist)
    custom.apply_transform(custom_t)
    # Dimensions measured from the exact official print STL in gripper-link coordinates.
    z = -23.4001
    cutters = []
    for x in [-5.0, 3.1]:
        cutters.append(cylinder(1.6, 16.9, 24.2, x, z))
        # Six-sided prism, point up in XZ, gives 5.6 mm across the X flats.
        hexagon = cylinder(5.6 / np.sqrt(3), 16.9, 20.5, 0, 0, 6)
        hexagon.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 6, [0, 1, 0]))
        hexagon.apply_translation([x, 0, z])
        cutters.append(hexagon)
    # Restore only the original relief around the bracket locating tongue.
    # This intersects an existing edge recess; it does not cut a new screw seat.
    relief_box = box([[-10, 21.5, -15.5], [-6, 24.2, -12.5]])
    relief = trimesh.boolean.difference([relief_box, stock], engine="manifold")
    cutters.append(relief)
    tool = trimesh.boolean.union(cutters, engine="manifold")
    modified = trimesh.boolean.difference([custom, tool], engine="manifold")
    removed = trimesh.boolean.intersection([custom, tool], engine="manifold")
    if not modified.is_watertight or not modified.is_volume or len(modified.split()) != 1:
        raise ValueError("Modified solid is not a single closed printable volume")
    raw = modified.copy()
    raw.apply_transform(np.linalg.inv(custom_t))
    raw.export(target)
    for name, m in [
        ("stock_aligned_mm", stock),
        ("original_aligned_mm", custom),
        ("modified_aligned_mm", modified),
        ("removed_aligned_mm", removed),
        ("cutters_aligned_mm", tool),
    ]:
        m.export(output / (name + ".stl"))
    (output / "Wrist_Roll_08c_ORIGINAL.stl").write_bytes(data)
    (output / "Wrist_Roll_Follower_SO101_REFERENCE.stl").write_bytes(stock_path.read_bytes())
    report = dict(
        status="CAD_CREATED_NOT_HARDWARE_VALIDATED",
        frame="gripper_link, mm",
        centers_xz_mm=[[-5, z], [3.1, z]],
        hole_diameter_mm=3.2,
        hex_across_flats_mm=5.6,
        hex_depth_mm=3.3,
        hex_y_interval_mm=[17.2, 20.5],
        remaining_bore_land_mm=3.5,
        center_distance_mm=8.1,
        between_hex_flats_mm=2.5,
        custom_to_gripper_mm=custom_t.tolist(),
        stock_to_gripper_mm=stock_t.tolist(),
        removed_mm3=float(removed.volume),
        original_mm3=float(custom.volume),
        modified_mm3=float(modified.volume),
        closed=modified.is_watertight,
        one_component=len(modified.split()) == 1,
        source_sha256=hashlib.sha256(data).hexdigest(),
        output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        license="LeFlexiTac derivative; redistribution UNCONFIRMED",
        strength="Geometric assessment only, no material test or FEA",
    )
    (output / "revision.json").write_text(json.dumps(report, indent=2))
    # An editable OpenSCAD CSG description, in the original custom STL coordinates.
    scad = "// LOCAL ONLY: derived LeFlexiTac geometry, redistribution unconfirmed.\n$fn=96;\n"
    scad += "module aligned_cuts(){\nfor(x=[-5,3.1]){\n"
    scad += "translate([x,24.2,-23.4001]) rotate([90,0,0]) cylinder(h=7.3,r=1.6);\n"
    scad += "translate([x,20.5,-23.4001]) rotate([90,0,0]) rotate([0,0,30]) cylinder(h=3.6,r=5.6/sqrt(3),$fn=6);\n}\n"
    scad += (
        "difference(){translate([-10,21.5,-15.5]) cube([4,2.7,3]);multmatrix("
        + json.dumps(stock_t.tolist())
        + ') import("Wrist_Roll_Follower_SO101_REFERENCE.stl");}\n}\n'
    )
    scad += (
        'difference(){import("Wrist_Roll_08c_ORIGINAL.stl");multmatrix('
        + json.dumps(np.linalg.inv(custom_t).tolist())
        + ") aligned_cuts();}\n"
    )
    (output / "restore_hex_mount.scad").write_text(scad)
    print(json.dumps(report))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--wrist", type=Path, required=True)
    p.add_argument("--output_dir", type=Path, required=True)
    a = p.parse_args()
    restore(a.wrist, a.output_dir)
