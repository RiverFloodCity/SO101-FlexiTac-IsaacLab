# SPDX-License-Identifier: Apache-2.0
"""CAD loading independent of USD or Isaac Lab; never alters input mechanics."""

from pathlib import Path

import trimesh

from .kinematics import description, origin, transform
from .spec import (
    BODY_RPY,
    BODY_XYZ,
    FRAME_RPY,
    FRAME_XYZ,
    MOUNT_RPY,
    MOUNT_XYZ,
    OPTICAL_RPY,
    WRIST_REGISTRATION,
    asset_dir,
)


def parts(local_assets, wrist_override=None):
    result = {}
    replacements = {
        "wrist_roll_follower_so101_v1.stl": ("Wrist_Roll_08c.stl", WRIST_REGISTRATION),
        "moving_jaw_so101_v1.stl": ("Moving_Jaw_08d.stl", (0, 0, 0)),
    }
    for link in description().findall("link"):
        out = []
        for v in link.findall("visual"):
            path = v.find("geometry/mesh").get("filename")
            name = Path(path).name
            m = trimesh.load_mesh(asset_dir() / path)
            if name in replacements:
                name, offset = replacements[name]
                m = trimesh.load_mesh(
                    Path(wrist_override)
                    if name == "Wrist_Roll_08c.stl" and wrist_override
                    else Path(local_assets) / name
                )
                m.apply_scale(0.001)
                m.apply_translation(offset)
            m.apply_transform(origin(v))
            material = v.find("material").get("name")
            out.append((name, m, material))
        result[link.get("name")] = out
    return result


def camera_parts():
    mount = trimesh.load_mesh(asset_dir() / "assets/wrist_camera_mount_so101_v1.stl")
    mount.apply_scale(0.001)
    mount.apply_transform(transform(MOUNT_XYZ, MOUNT_RPY))
    body = trimesh.load_mesh(asset_dir() / "assets/wrist_camera_so101_v1.stl")
    body.apply_transform(transform(MOUNT_XYZ, MOUNT_RPY) @ transform(BODY_XYZ, BODY_RPY))
    return mount, body


def optical_transform():
    return transform(FRAME_XYZ, FRAME_RPY) @ transform(rpy=OPTICAL_RPY)


def overlap_volume(a, b):
    # Actual closed triangle solids, not an AABB admission test.
    if not a.is_watertight or not b.is_watertight:
        raise ValueError("Cannot certify an open collision solid")
    return abs(trimesh.boolean.intersection([a, b], engine="manifold").volume)


def hex_mount_fasteners():
    """Nominal M3 small-head 8 mm screws and DIN934 nuts, in gripper frame.

    Manufacturer reference dimensions: NBK SNS-M3-8-SD and ETTINGER M3 nut.
    Smooth thread envelope only; no preload or helical thread simulation.
    """
    import numpy as np

    def cylinder(radius, lo, hi, sections=96):
        m = trimesh.creation.cylinder(radius=radius, height=hi - lo, sections=sections)
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        m.apply_translation([0, (lo + hi) / 2, 0])
        return m

    result = []
    for i, x in enumerate([-5.0, 3.1]):
        head = cylinder(2.25, 26, 29)
        shaft = cylinder(1.5, 18, 26)
        screw = trimesh.boolean.union([head, shaft], engine="manifold")
        nut = cylinder(5.5 / np.sqrt(3), 18.1, 20.5, 6)
        nut.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 6, [0, 1, 0]))
        nut = trimesh.boolean.difference([nut, cylinder(1.55, 18, 20.6)], engine="manifold")
        for kind, m, mass in [("Screw", screw, 0.00059), ("Nut", nut, 0.00032)]:
            m.apply_translation([x, 0, -23.4001])
            if kind == "Screw":
                # Seat on the actual community bracket's rotated counterbore plane.
                # Its nominal bore center is 0.05 mm above the wrist-hole center.
                m.apply_translation([0, 0, 0.0501])
                reference = transform(np.array(MOUNT_XYZ) * 1000, [-np.pi / 2, 0, -np.pi / 2])
                actual = transform(np.array(MOUNT_XYZ) * 1000, MOUNT_RPY)
                m.apply_transform(actual @ np.linalg.inv(reference))
            m.apply_scale(0.001)
            result.append((f"{kind}{i + 1}", m, mass))
    return result
