# SPDX-License-Identifier: Apache-2.0
"""Engineering parameters; provisional, not a hardware calibration."""

from pathlib import Path

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
PAD = dict(
    rows=12,
    cols=32,
    pitch=0.002,
    subsamples=3,
    width=0.026,
    length=0.064,
    face_x=-0.0065,
    center_z=-0.080,
    thickness=0.0005,
    adhesive=0.0002,
    spacer=0.0007,
    kn=1.0,
    kd=0.003,
    physics_kn=10000.0,
    physics_kd=10.0,
    update_hz=60,
    mass=0.003,
    static_friction=1.0,
    dynamic_friction=0.8,
)
CAMERA = dict(
    width=1280,
    height=960,
    fx=508.7,
    fy=508.7,
    cx=640.0,
    cy=480.0,
    distortion_model="plumb_bob",
    distortion=[0.0, 0.0, 0.0, 0.0, 0.0],
    hz=30,
    status="community_FOV_placeholder_not_measured",
    mass=0.015,
    mount_mass=0.020,
)
BASE_TRANSLATION = (-0.0211353, 0.0, 0.0024)
WRIST_REGISTRATION = (0.0, -0.000218214, 0.000949706)
MOUNT_XYZ = (-0.015, 0.024, -0.0315)
MOUNT_RPY = (-1.5708, -0.0008, -1.5708)
BODY_XYZ = (-0.04941, -0.03813, 0.0175)
BODY_RPY = (0.0, -1.5708, -2.0076)
FRAME_XYZ = (0.0025, 0.0675, -0.0062)
FRAME_RPY = (3.141593, 1.136305, -1.570797)
OPTICAL_RPY = (-1.5708, 0.0, -1.5708)
LIMITS = (
    (-90, 90),
    (-105.2967032967033, 90),
    (-90, 98.96703296703296),
    (-102.04395604395604, 90),
    (-170, 170),
    (-9.7402825, 80.2140913),
)


def asset_dir():
    return Path(__file__).resolve().parent / "assets"
