# SPDX-License-Identifier: Apache-2.0
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .spec import BASE_TRANSLATION, asset_dir


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    m[:3, 3] = xyz
    return m


def origin(element):
    n = element.find("origin")
    return (
        transform()
        if n is None
        else transform(
            [float(v) for v in n.get("xyz", "0 0 0").split()],
            [float(v) for v in n.get("rpy", "0 0 0").split()],
        )
    )


def description():
    return ET.parse(asset_dir() / "so101.urdf").getroot()


def forward(q):
    poses = {"base_link": transform(BASE_TRANSLATION)}
    pending = list(description().findall("joint"))
    while pending:
        progress = False
        for j in pending[:]:
            parent = j.find("parent").get("link")
            child = j.find("child").get("link")
            if parent not in poses:
                continue
            rot = np.eye(4)
            if j.get("type") != "fixed":
                axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
                rot[:3, :3] = Rotation.from_rotvec(axis * q.get(j.get("name"), 0.0)).as_matrix()
            poses[child] = poses[parent] @ origin(j) @ rot
            pending.remove(j)
            progress = True
        if not progress:
            raise ValueError("Unresolved URDF link graph")
    return poses
