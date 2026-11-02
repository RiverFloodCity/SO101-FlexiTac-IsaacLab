# SPDX-License-Identifier: Apache-2.0
"""Resolve a locally verified platform without bundling restricted derived CAD."""
import hashlib
import json
import os
from pathlib import Path


def resolve_local_platform(manifest=None):
    manifest = Path(manifest or os.environ.get("SO101_FLEXITAC_PLATFORM_MANIFEST")
                    or Path(__file__).resolve().parents[2] / ".local/platform.json").resolve()
    data = json.loads(manifest.read_text())
    if data.get("variant") != "Wrist_Roll_08c_HexNut_M3_r1":
        raise ValueError("Only the explicitly verified r1 platform is registered")
    resolved = dict(data)
    for key in ("robot", "verification", "cad"):
        path = (manifest.parent / data[key]["path"]).resolve()
        if hashlib.sha256(path.read_bytes()).hexdigest() != data[key]["sha256"]:
            raise ValueError("Local platform artifact changed: " + key)
        resolved[key+"_path"] = str(path)
    review = json.loads(Path(resolved["verification_path"]).read_text())
    if review.get("overall") != "LOCAL_GEOMETRY_AND_SIMULATION_PASS_HARDWARE_UNVERIFIED":
        raise ValueError("Local numerical verification is missing")
    if review["cad"]["status"] != "LOCAL_MOUNT_GEOMETRY_PASS" or not review["grasp"]["full_grasp_pass"]:
        raise ValueError("CAD/grasp verification failed")
    # Historical reproduction reports are valid only for the exact reviewed artifacts.
    if "reviewed_artifacts" in review:
        for key in ("cad", "robot"):
            if review["reviewed_artifacts"].get(key) != data[key]["sha256"]:
                raise ValueError("Artifact does not match historical review: " + key)
    resolved["manifest_path"] = str(manifest)
    resolved["manifest_sha256"] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    return resolved
