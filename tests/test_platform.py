# SPDX-License-Identifier: Apache-2.0
import hashlib
import json
import pytest
from so101_flexitac.platform import resolve_local_platform


def test_verified_local_manifest_and_tamper_rejection(tmp_path):
    review={"overall":"LOCAL_GEOMETRY_AND_SIMULATION_PASS_HARDWARE_UNVERIFIED",
            "cad":{"status":"LOCAL_MOUNT_GEOMETRY_PASS"},"grasp":{"full_grasp_pass":True}}
    files={"robot":b"local robot fixture", "cad":b"local CAD fixture", "verification":json.dumps(review).encode()}
    manifest={"variant":"Wrist_Roll_08c_HexNut_M3_r1"}
    for name,data in files.items():
        (tmp_path/name).write_bytes(data)
        manifest[name]={"path":name,"sha256":hashlib.sha256(data).hexdigest()}
    path=tmp_path/"manifest.json";path.write_text(json.dumps(manifest))
    assert resolve_local_platform(path)["cad_path"]==str(tmp_path/"cad")
    (tmp_path/"cad").write_bytes(b"changed")
    with pytest.raises(ValueError,match="artifact changed"):resolve_local_platform(path)


def test_unknown_variant_rejected(tmp_path):
    path=tmp_path/"manifest.json";path.write_text(json.dumps({"variant":"original_08c"}))
    with pytest.raises(ValueError,match="verified r1"):resolve_local_platform(path)


def test_historical_review_must_match_reproduced_artifacts(tmp_path):
    review={"overall":"LOCAL_GEOMETRY_AND_SIMULATION_PASS_HARDWARE_UNVERIFIED",
            "cad":{"status":"LOCAL_MOUNT_GEOMETRY_PASS"},"grasp":{"full_grasp_pass":True},
            "reviewed_artifacts":{"cad":"not-the-local-cad","robot":"not-the-local-robot"}}
    files={"robot":b"robot", "cad":b"cad", "verification":json.dumps(review).encode()}
    manifest={"variant":"Wrist_Roll_08c_HexNut_M3_r1"}
    for name,data in files.items():
        (tmp_path/name).write_bytes(data)
        manifest[name]={"path":name,"sha256":hashlib.sha256(data).hexdigest()}
    path=tmp_path/"manifest.json"; path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match="historical review"):
        resolve_local_platform(path)
