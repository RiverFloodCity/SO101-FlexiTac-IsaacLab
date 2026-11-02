# SPDX-License-Identifier: Apache-2.0
"""Import only two explicit user-provided CAD files; never a private source tree."""

import argparse
import hashlib
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--wrist", type=Path, required=True)
p.add_argument("--jaw", type=Path, required=True)
p.add_argument("--output_dir", type=Path, default=Path(".local/assets"))
a = p.parse_args()
expected = {
    "Wrist_Roll_08c.stl": "7e78b577cd03f5c3e1ec5c2a5c1f0fb71cbaed23e6dd2bf76dc166a3b0b7bdeb",
    "Moving_Jaw_08d.stl": "c0f71e0134a7cd3a425488ba5cab87e16ee677f4768cd57036d21b9b81c6ea60",
}
a.output_dir.mkdir(parents=True, exist_ok=True)
for source, name in [(a.wrist, "Wrist_Roll_08c.stl"), (a.jaw, "Moving_Jaw_08d.stl")]:
    data = source.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected[name]:
        raise ValueError(f"Unreviewed geometry for {name}; registration must be revalidated")
    (a.output_dir / name).write_bytes(data)
print("Imported local-only files. Their redistribution permission remains unconfirmed.")
