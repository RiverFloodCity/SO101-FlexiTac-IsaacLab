# SPDX-License-Identifier: Apache-2.0
import argparse
from pathlib import Path

from so101_flexitac.isaaclab.build import build

p = argparse.ArgumentParser()
p.add_argument("--local_assets", type=Path, required=True)
p.add_argument("--output_dir", type=Path, required=True)
p.add_argument("--camera", choices=["true", "false"], default="true")
p.add_argument("--wrist_override", type=Path, default=None)
a = p.parse_args()
print(
    build(
        a.local_assets,
        a.output_dir / "robot.usda",
        a.camera == "true",
        wrist_override=a.wrist_override,
    )
)
