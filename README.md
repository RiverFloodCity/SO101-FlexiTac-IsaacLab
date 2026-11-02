# SO101 + FlexiTac + Wrist Camera

An Isaac Lab robot platform that brings together an **SO101 arm**, a **single-sided 12 × 32 FlexiTac tactile sensor**, and an **InnoMaker U20CAM-1080P wrist camera**. Use it to study contact-rich manipulation with synchronized vision, tactile sensing, and robot state.

[Get started](#quick-start) · [Run the demo](#run-the-demo) · [Use in your project](#use-in-your-project) · [Validation](docs/validation.md) · [License and attribution](#license-and-attribution)

> **Status:** The platform has been tested in simulation. The modified wrist has **not** been printed or assembled on hardware, and the camera parameters are estimates, not a measured calibration.

## What is included

- **Robot:** SO101 articulation with a locally generated r1 Wrist Roll, the original LeFlexiTac Moving Jaw, and the official Hex-Nut camera mount.
- **Touch:** one 12 × 32 normal-force proxy map using PhysX and Isaac Lab TacSL, with spring and damping terms (`kn = 1`, `kd = 0.003`). Readings are not calibrated in newtons.
- **Vision:** a wrist-mounted RGB camera with estimated intrinsics and extrinsics; the demo also renders a global view.
- **Examples and tools:** a physical grasp-and-release demo, asset builder, tests, and Isaac Lab configuration factories.

The reference simulation uses 240 Hz physics, 60 Hz tactile sensing, and 30 Hz RGB. The grasp demo moves a free object through physical contact; it does not attach the object to the gripper.

## Quick start

### 1. Install

You need Linux x86-64, Python 3.12, an NVIDIA RTX GPU, a compatible driver, and [uv](https://docs.astral.sh/uv/getting-started/installation/). The tested stack is Isaac Lab 3.0.0 and Isaac Sim 6.2.0.0. Isaac Sim is a separate NVIDIA-licensed runtime; review its terms before use.

```bash
git clone https://github.com/RiverFloodCity/SO101-FlexiTac-IsaacLab.git
cd SO101-FlexiTac-IsaacLab

uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python --overrides constraints-sim.txt \
  --prerelease allow --index-strategy unsafe-best-match \
  --extra-index-url https://pypi.nvidia.com \
  --extra-index-url https://download.pytorch.org/whl/cu130 \
  -r requirements-sim.lock
uv pip install --python .venv/bin/python --no-deps -e .
```

### 2. Build the robot assets

```bash
.venv/bin/python scripts/build_platform.py --download
```

This downloads the two original files from the [official LeFlexiTac documentation](https://tna001-ai.github.io/LeFlexiTac/docs.html), checks their SHA-256 hashes, generates the modified Wrist Roll, and builds the robot USD under `.local/platform_build/`.

Already have the official files? Build without downloading:

```bash
.venv/bin/python scripts/build_platform.py \
  --wrist /path/to/Wrist_Roll_08c.stl \
  --jaw /path/to/Moving_Jaw_08d.stl
```

The output directory must not already exist. To build again, choose a fresh location with `--output_dir`. The builder checks the generated files against the previously reviewed r1 artifacts; it does not run a new simulation test.

The redistribution terms for the LeFlexiTac custom STL files are not yet confirmed. They and the CAD/USD generated from them are **local only** and are not included in this repository. See the [license audit](docs/license_audit.md).

### 3. Check the installation

```bash
.venv/bin/python -m pytest -q
export SO101_FLEXITAC_PLATFORM_MANIFEST="$PWD/.local/platform_build/platform.json"
.venv/bin/python -c 'from so101_flexitac.platform import resolve_local_platform; print(resolve_local_platform()["robot_path"])'
```

The last command should print the path to `robot/robot.usda`. The manifest lets other Python projects find the local assets without copying this package.

## Run the demo

The demo starts with an empty gripper and a free block, then approaches, grasps, lifts, places, and releases it. To use your own setup, first create `.local/rest.json` with an **operator-confirmed** rest posture. Enter all six joint positions in radians; do not use the placeholders below as robot commands:

```json
{
  "operator_confirmed": false,
  "hardware_mapping_verified": false,
  "provenance": "Describe how the posture and mapping were checked",
  "joint_positions_rad": {
    "shoulder_pan": null,
    "shoulder_lift": null,
    "elbow_flex": null,
    "wrist_flex": null,
    "wrist_roll": null,
    "gripper": null
  }
}
```

Replace the `null` values with your measured values and set `operator_confirmed` to `true` only after confirmation. Set `hardware_mapping_verified` truthfully. The simulator does not connect to hardware; this example is not a substitute for Leader–Follower alignment.

```bash
OMNI_KIT_ACCEPT_EULA=yes .venv/bin/python scripts/demo.py \
  --robot .local/platform_build/robot/robot.usda \
  --rest .local/rest.json \
  --mode grasp --steps 580 --seed 42 --device cuda:0 --viz none \
  --camera true --render true --output_dir .local/grasp
```

The output includes global and wrist RGB images, a tactile heatmap, simulation results, and a reset check. See [validation](docs/validation.md) for the tested motion, contact checks, and known limits. A different initial posture may require a different motion plan.

## Use in your project

Install the package in the same Python environment as Isaac Lab. Point `SO101_FLEXITAC_PLATFORM_MANIFEST` to your generated `platform.json`, then use the provided configuration factories:

```python
from so101_flexitac.platform import resolve_local_platform
from so101_flexitac.isaaclab.config import robot_cfg, tactile_cfg, camera_cfg

platform = resolve_local_platform()
# robot_cfg(platform["robot_path"], robot_prim, initial_joints)
# tactile_cfg(robot_prim, contact_object_prim)
# camera_cfg(robot_prim, width=640, height=480)
```

Supply all six initial joint positions explicitly. Contact objects used with TacSL require an SDF collision mesh. The platform specifications and geometry are separated from the Isaac Lab adapter; Isaac Lab is the only supported simulation backend.

## Model and validation notes

- The r1 wrist restores two M3 mounting holes and hex-nut pockets in a **local copy** of the LeFlexiTac wrist. The official mount and Moving Jaw are retained. See [geometry and motion validation](docs/validation.md) for clearances and the scope of the checks.
- Tactile readout follows a unilateral spring–damper model inspired by [VT-Refine, Appendix B.2](https://arxiv.org/abs/2510.14930). The implementation is independently authored; VT-Refine source code is not included.
- At 640 × 480, the estimated camera matrix uses `fx = fy = 254.35` and `cx = 320`, `cy = 240`. Unknown lens distortion is approximated as zero; nonzero distortion is not implemented.
- Camera and mount masses are estimates. Cable dynamics, bolt preload, print tolerances, and hardware installation have not been validated.

## License and attribution

Original project code is released under [Apache-2.0](LICENSE). See [third-party notices](THIRD_PARTY_NOTICES.md) and the [per-file source manifest](docs/source_manifest.json) for upstream versions, licenses, and attribution. If you use the platform in research, please also see [CITATION.cff](CITATION.cff).
