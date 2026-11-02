# SO101 + LeFlexiTac + InnoMaker for Isaac Lab

Independent open-source platform for SO101 with one 12×32 FlexiTac pad and an InnoMaker U20CAM-1080P wrist RGB camera. Assets and geometry are separate from the Isaac Lab adapter. Only Isaac Lab 3.0 / Isaac Sim 6.2 is implemented. There are no private project imports, task datasets, hardware credentials, remote services or agent frameworks.

**Current r1: local geometry and simulation verified; hardware installation untested.** The original LeFlexiTac Wrist Roll remains unchanged and incompatible with the official Hex-Nut bracket. With explicit authorization, a separate printable copy restores the official two M3 holes and hex-nut recesses, plus a tiny stock-profile relief. It retains the official bracket, original Moving Jaw, servo/joint geometry and tactile installation. No adapter was introduced. See [validation](docs/validation.md) for dimensions, remaining wall, fasteners, motion checks and hardware limitations.

The custom LeFlexiTac CAD redistribution license is unconfirmed. Supply those two files locally; do not publish them, generated robot USDs, or renders containing their geometry until permission is resolved. The release ZIP deliberately excludes these derivatives. See [license audit](docs/license_audit.md), [file-level provenance](docs/source_manifest.json), and [validation](docs/validation.md).

## Local images (not in the public release ZIP)

The local r1 review is in `.local/mechanical_revision/final/`: `geometry_comparison.png`, `overlay_3d.png`, `local_sections.png`, `assembly.png`. These CAD derivative images remain local-only.

After running the demo, the output folder contains `global_camera_249.png`, `wrist_194.png`, and `tactile_heatmap.png`. These are actual RTX/TacSL outputs, not illustrative renders. The checked local evidence path is recorded in `docs/validation.md`.

## Installation

Linux x86_64, Python 3.12, NVIDIA RTX GPU with CUDA support and a driver supported by Isaac Sim 6.2. The independently tested environment uses Isaac Lab 3.0.0, Isaac Sim 6.2.0.0, PyTorch 2.11.0+cu130 and Warp 1.17.0. Isaac Sim is a separately licensed runtime, not included here. Review and accept its EULA when running it.

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python --overrides constraints-sim.txt \
  --prerelease allow --index-strategy unsafe-best-match \
  --extra-index-url https://pypi.nvidia.com \
  --extra-index-url https://download.pytorch.org/whl/cu130 \
  -r requirements-sim.lock
uv pip install --python .venv/bin/python --no-deps -e .
.venv/bin/python -m pytest -q
```

The lock lists installed packages, not bundled copies. `constraints-sim.txt` contains the Lab 3.0 compatibility overrides. Package installation must not copy another application's virtual environment or source tree.

## One-command local asset build

The [LeFlexiTac documentation](https://tna001-ai.github.io/LeFlexiTac/docs.html) provides the original Wrist Roll and Moving Jaw. Their redistribution permission is not confirmed, so these files and every generated derivative stay on your machine. After installation, either download the two official files as part of the build:

```bash
.venv/bin/python scripts/build_platform.py --download
```

or supply your own copies, without network access:

```bash
.venv/bin/python scripts/build_platform.py --wrist /path/to/Wrist_Roll_08c.stl \
  --jaw /path/to/Moving_Jaw_08d.stl
```

The command checks exact input hashes, creates the r1 bores and hex pockets, and builds the complete camera-equipped robot USD in a new `.local/platform_build/` directory. It refuses an existing output directory; use `--output_dir` to select a fresh one. It checks that the CAD and USD are byte-identical to the artifacts used in the historical numerical review. The generated `platform.json` is accepted by the public resolver:

```bash
export SO101_FLEXITAC_PLATFORM_MANIFEST="$PWD/.local/platform_build/platform.json"
.venv/bin/python -c 'from so101_flexitac.platform import resolve_local_platform; print(resolve_local_platform()["robot_path"])'
```

The linked numerical report describes earlier verification of those exact artifacts; generating the files does **not** run a new grasp, motion check, print test or hardware assembly. If upstream files or generated artifact hashes change, the builder fails rather than labeling them reviewed. Keep the original CAD, generated CAD/USD, review renders, rest input and the entire output local until rights are clarified. Fine-grained geometric review can be rerun with `scripts/review_hex_revision.py --local_assets .local/platform_build/assets --revision .local/platform_build/revision`.

Provide `.local/rest.json` containing `operator_confirmed: true`, `joint_positions_rad` with all six named joint values, `hardware_mapping_verified` and a truthful `provenance` description. There is intentionally no invented universal hardware rest pose. Names are `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`. No device IDs or calibration credentials belong in this repository. This simulator does not connect to or command hardware. Leader alignment remains an external required operation.

## Simulation diagnostic

```bash
OMNI_KIT_ACCEPT_EULA=yes .venv/bin/python scripts/demo.py \
  --robot .local/platform_build/robot/robot.usda --rest .local/rest.json \
  --mode grasp --steps 580 --seed 42 --device cuda:0 --viz none \
  --camera true --render true --output_dir .local/grasp
```

The fixed, seeded scene has a 1.2×0.7 m tabletop without legs, an empty gripper at the supplied rest pose and a free 26×18×100 mm wooden block of estimated mass 25 g. Joints physically approach, grasp, lift, lower, open and withdraw. Object pose writes occur only at reset. The controller uses model kinematics as a diagnostic, not a learned policy. The r1 run verifies the local restored geometry in simulation; printing, actual assembly, cable routing and load testing remain unperformed.

For performance, build a separate `--camera false` USD, then run `--mode hold --steps 150 --camera false` and `--mode hold --steps 150 --camera true` with otherwise identical settings. Retain both raw summaries; the first 30 control frames are warmup. Report wall time per control step and real-time factor separately. Sensor readback and plotting costs outside the timed step are not included.

## Public API

`so101_flexitac.spec` contains backend-neutral physical parameters; `kinematics`, `geometry` and `tactile` do not import a simulator. Explicit Isaac Lab adapters are:

```python
from so101_flexitac.isaaclab.config import robot_cfg, tactile_cfg, camera_cfg
# robot_cfg(usd_path, robot_prim, explicit_initial_joint_dictionary)
# tactile_cfg(robot_prim, contact_object_prim)
# camera_cfg(robot_prim, width=640, height=480)
```

Consumers install this package and pass local USD paths. They must not copy package source. The public `so101_flexitac.platform.resolve_local_platform()` interface resolves a local verified r1 manifest and checks the USD, CAD and numerical-review SHA-256 hashes. Set `SO101_FLEXITAC_PLATFORM_MANIFEST` to a local manifest when using an installed wheel; an editable source checkout uses `.local/platform.json` by default for legacy local setups. For the one-command build, set the environment variable as above. Consumers can select the optional profile without copying source or bundling restricted CAD. Wrist Camera configuration enables per-frame pose updates for dynamic optical extrinsics. A local CAD/motion PASS is not hardware approval. Existing robot profiles and old datasets stay unchanged.

## Sensor and model limitations

* One normal-only 12×32 pad on `gripper_link`, rows +Y, columns +Z, normal +X, 2 mm pitch and 3×3 area samples. Hardware taxel orientation remains unconfirmed.
* Independently authored unilateral Kelvin–Voigt readout following [VT-Refine Appendix B.2](https://arxiv.org/html/2510.14930v2): compression-positive `max(0, kn*d + kd*v)` inside contact, zero outside. `kn=1`, `kd=0.003`. Pose differencing estimates relative material-point velocity using simulation time; reset clears history. Readout units are an uncalibrated proxy, not validated newtons. No VT-Refine source code is included.
* PhysX compliant pad forces use a distinct `10000 N/m`, `10 Ns/m` material. Those physical parameters, the 0.5 mm pad, 0.2 mm adhesive and 0.7 mm spacer are preserved. Moving Jaw is unchanged. Wrist Roll defaults to the original; `--wrist_override` explicitly selects the separately restored r1 copy with unchanged registration and tactile faces.
* Official BSD-licensed Isaac Lab TacSL supplies PhysX GPU SDF queries. A contact object must have an SDF mesh. No contact or mirrored data is used to fabricate a second sensor.
* Camera K at 1280×960: fx=fy=508.7, cx=640, cy=480. These community FOV estimates are **not factory or measured calibration**. Zero Brown–Conrady/plumb_bob distortion means unknown distortion approximated as zero, not a distortion-free lens. The pinhole renderer implements this zero-distortion case; nonzero calibrated distortion is not yet supported.
* Extrinsics use the ROS2 optical frame directly relative to `gripper_link` (not relative to the bracket). USD uses -Z forward; ROS optical uses +Z forward. A 15 mm near clipping plane avoids the opaque internal lens mesh (its forward extent is 12.22 mm). This is a rendering choice, not a measured entrance pupil.
* Camera 15 g and bracket 20 g are community estimates. Inertias are newly computed from uniform closed collision solids and those estimated masses. Cable dynamics and bolt preload are not modeled.
* RGB and tactile start at t=0. At each 30 Hz output both report the same actual 240 Hz physics step; raw simulator floating timestamps are retained. The tactile cadence remains 60 Hz.

## Release preparation

```bash
.venv/bin/python scripts/audit_release.py --output_dir .local/release
```

For a motion-only test, use `--mode sweep --steps 680` with a fresh output directory. `scripts/check_motion.py --output_dir <demo-output>` checks saved PhysX cooked shapes against all recorded camera poses. `restore_hex_mount.scad` and Python boolean authoring provide editable modification source; OpenSCAD compilation itself was not tested.

Review `RELEASE_FILES.json` and `.local/release/release_audit.json`. `.local/release/public_candidate.zip` contains only allowlisted files. Nothing is pushed or remotely published. Apache-2.0 covers this project's new implementation; upstream notices remain intact and do not confer rights to excluded custom STL files. See [third-party notices](THIRD_PARTY_NOTICES.md) and `CITATION.cff`. The unchanged `docs/upstream_mount.md` is an archival source document: its relative CAD links refer to the pinned upstream repository, not this project; use the URLs in `docs/source_manifest.json`.
