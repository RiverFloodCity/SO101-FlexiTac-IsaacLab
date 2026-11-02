# Authorized Wrist Roll restoration r1 — 2026-11-02

**Local numerical result: geometry and simulation PASS within the checks below. Physical printing/installation/load testing: NOT PERFORMED.** The user explicitly authorized modification of a separate LeFlexiTac Wrist Roll copy after the original Hex-Nut incompatibility was demonstrated. No adapter was designed; the original STL, Moving Jaw, maintained consumer assets and official camera bracket remain unchanged. This supersedes the installation blocker only for the new local r1 geometry. The unmodified candidate and its older results remain separately identified below.

## Exact source alignment and modification

The newly downloaded exact official `STL/SO101/Individual/Wrist_Roll_Follower_SO101.stl` at pinned SO-ARM100 commit `a758567c3978dfeefe282ede0500085a48fe8f78` has SHA-256 `3f47a4b889b95a4c71046bda9ddfc9399d96d80a57ad7fc079700e240323a302`. It is not merely the simulation mesh. The original custom `Wrist_Roll_08c.stl` hash is unchanged (`7e78b577…b7bdeb`). Full hashes and original-source licensing are in the manifest. The official Hex-Nut bracket is the same unmodified pinned file as before.

Both wrist meshes were aligned into the existing gripper-link frame, with millimetres for CAD calculations. The official raw mesh transforms approximately as x'=x, y'=-y-0.218214, z'=-z+0.949706; the custom raw mesh uses the established registration and therefore approximately x'=x, y'=-y, z'=-z. Full matrices, including the original URDF's tiny angular rounding, are in the local `revision.json`. Shared servo seating surfaces, screw-axis sections and outer profiles were compared in that frame; no global ICP was used to hide local differences.

The custom model fills both official nut pockets and through-bores with solid wall, changes the nearby edge contour/fillet, and extends the tactile finger. Its original internal servo seat remains available. At the restored locations the wall spans approximately Y=17.2–24.0 mm. The editable modification uses two analytic circular/hexagonal cutters and a small, bounded stock-profile relief at the bracket locating tongue. There is no new attachment method and no reinforcement.

| Feature | Dimension in aligned gripper frame |
|---|---|
| Hole axes | +Y; (X,Z)=(-5,-23.4001) and (3.1,-23.4001) mm |
| Center spacing | 8.10 mm |
| Through-bore | Ø3.20 mm; nominal stock size, finer 96-sided tessellation |
| Hex pocket | 5.60 mm across flats; Y=17.2–20.5 mm, depth 3.30 mm |
| Bore land outside pocket | 3.50 mm to exterior Y=24 mm |
| Ligament between hex flats | 2.50 mm |
| Minimum sampled local ligament | Modified 1.5937 mm; official 1.3452 mm |
| Material removed | 239.206 mm³, 0.3534% of original custom solid |

The ligament metric is the minimum in-plane contour separation over 101 sections through the nut depth, not a global 3-D wall-thickness proof or strength calculation. Restoring the holes does not intersect the servo solid. The removed geometry lies at Z=-26.6333 to -12.5 mm, remote from the tactile face at Z=-112 to -48 mm. Mesh roundoff is at micrometre-submultiple scale; the tactile collision shape, sensor transform, Moving Jaw mesh and all joint attributes compare equal in the two USD variants.

## Fasteners, assembly access and printing

Use two M3×8 screws with a head that actually fits the unchanged bracket. Its head recess is about Ø5.4 mm: a generic Ø5.5 mm cap head is not automatically compatible. The virtual assembly uses a dimensionally documented [NBK SNS-M3-8-SD](https://www.nbk1560.com/en-US/products/specialscrew/nedzicom/socketheadcapscrew/SNS-SD/SNS-M3-SD/?SelectedLanguage=en-US) envelope (head Ø4.5×3 mm) and [ETTINGER M3 nuts](https://www.ettinger.de/en/product-datasheet/9565ea9a3614479d7cc8eb87a306fedb/create) (AF 5.5×height 2.4 mm). These are independently generated smooth proxies, not vendor CAD. The screw axes/seating planes follow the actual bracket transform; their small offset/tilt fits the restored bores. No thread engagement forces or bolt preload are modeled.

Nominal screw under-head position is around Y=26 mm, tip around Y=18 mm: the full 2.4 mm nut height is engaged, with roughly 0.9 mm tip clearance to the servo. Nuts seat at Y=18.1–20.5 mm, about 1 mm from the servo. M3×10 would intrude toward the servo and is not interchangeable. Small differences from these rounded positions are caused by the community bracket's 0.0008 rad tilt.

Follow the official access sequence: remove motor 6 with its Moving Jaw, introduce each nut through the empty motor cavity, seat it from inside along +Y, reinstall the motor using its existing M2 attachments, then fit the official bracket using the two M3 screws. Continuous convex swept-volume tests for the nut entry and seating paths are clear. All nut/screw versus revised wrist/servo intersections are zero; seated-head/bracket roundoff is below 1e-5 mm³. Do not attempt installation with motor 6 occupying the nut-entry cavity.

The revised wrist versus official bracket and camera hull is clear. At 101 jaw angles from -0.17 to 1.4 rad, camera/bracket/new fasteners versus Moving Jaw are clear. The original CAD already has a nominal wrist/servo overlap of 10.6809 mm³; this is identical before/after, and the removed region has zero servo overlap. It is disclosed as an inherited model limitation, not certified as physically harmless by this check. No existing internal free space is consumed by the seated nuts/tips: they remain inside the previously solid wall. Existing cable channels are untouched; an external camera cable, bend radius and strain-relief sweep are not modeled and still require physical review.

The output is a single closed, consistently wound, positive-volume mesh with zero degenerate faces. All 595 horizontal sections at 0.2 mm spacing close successfully. This supports geometric printability, not a completed print trial or machine-specific slicer validation. Horizontal holes/pocket roofs require suitable support/bridging strategy. The stock AF clearance is only 0.05 mm per nut flat, so print a fit coupon and apply printer/material compensation rather than force-fitting a nut. Retain at least three 0.4 mm perimeter lines in the narrow ligament and inspect the actual slicer result. No FEA, tightening-torque qualification, fatigue test or load certification has been performed.

## Independent Isaac Lab verification

The new USD uses an explicit `--wrist_override`; the original default asset remains unchanged. It includes the official bracket/camera and nominal fasteners. The assembled link's estimated inertia subtracts removed PLA at 1240 kg/m³ and adds catalog fastener masses; measured mass/inertia remain pending. Pad geometry, kn=1, kd=0.003, PhysX stiffness 10000 N/m and damping 10 Ns/m are unchanged.

The final seed-42 physical grasp completed 580 control steps (19.333 simulated seconds) with approach, physical grasp, lift, placement, release and withdrawal. Maximum lift is 65.879 mm; tactile peak is 0.000686913 proxy units; final block center z=0.050000038 m and final tactile is zero. The original success thresholds were retained. Reset passes. Early fixed-orientation placement attempts tipped the slender block; the final controller uses the measured held pose to place it near support before opening, then separates before lifting. There are no object pose writes except reset and no hidden attachment.

A separate 680-step empty-gripper sweep reaches wrist roll -169.99999° to +169.96260° and jaw -9.61628° to +80.21409°. The grasp also exercises wrist flex from approximately -102.04° to +8.83°. These ranges are measured achieved motion, not merely commands. Seven standalone tests pass, including fastener/bracket fit regression.

Every recorded frame in both final runs (580+680 at 30 Hz) was checked against the actual PhysX cooked camera/mount collision solids and other robot links, excluding only the named fixed mating relations. All tested non-mating pair intersection volumes are zero. This is not a continuous-time/full six-joint-workspace collision certificate. Self-collision physics remains enabled. The camera-to-gripper relative translation drift stays below 4.86e-8 m and angular drift below 4.45e-7 rad. RGB and tactile update timestamps agree at every recorded frame. Actual wrist RGB and single-sided tactile images were inspected.

## Clean installation and reproducibility check (2026-11-02)

A new isolated Python 3.12 virtual environment installed the locked Isaac Lab / Isaac Sim dependencies from the documented package indexes and installed this project from the candidate source. Both official STLs were downloaded through the one-command builder and matched their recorded SHA-256 digests; the generated r1 STL and robot USD matched the historical reviewed hashes. Six current tests passed. A 45-step GPU hold with both cameras and tactile loaded the freshly generated USD, produced actual RGB frames and a zero-contact tactile frame, and passed reset. Final RGB and tactile timestamps agreed. This was a short installation smoke test, not a new full grasp or hardware verification. Demo runs require more than 30 steps because the first 30 are timing warmup.

## Delivery and boundary

Local-only deliverables are in `.local/mechanical_revision/final/`: `Wrist_Roll_08c_HexNut_M3_r1.stl`, preserved original/reference STLs, `restore_hex_mount.scad`, `revision.json`, dimensional comparison/section/overlay/assembly PNGs and compact checks. The authoritative, executed editable source is `scripts/restore_hex_mount.py`; the equivalent OpenSCAD CSG file is supplied but was not compiled with OpenSCAD in this environment. `scripts/review_hex_revision.py` regenerates dimensions, diagrams and geometric slicing checks.

Final robot: `.local/mechanical_revision/platform/robot.usda`. Final physical evidence: `.local/mechanical_revision/grasp_verified/` and `sweep/`, including recorded poses, cooked shape archives, RGB, tactile and reset evidence. Superseded unsuccessful runs are removed after preserving their useful conclusion above. At the time of that revision no consumer source or asset was changed, and no remote repository was created. The former blocked candidate remains historical; the r1 interface is available through the public package.

The modified CAD, imported-geometry SCAD package, generated USD and derivative renders remain excluded from public archives until LeFlexiTac redistribution rights are confirmed. Original source hashes remain unchanged. Public code, exact licensed official reference, notices and manifest are updated and re-audited. **A numerical PASS is not an actual installed/printed platform PASS.**

---

# Original unmodified candidate — retained baseline results

**Overall: BLOCKED_MECHANICAL_FIT.** The independent software candidate runs in Isaac Lab, but the required unchanged hardware assembly is not physically installable. Diagnostic simulation success must not be interpreted as platform acceptance, task acceptance or data readiness. Nothing has been pushed to a remote repository.

## Environment and independence

A separately installed Python 3.12 environment ran Isaac Lab 3.0.0, Isaac Sim 6.2.0.0, PyTorch 2.11.0+cu130 and Warp 1.17.0 on one NVIDIA RTX 4090 (driver 595.99.02, reported memory 49140 MiB). This GPU memory configuration is not a standard 24 GB retail-card baseline. The environment has no private project package. A built wheel was also installed into a separate minimal environment: public imports, six-joint kinematics, 384 taxels and packaged robot/camera meshes loaded without a private project import. Simulation is supported only through Isaac Lab.

Six standalone tests pass, covering normal stiffness/damping and unilateral unloading, the explicit taxel lattice and area samples, shared acquisition time, wrist-versus-jaw attachment, and provisional camera parameters. Three integration tests confirm FAIL, UNVERIFIED and missing mount status cannot activate the candidate. These tests complement, rather than replace, the real simulation below.

## Protected mechanics and mounting

The local custom STL hashes match the reviewed inputs. Symmetric mesh-vertex differences and local-transform differences against the maintained platform are exactly zero for the custom Wrist Roll visual, Moving Jaw visual and pad collision shape. The source meshes were not modified. Joint mapping, pad registration and physical/readout parameters remain unchanged.

Closed-solid boolean tests used two nominal M3 passage cylinders (radius 1.5 mm, length 6 mm, +Y axis), centered at [-5,21,-23.4] and [3.1,21,-23.4] mm in the gripper frame. Stock SO101 obstruction is zero; custom Wrist_Roll_08c obstruction is 100% at both locations within numerical precision. Required mounting holes/nut access are absent. The bracket also intersects the custom wrist by 0.24049 mm³ at a locator feature. Merely changing a camera pose cannot repair the required fastener engagement.

At 33 sampled jaw angles from -0.17 to 1.4 rad, camera/bracket versus Moving Jaw solid intersections were zero. This is a sampled local test, not a continuous swept-volume or full-workspace self-collision certificate. Camera hull and bracket convex-decomposition collision bodies are present, but complete noninterference acceptance FAILS because the protected mounting interface is incompatible. A separately authorized removable adapter requires its own design and verification; none is silently introduced here.

## Real simulation diagnostic

The candidate USD loaded as a fixed-base articulated robot. Six commanded joints, including wrist roll and moving jaw, moved during a complete physical grasp, lift, lower, release and withdrawal. The camera is fixed to the wrist chain; its optical transform is independent of the moving jaw, and actual RTX wrist images show the manipulated block between the jaws. The camera optical origin and orientation follow the community gripper-frame reference; a 15 mm near plane prevents the model's own opaque lens mesh from occluding the view.

Seed 42; all entities are fixed rather than randomized. The 1.2×0.7 m legless table supports a free 26×18×100 mm, estimated 25 g block at [0.23,0,0.05] m, identity orientation. The robot begins empty at the operator-confirmed measured local rest posture from 2026-11-01. That local input is excluded from the public release; no universal hardware zero or calibrated Leader–Follower mapping is claimed. There is no IK admission filtering or resampling.

Initialization checked the free block and table against actual PhysX cooked robot convex pieces using solid intersections, with explicitly allowed base mounting/support relations. It passed for this fixed state. The block SDF check uses its closed source surface, not an exact discretized SDF isosurface. Internal mounting is checked separately and fails as above. These scoped checks do not establish that the entire assembled candidate is geometrically legal.

The run completed 440 control steps (14.667 simulated seconds) at 240 Hz physics, 60 Hz single-sided tactile and 30 Hz RGB. Its 60-second timeout was not reached. Success requires block center z > 0.09 m throughout frames 250–279, tactile peak > 1e-6 during that interval, final center z within 5 mm of 0.05 m, and the last five frames below 1e-6 tactile. The diagnostic passed: maximum lift 66.261 mm, maximum normal readout 0.000728533 proxy units, final block z 0.0503149 m, final tactile zero and final contact force zero. These proxy units are not calibrated force units. The object was never attached or teleported during manipulation; object pose writes occur only at reset.

Single-sided 12×32 geometry, 2 mm pitch, 3×3 area sampling and kn=1, kd=0.003 were preserved. PhysX compliance separately uses 10000 N/m and 10 Ns/m. Initial and post-release noncontact readings are zero. Reset restores the block pose, clears tactile velocity history and returns joints within 5.71e-8 rad of the supplied rest configuration; reset tactile is zero.

At every one of the 440 recorded RGB frames, actual RGB and tactile update timestamps are equal (maximum difference 0 s). Both acquisition schedules are phase-initialized at t=0. Simulator float timestamps accumulate about 0.181 ms of shared drift by the final frame; integer physics steps are retained as the exact time reference. This verifies alignment at RGB output times, not external hardware clock calibration.

## Paired performance measurement

Three sequential baseline/camera pairs, each 150 control steps with the first 30 excluded as warmup. Both use one environment, 240 Hz PhysX, 60 Hz tactile with 3456 SDF samples, and 30 Hz 960×720 global RTX. The camera variant adds the 640×480 wrist stream at 30 Hz and its estimated 35 g camera/bracket assembly. Timing wraps env.step and CUDA synchronization; readback and plotting outside that interval are excluded.

| Condition | Three run means (ms/control step) | Median (ms) | Real-time factor |
|---|---|---|---|
| Baseline | 117.980, 117.247, 118.725 | 117.980 | 0.2825 |
| Wrist camera | 125.045, 124.583, 122.535 | 124.583 | 0.2676 |

Median step-time increase is **5.60%**. These are single-environment measurements on this machine, not a throughput or real-time guarantee. Per-run p95 and initialization/reset results are retained in `validation.json`; timings show system noise and are not confidence intervals.

## Integration and release scope

The consumer uses the external `so101-flexitac-isaaclab` distribution and public `robot_cfg`, `tactile_cfg`, `camera_cfg` factories. A new optional profile and adapter exist; the adapter rejects the current FAIL mount report before loading an asset. Existing profiles, task selection and datasets are unchanged. Interface import/rejection checks passed; a normal integrated task rollout is deliberately NOT accepted or enabled before mechanical fit passes.

The source ZIP contains the explicit `RELEASE_FILES.json` allowlist, Apache-licensed new implementation, licensed upstream files, notices, exact per-file provenance, citations, dependency lock, demo and tests. Pattern-based sensitive-data/private-import checks and borrowed-file digest verification are automated; they are bounded checks, not a legal opinion or proof against every possible secret. The two unlicensed custom STL files, generated USDs, rest input, local renders, runtime environment and logs are excluded. The wheel carries upstream licenses, notices and the source manifest.

Remaining release/acceptance blockers: (1) actual Hex-Nut mounting incompatibility without altering protected parts; (2) permission to redistribute custom LeFlexiTac CAD and derivatives; (3) measured camera intrinsics/distortion, hardware extrinsics and mass/inertia validation remain pending. Full-workspace collision certification and hardware Leader alignment are not claimed. No alternate simulator backend is implemented.

## Retained local evidence

Paths below are relative to the independent project root and intentionally excluded from the source ZIP:

* `.local/platform_candidate/robot.usda` and `.local/platform_baseline/robot.usda`: generated embedded assets for reproduction and paired testing.
* `.local/verification/mount_geometry.json`: failed fastener and local solid-intersection review.
* `.local/verification/protected_geometry.json`: unchanged protected geometry comparison.
* `.local/verification/grasp/result.json`, `initialization.json`, `reset.json`: full final diagnostic and reset evidence.
* `.local/verification/grasp/global_camera_249.png`, `wrist_194.png`, `tactile_heatmap.png`: actual RTX/TacSL images. Other retained milestone images show initial, approach, contact and released states.
* `.local/verification/benchmark/performance.json`: final paired performance summaries.
* `.local/release/`: audited source ZIP, wheel and file/hash audit.

Superseded diagnostic runs and redundant hold-only benchmark images/logs were removed after their conclusions were incorporated here. The final evidence remains local pending permission and maintainer review.
