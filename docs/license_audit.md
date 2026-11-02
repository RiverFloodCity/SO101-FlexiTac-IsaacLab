# Source and license review — 2026-11-02

Completed before authoring implementation. `source_manifest.json` records each borrowed file, exact revision, digest, modifications and redistribution decision.

* SO-ARM100: Apache-2.0; original license retained. Individual URDF and mesh files downloaded directly from the pinned upstream, not copied from a private project. Hex-Nut original millimetre mesh and simulation-frame mesh retained for registration audit.
* so_arm_ros2: identifies Apache-2.0; its license text differs from the standard template, preserved verbatim. Only explicitly marked placeholder numerical calibration and frame reference used. Copyright 2026 Aditya Kamath (Kamath Robotics). No ROS implementation copied.
* ICL-so101: MIT at 01782763fb8ad2473af96a345f91bfc319300326; reference reviewed, no files incorporated; equivalent camera assets obtained from SO-ARM100.
* LeFlexiTac: custom Wrist_Roll_08c and Moving_Jaw_08d source attribution retained. Separate redistribution permission not established. Local user-provided inputs only; derived meshes/USD and images containing them are NOT approved for redistribution by this audit.
* NVlabs/vt-refine: NVIDIA License, section 3.3 restricts use to research/evaluation non-commercially. No source, assets or converted code incorporated. Mathematical model independently implemented from the paper, not relicensed code.
* FlexiTac-IsaacSim-Simulation: no top-level license found at dc2e1c6607a1b1a0e4b196bb4e2d4ccdc1afdcb0; per-file BSD headers do not establish rights to all assets. No files incorporated.
* Isaac Lab 3.0 official installed TacSL: BSD-3-Clause per module headers, external runtime dependency; not vendored. Isaac Sim/PhysX runtime remains subject to NVIDIA runtime terms.
* Existing platform: read-only engineering reference for physical dimensions, scalar gains, rigid-body parameters and joint convention. No private implementation copied. Device identities, calibration files, task source/data, credentials and execution framework are excluded.

This records engineering provenance and explicit license evidence, not a grant of missing third-party rights. Public release remains subject to maintainer review.
