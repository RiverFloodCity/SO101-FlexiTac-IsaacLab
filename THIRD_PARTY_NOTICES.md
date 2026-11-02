# Third-party notices

Each borrowed file, URL, commit, SHA-256 and modification decision is listed in [docs/source_manifest.json](docs/source_manifest.json). This list is authoritative at file granularity.

| Source | Version | License / handling |
|---|---|---|
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | a758567c3978dfeefe282ede0500085a48fe8f78 | Apache-2.0, retained in licenses/SO-ARM100.txt; SO101 URDF, required meshes, exact Hex-Nut bracket, camera model and camera reference URDF. Files are unmodified; generated derivatives are local-only. |
| [adityakamath/so_arm_ros2](https://github.com/adityakamath/so_arm_ros2) | 82f383d94b2b9b2e6f8a44e19d191b053a94d0a6 | Declared Apache-2.0; original text retained in licenses/so_arm_ros2.txt. Copyright 2026 Aditya Kamath (Kamath Robotics). Placeholder camera YAML and transform Xacro retained unmodified. Numerics are used in a newly authored configuration. |
| [desmondzee/ICL-so101](https://github.com/desmondzee/ICL-so101) | 01782763fb8ad2473af96a345f91bfc319300326 | MIT; reference only, no file incorporated. Camera meshes obtained from SO-ARM100 rather than duplicated. |
| [LeFlexiTac](https://tna001-ai.github.io/LeFlexiTac/docs.html) | exact STL SHA-256 in manifest | Redistribution permission unconfirmed; user-local only. Attribution is not permission. Both source CAD and derived USD/meshes/images excluded from release candidate. |
| [NVlabs/vt-refine](https://github.com/NVlabs/vt-refine) | 81beb9ec2a5198c14931f31b8b243a7502d3c745 | NVIDIA License section 3.3 non-commercial research/evaluation restriction. No code/assets copied, adapted or vendored. Paper cited for the mathematical model. |
| [FlexiTac-IsaacSim-Simulation](https://github.com/binghao-huang/FlexiTac-IsaacSim-Simulation) | dc2e1c6607a1b1a0e4b196bb4e2d4ccdc1afdcb0 | No top-level redistribution license established; no files incorporated. |
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab) | 3.0.0 installed package | External dependency, BSD-3-Clause module headers, including official TacSL. Not copied into this repository. |
| Isaac Sim / PhysX | 6.2.0.0 runtime | External NVIDIA runtime under its own terms; not redistributed. |

New contributions: reproducible CAD-to-USD authoring, explicit local asset import, independent unilateral spring-damper implementation and Isaac Lab adapter, acquisition phase handling, diagnostic demo and license/mount checks. Engineering constants were read from the maintained local platform; no task code, control service, datasets, credentials or source folders were imported. This is not a claim of a clean-room process: the existing implementation was consulted read-only as authorized.

The original authors retain credit for SO101 mechanics, LeFlexiTac modifications, the camera bracket/community parameters, TacSL and VT-Refine. New code's Apache license does not override any upstream restrictions. No trademark endorsement is implied.


The authorized local r1 Wrist Roll is a LeFlexiTac derivative, not an Apache-cleared CAD asset. It is excluded from public source and wheel archives together with its rendered comparisons and generated USD. New generic restoration/checking scripts are independently authored. The exact official printable `Wrist_Roll_Follower_SO101.stl` is now included unchanged as an Apache-2.0 reference, with its pinned URL/hash in the manifest.

Fastener dimensional references only (no vendor CAD copied): [ETTINGER M3 DIN934/ISO4032 nut](https://www.ettinger.de/en/product-datasheet/9565ea9a3614479d7cc8eb87a306fedb/create), AF 5.5 mm and height 2.4 mm; [NBK SNS-M3-8-SD](https://www.nbk1560.com/en-US/products/specialscrew/nedzicom/socketheadcapscrew/SNS-SD/SNS-M3-SD/?SelectedLanguage=en-US), M3×8, head diameter 4.5 mm, height 3 mm. Smooth envelopes are newly generated dimensional proxies; neither thread contact nor bolt preload is simulated.
