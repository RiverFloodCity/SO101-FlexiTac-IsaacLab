# v0.1.0 — release preview

SO101 + single-sided 12×32 LeFlexiTac + InnoMaker U20CAM-1080P wrist RGB in Isaac Lab 3.0 / Isaac Sim 6.2. Includes original implementation, licensed upstream references, local r1 CAD generation, robot USD construction, demos, tests, citations and notices.

Run `python scripts/build_platform.py --download` after installation to obtain the two original CAD files directly from the official LeFlexiTac site, verify their hashes, and generate the local-only r1 platform. Local file inputs are also supported. The generated manifest identifies an exact reproduction of previously reviewed artifacts; a new simulation is not implied.

LeFlexiTac original STL files, modified CAD, generated full USD and derivative renders are not distributed: their redistribution permission is unconfirmed. Wrist camera calibration and masses are estimates. Hardware printing, assembly, cable routing and calibration remain unverified. See README, THIRD_PARTY_NOTICES.md and docs/validation.md.
