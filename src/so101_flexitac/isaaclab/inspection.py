# SPDX-License-Identifier: Apache-2.0
"""Bounded initial-placement check against the live PhysX cooked shapes.

AABBs only avoid unnecessary exact solid boolean operations. No IK or policy
success is used as an admission filter. This fixed demo has no resampling.
"""

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def check_initialization(env, output_dir=None):
    from omni.physx import get_physx_cooking_interface
    from pxr import PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics, UsdUtils

    stage = env.sim.stage
    cache = UsdUtils.StageCache.Get()
    sid = cache.Insert(stage).ToLongInt()
    cook = get_physx_cooking_interface()
    prefix = "/World/envs/env_0"
    saved = []
    pieces = {}
    counts = {}
    names = env.robot.body_names
    poses = env.robot.data.body_link_pose_w.torch[0].cpu().numpy()

    def pose_matrix(p):
        t = np.eye(4)
        t[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
        t[:3, 3] = p[:3]
        return t

    frames = {name: pose_matrix(poses[i]) for i, name in enumerate(names)}
    frames["Block"] = pose_matrix(env.block.data.root_link_pose_w.torch[0].cpu().numpy())
    frames["Table"] = np.eye(4)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(prefix)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not prim.IsA(UsdGeom.Mesh):
            continue
        path = str(prim.GetPath())
        group = (
            path.split("/Robot/")[1].split("/")[0]
            if "/Robot/" in path
            else "Block"
            if "/Block/" in path
            else "Table"
        )
        g = UsdGeom.Mesh(prim)
        approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        batch = []
        if approx in ("convexHull", "convexDecomposition"):

            def receive(result, hulls, batch=batch):
                for hull in hulls:
                    # The convex envelope of the cooker's vertices is the actual cooked solid.
                    batch.append(trimesh.convex.convex_hull(np.array(hull.vertices)))

            cook.request_convex_collision_representation(
                stage_id=sid,
                collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPath()),
                run_asynchronously=False,
                on_result=receive,
            )
            if not batch:
                raise RuntimeError("No cooked collision pieces for " + path)
        elif approx == "sdf":
            mesh = trimesh.Trimesh(
                np.array(g.GetPointsAttr().Get()),
                np.array(g.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3),
                process=True,
            )
            if not mesh.is_watertight:
                raise RuntimeError("SDF source surface is open")
            batch = [mesh]
        else:
            raise RuntimeError("Unsupported collision approximation: " + str(approx))
        if output_dir is not None:
            directory = output_dir / "cooked_shapes"
            directory.mkdir(exist_ok=True)
            for mesh in batch:
                filename = f"shape_{len(saved):04d}.npz"
                np.savez_compressed(directory / filename, vertices=mesh.vertices, faces=mesh.faces)
                saved.append(dict(file=filename, group=group, prim=path))
        for mesh in batch:
            mesh.apply_transform(frames[group])
        pieces.setdefault(group, []).extend(batch)
        counts[path] = len(batch)
    if output_dir is not None:
        import json

        (output_dir / "cooked_shapes/index.json").write_text(json.dumps(saved, indent=2))
    violations = []
    pairs = [("Block", name) for name in pieces if name != "Block"] + [
        ("Table", name) for name in names if name != "base_link"
    ]
    for left, right in pairs:
        volume = 0.0
        for a in pieces.get(left, []):
            for b in pieces.get(right, []):
                if np.any(
                    np.minimum(a.bounds[1], b.bounds[1]) - np.maximum(a.bounds[0], b.bounds[0]) <= 0
                ):
                    continue
                volume += abs(trimesh.boolean.intersection([a, b], engine="manifold").volume)
        if volume > 1e-10:
            violations.append(dict(pair=[left, right], volume_m3=volume))
    return dict(
        status="PASS" if not violations else "FAIL",
        seed=env.cfg.seed,
        sampling="fixed, no resampling",
        entities={
            "block": {"xyz_m": [0.23, 0.0, 0.05], "orientation_xyzw": [0, 0, 0, 1]},
            "robot": "explicit operator rest",
            "table": "fixed",
        },
        violations=violations,
        cooked_piece_counts=counts,
        allowed_relations=[
            "base_link mounting plane on tabletop",
            "zero-volume block support contact",
        ],
        tolerance_m3=1e-10,
        scope="free block and table against actual cooked robot shapes; fixed robot internal assembly checked separately by check_mount.py",
        sdf_limit="Closed source collision surface used; not exact discretized SDF isosurface",
    )
