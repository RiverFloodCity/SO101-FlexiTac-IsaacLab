# SPDX-License-Identifier: Apache-2.0
import argparse
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from so101_flexitac.isaaclab.build import inertia, mesh

p = argparse.ArgumentParser()
p.add_argument("--output_dir", type=Path, required=True)
a = p.parse_args()
scene = a.output_dir / "objects.usda"
s = Usd.Stage.CreateNew(str(scene))
root = UsdGeom.Xform.Define(s, "/Objects")
s.SetDefaultPrim(root.GetPrim())
UsdGeom.SetStageUpAxis(s, "Z")
UsdGeom.SetStageMetersPerUnit(s, 1.0)
table = trimesh.creation.box(extents=[1.2, 0.7, 0.04])
table.apply_translation([0.25, 0, -0.02])
mesh(s, "/Objects/TableVisual", table, (0.7, 0.7, 0.68))
mesh(s, "/Objects/TableCollision", table, (0.7, 0.7, 0.68), True, "convexHull")
b = UsdGeom.Xform.Define(s, "/Objects/Block")
b.AddTranslateOp().Set(Gf.Vec3d(0.23, 0, 0.05))
UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
inertia(
    b.GetPrim(),
    0.025,
    [0, 0, 0],
    np.diag(np.array([0.018**2 + 0.1**2, 0.026**2 + 0.1**2, 0.026**2 + 0.018**2]) * 0.025 / 12),
)
box = trimesh.creation.box(extents=[0.026, 0.018, 0.1])
mesh(s, "/Objects/Block/Visual", box, (0.55, 0.32, 0.12))
mesh(s, "/Objects/Block/Collision", box, (0.55, 0.32, 0.12), True, "sdf")
s.GetRootLayer().Save()
