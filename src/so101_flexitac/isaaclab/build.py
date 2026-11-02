# SPDX-License-Identifier: Apache-2.0
"""Reconstruct a portable USD from licensed upstream CAD and explicit local inputs."""

from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from scipy.spatial.transform import Rotation

from ..geometry import camera_parts, hex_mount_fasteners, optical_transform, parts
from ..kinematics import description, forward, origin
from ..spec import BASE_TRANSLATION, CAMERA, JOINTS, LIMITS, PAD


def px(prim, schema, **attrs):
    prim.AddAppliedSchema(schema)
    namespace = {
        "PhysxCollisionAPI": "physxCollision",
        "PhysxConvexDecompositionCollisionAPI": "physxConvexDecompositionCollision",
        "PhysxSDFMeshCollisionAPI": "physxSDFMeshCollision",
        "PhysxMaterialAPI": "physxMaterial",
        "PhysxRigidBodyAPI": "physxRigidBody",
        "PhysxContactReportAPI": "physxContactReport",
        "PhysxArticulationAPI": "physxArticulation",
    }[schema]
    for name, value in attrs.items():
        kind = (
            Sdf.ValueTypeNames.Bool
            if type(value) is bool
            else Sdf.ValueTypeNames.Int
            if type(value) is int
            else Sdf.ValueTypeNames.Token
            if isinstance(value, str)
            else Sdf.ValueTypeNames.Float
        )
        prim.CreateAttribute(namespace + ":" + name, kind).Set(value)


def quat(matrix):
    q = Rotation.from_matrix(matrix).as_quat()
    return Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3])))


def xform(stage, path, m):
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTransformOp().Set(Gf.Matrix4d(np.asarray(m).T.tolist()))
    return prim.GetPrim()


def mesh(stage, path, m, color, collision=False, approx="convexDecomposition"):
    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr(m.vertices.astype(np.float32).tolist())
    g.CreateFaceVertexCountsAttr([3] * len(m.faces))
    g.CreateFaceVertexIndicesAttr(m.faces.ravel().tolist())
    g.CreateSubdivisionSchemeAttr("none")
    g.CreateDisplayColorAttr([color])
    if collision:
        p = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approx)
        px(p, "PhysxCollisionAPI", contactOffset=0.0001, restOffset=0.0)
        if approx == "convexDecomposition":
            px(
                p,
                "PhysxConvexDecompositionCollisionAPI",
                maxConvexHulls=64,
                hullVertexLimit=64,
                shrinkWrap=True,
            )
        if approx == "sdf":
            px(p, "PhysxSDFMeshCollisionAPI", sdfResolution=128)
        g.CreateVisibilityAttr("invisible")
    return g.GetPrim()


def inertia(prim, mass, com, tensor):
    eig, axes = np.linalg.eigh(tensor)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    if eig.min() <= 0:
        raise ValueError("Invalid inertia")
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, eig)))
    api.CreatePrincipalAxesAttr(quat(axes))


def build(local_assets, output, camera=True, wrist_override=None):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    links = parts(local_assets, wrist_override=wrist_override)
    poses = forward({})
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(root.GetPrim())
    padmat = UsdShade.Material.Define(stage, "/Robot/Materials/Pad")
    material = UsdPhysics.MaterialAPI.Apply(padmat.GetPrim())
    material.CreateStaticFrictionAttr(1.0)
    material.CreateDynamicFrictionAttr(0.8)
    px(
        padmat.GetPrim(),
        "PhysxMaterialAPI",
        compliantContactStiffness=PAD["physics_kn"],
        compliantContactDamping=PAD["physics_kd"],
        compliantContactAccelerationSpring=False,
        frictionCombineMode="average",
    )
    bare = UsdShade.Material.Define(stage, "/Robot/Materials/Plastic")
    api = UsdPhysics.MaterialAPI.Apply(bare.GetPrim())
    api.CreateStaticFrictionAttr(0.5)
    api.CreateDynamicFrictionAttr(0.5)
    for link in description().findall("link"):
        name = link.get("name")
        path = "/Robot/" + name
        if not links[name]:
            continue
        p = xform(stage, path, poses[name])
        UsdPhysics.RigidBodyAPI.Apply(p)
        px(p, "PhysxRigidBodyAPI", solverPositionIterationCount=16)
        px(p, "PhysxContactReportAPI", threshold=0.0)
        inert = link.find("inertial")
        mass = float(inert.find("mass").get("value"))
        com = origin(inert)[:3, 3]
        i = inert.find("inertia")
        tensor = np.array(
            [[float(i.get("i" + a + b if a <= b else "i" + b + a)) for b in "xyz"] for a in "xyz"]
        )
        # Engineering measurements of the maintained assembled local mechanics.
        # Scalar mass properties are preserved; no private mesh/code is copied.
        if name == "gripper_link":
            mass = 0.0964043140411377
            com = np.array([-0.0006214054301381111, 0.0003280297969467938, -0.030430013313889503])
            axes = Rotation.from_quat(
                [
                    0.38903337717056274,
                    0.5902107954025269,
                    -0.5843392610549927,
                    0.3985622525215149,
                ]
            ).as_matrix()
            tensor = (
                axes
                @ np.diag([3.1822663004277274e-5, 6.15917015238665e-5, 7.262337749125436e-5])
                @ axes.T
            )
        elif name == "moving_jaw_so101_v1_link":
            mass = 0.013648119755089283
            com = np.array([-0.0026884509716182947, -0.03329693526029587, 0.019230090081691742])
            axes = Rotation.from_quat(
                [
                    0.5290279388427734,
                    0.4688524305820465,
                    0.483066588640213,
                    0.5166754722595215,
                ]
            ).as_matrix()
            tensor = (
                axes
                @ np.diag([1.950950263562845e-6, 6.653328910033451e-6, 7.977276254678145e-6])
                @ axes.T
            )
        if name == "gripper_link" and wrist_override:
            # Approximate inertia update: subtract removed PLA at 1240 kg/m3,
            # then add catalog-mass fasteners. Hardware weighing remains pending.
            original = parts(local_assets)["gripper_link"][1][1]
            revised = links["gripper_link"][1][1]
            removed = trimesh.boolean.difference([original, revised], engine="manifold")
            added = [(removed, -1240.0 * abs(removed.volume))]
            added.extend((m, value) for _, m, value in hex_mount_fasteners())
            second = tensor + mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
            first = mass * com
            for solid, delta_mass in added:
                solid = solid.copy()
                solid.density = abs(delta_mass) / abs(solid.volume)
                c = solid.center_mass
                second += np.sign(delta_mass) * solid.moment_inertia + delta_mass * (
                    np.dot(c, c) * np.eye(3) - np.outer(c, c)
                )
                first += delta_mass * c
                mass += delta_mass
            com = first / mass
            tensor = second - mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
        inertia(p, mass, com, tensor)
        for index, (source, m, mat) in enumerate(links[name]):
            color = (0.9, 0.66, 0.1) if mat == "3d_printed" else (0.1, 0.11, 0.12)
            mesh(stage, path + f"/Visuals/part_{index}", m, color)
            cp = mesh(stage, path + f"/Collisions/part_{index}", m, color, True)
            UsdShade.MaterialBindingAPI.Apply(cp).Bind(bare, materialPurpose="physics")
        if name == "gripper_link" and wrist_override:
            for label, solid, _ in hex_mount_fasteners():
                mesh(stage, path + "/Fasteners/" + label, solid, (0.45, 0.46, 0.48))
                mesh(
                    stage,
                    path + "/Fasteners/" + label + "Collision",
                    solid,
                    (0.45, 0.46, 0.48),
                    True,
                )
        if name == "gripper_link":
            # Fixed pad stack: adhesive, spacer and 0.5 mm compliant sensing layer.
            for label, x0, x1 in [
                ("Spacer", -0.0079, -0.0072),
                ("Adhesive", -0.0072, -0.0070),
                ("Pad", -0.0070, -0.0065),
            ]:
                m = trimesh.creation.box(extents=[x1 - x0, 0.0256, 0.0636])
                m.apply_translation([(x0 + x1) / 2, 0, -0.08])
                if label == "Pad":
                    front = m.vertices[:, 0] > ((x0 + x1) / 2)
                    m.vertices[front, 1] *= 0.026 / 0.0256
                    m.vertices[front, 2] = -0.08 + (m.vertices[front, 2] + 0.08) * 0.064 / 0.0636
                mesh(stage, path + "/" + label + "Visual", m, (0.12, 0.25, 0.32))
                if label == "Pad":
                    cp = mesh(
                        stage,
                        path + "/" + label + "Collision",
                        m,
                        (0.12, 0.25, 0.32),
                        True,
                        "convexHull",
                    )
                    UsdShade.MaterialBindingAPI.Apply(cp).Bind(padmat, materialPurpose="physics")
            m = np.array(
                [[0, 0, -1, -0.0065], [1, 0, 0, 0], [0, -1, 0, -0.08], [0, 0, 0, 1]],
                float,
            )
            xform(stage, path + "/TacSLSensor", m)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/Robot/base_link"))
    px(
        stage.GetPrimAtPath("/Robot/base_link"),
        "PhysxArticulationAPI",
        enabledSelfCollisions=True,
    )
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Robot/Joints/base_fixed")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/base_link")])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*BASE_TRANSLATION))
    for j in description().findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        name = j.get("name")
        m = origin(j)
        if not links.get(child):
            xform(stage, "/Robot/" + parent + "/" + child, m)
            continue
        joint = (
            UsdPhysics.FixedJoint if j.get("type") == "fixed" else UsdPhysics.RevoluteJoint
        ).Define(stage, "/Robot/Joints/" + name)
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/" + parent)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/" + child)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, m[:3, 3])))
        joint.CreateLocalRot0Attr(quat(m[:3, :3]))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
        joint.CreateCollisionEnabledAttr(False)
        if j.get("type") != "fixed":
            axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
            if not np.allclose(axis, [0, 0, 1]):
                raise ValueError("Only original Z joints supported")
            joint.CreateAxisAttr("Z")
            lo, hi = LIMITS[JOINTS.index(name)]
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            grip = name == "gripper"
            drive.CreateStiffnessAttr(2.0 if grip else 30.0)
            drive.CreateDampingAttr(0.08 if grip else 1.5)
            drive.CreateMaxForceAttr(0.25 if grip else 1.5)
            drive.CreateTypeAttr("force")
    if camera:
        mount, body = camera_parts()
        for name, m, mass, collision in [
            ("CameraMount", mount, 0.020, mount),
            ("CameraBody", body, 0.015, body.convex_hull),
        ]:
            p = xform(stage, "/Robot/" + name, poses["gripper_link"])
            UsdPhysics.RigidBodyAPI.Apply(p)
            # Community masses are estimates; use closed uniform-volume collision inertia.
            solid = collision.copy()
            solid.density = mass / solid.volume
            inertia(p, mass, solid.center_mass, solid.moment_inertia)
            mesh(stage, "/Robot/" + name + "/Visual", m, (0.15, 0.15, 0.15))
            mesh(
                stage,
                "/Robot/" + name + "/Collision",
                collision,
                (0.15, 0.15, 0.15),
                True,
                "convexDecomposition" if name == "CameraMount" else "convexHull",
            )
            j = UsdPhysics.FixedJoint.Define(stage, "/Robot/Joints/" + name)
            j.CreateBody0Rel().SetTargets(
                [Sdf.Path("/Robot/gripper_link" if name == "CameraMount" else "/Robot/CameraMount")]
            )
            j.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/" + name)])
            j.CreateCollisionEnabledAttr(False)
        # Camera is a child of the rigid body. Optical convention: +Z forward,+Y down.
        # USD camera convention: -Z forward,+Y up.
        optical = optical_transform()
        optical[:3, :3] = optical[:3, :3] @ np.diag([1, -1, -1])
        c = UsdGeom.Camera.Define(stage, "/Robot/CameraBody/WristCamera")
        c.AddTransformOp().Set(Gf.Matrix4d(optical.T.tolist()))
        c.CreateFocalLengthAttr(CAMERA["fx"] * 0.01)
        c.CreateHorizontalApertureAttr(CAMERA["width"] * 0.01)
        c.CreateVerticalApertureAttr(CAMERA["height"] * 0.01)
        c.CreateClippingRangeAttr(Gf.Vec2f(0.015, 10.0))
    stage.GetRootLayer().customLayerData = {
        "redistribution": "Contains local LeFlexiTac derivatives; NOT cleared for public distribution",
        "builder": "so101_flexitac",
        "mechanical_variant": "hex_m3_r1" if wrist_override else "original_08c",
        "inertia_status": "removed PLA density estimate plus catalog fasteners"
        if wrist_override
        else "maintained baseline",
    }
    stage.GetRootLayer().Save()
    return output
