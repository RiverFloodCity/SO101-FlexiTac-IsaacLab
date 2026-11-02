# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

import isaaclab.sim as sim
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import CameraCfg
from isaaclab_contrib.sensors.tacsl_sensor import VisuoTactileSensorCfg
from isaaclab_contrib.sensors.tacsl_sensor.visuotactile_sensor_cfg import (
    GelSightRenderCfg,
)
from isaaclab_physx.renderers import IsaacRtxRendererCfg

from ..spec import JOINTS


def robot_cfg(usd_path, prim_path, initial_joints):
    if set(initial_joints) != set(JOINTS):
        raise ValueError("Six explicit initial joints required; no claimed hardware default")
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim.UsdFileCfg(usd_path=str(Path(usd_path).resolve()), activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=initial_joints, joint_vel={".*": 0.0}),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=list(JOINTS[:5]),
                stiffness=30.0,
                damping=1.5,
                joint_effort_limit=1.5,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper"],
                stiffness=2.0,
                damping=0.08,
                joint_effort_limit=0.25,
            ),
        },
    )


def tactile_cfg(robot_prim, object_prim):
    return VisuoTactileSensorCfg(
        class_type="so101_flexitac.isaaclab.sensor:FlexiTacSensor",
        prim_path=robot_prim + "/gripper_link/TacSLSensor",
        update_period=1 / 60,
        enable_camera_tactile=False,
        enable_force_field=True,
        tactile_array_size=(12, 32 * 9),
        tactile_margin=0.001,
        contact_object_prim_path_expr=object_prim,
        normal_contact_stiffness=1.0,
        friction_coefficient=2.0,
        tangential_stiffness=0.1,
        render_cfg=GelSightRenderCfg(
            base_data_path="",
            sensor_data_dir_name="FORCE_ONLY",
            image_height=1,
            image_width=1,
            mm_per_pixel=1.0,
        ),
    )


def camera_cfg(robot_prim, width=640, height=480):
    if width * 3 != height * 4:
        raise ValueError("Placeholder intrinsics support 4:3 scaling only")
    return CameraCfg(
        prim_path=robot_prim + "/CameraBody/WristCamera",
        spawn=None,
        width=width,
        height=height,
        update_period=1 / 30,
        data_types=["rgb"],
        update_latest_camera_pose=True,
        renderer_cfg=IsaacRtxRendererCfg(),
    )
