# SPDX-License-Identifier: Apache-2.0
"""One environment, freely resting block and joint-driven grasp/release."""

import isaaclab.sim as sim
import torch
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.renderers import IsaacRtxRendererCfg

from ..spec import JOINTS
from .config import camera_cfg, robot_cfg, tactile_cfg


@configclass
class DemoCfg(DirectRLEnvCfg):
    decimation = 8
    episode_length_s = 60.0
    action_space = 6
    observation_space = 12
    sim = sim.SimulationCfg(
        dt=1 / 240,
        render_interval=8,
        physics=PhysxCfg(
            gpu_collision_stack_size=2**28,
            gpu_max_rigid_contact_count=2**18,
            gpu_found_lost_aggregate_pairs_capacity=2**18,
        ),
    )
    scene = InteractiveSceneCfg(
        num_envs=1, env_spacing=2.0, replicate_physics=False, lazy_sensor_update=False
    )


def make_cfg(robot, scene, rest, output, camera=True, render=True):
    c = DemoCfg()
    c.sim.log_dir = str(output)
    c.seed = 42
    c.scene.robot = robot_cfg(robot, "{ENV_REGEX_NS}/Robot", rest)
    c.scene.geometry = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Objects",
        spawn=sim.UsdFileCfg(usd_path=str(scene), activate_contact_sensors=True),
    )
    c.scene.block = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Objects/Block",
        spawn=None,
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.23, 0.0, 0.05)),
    )
    c.scene.tactile = tactile_cfg("{ENV_REGEX_NS}/Robot", "{ENV_REGEX_NS}/Objects/Block")
    c.scene.contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/gripper_link",
        update_period=1 / 240,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Objects/Block"],
    )
    c.scene.light = AssetBaseCfg(prim_path="/World/Light", spawn=sim.DomeLightCfg(intensity=650.0))
    if camera and render:
        c.scene.wrist = camera_cfg("{ENV_REGEX_NS}/Robot")
    if render:
        c.scene.global_camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/GlobalCamera",
            width=960,
            height=720,
            update_period=1 / 30,
            data_types=["rgb"],
            renderer_cfg=IsaacRtxRendererCfg(),
            spawn=sim.PinholeCameraCfg(
                focal_length=28.0, horizontal_aperture=36.0, clipping_range=(0.01, 10.0)
            ),
        )
    return c


class DemoEnv(DirectRLEnv):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.robot = self.scene["robot"]
        self.block = self.scene["block"]
        self.ids = torch.tensor(
            self.robot.find_joints(list(JOINTS), preserve_order=True)[0],
            device=self.device,
        )
        self.target = self.robot.data.default_joint_pos.torch[:, self.ids].clone()
        self.rest = self.target.clone()
        if "global_camera" in self.scene.sensors:
            self.scene["global_camera"].set_world_poses_from_view(
                eyes=torch.tensor([[0.7, -0.65, 0.6]], device=self.device),
                targets=torch.tensor([[0.15, 0.0, 0.15]], device=self.device),
            )

    def _setup_scene(self):
        pass

    def _pre_physics_step(self, actions):
        self.target = actions.clone()

    def _apply_action(self):
        self.robot.actuators.target_command.set_position_index(
            value=self.target, joint_ids=self.ids
        )

    def _get_observations(self):
        return {
            "policy": torch.cat(
                [self.robot.data.joint_pos.torch, self.robot.data.joint_vel.torch], -1
            )
        }

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        return torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        ), self.episode_length_buf >= self.max_episode_length

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self.robot.write_joint_position_to_sim_index(
            position=self.rest[env_ids], joint_ids=self.ids, env_ids=env_ids
        )
        self.robot.write_joint_velocity_to_sim_index(
            velocity=torch.zeros_like(self.rest[env_ids]),
            joint_ids=self.ids,
            env_ids=env_ids,
        )
        self.target = self.rest.clone()
        self.robot.actuators.target_command.set_position_index(
            value=self.target[env_ids], joint_ids=self.ids, env_ids=env_ids
        )
        self.block.write_root_pose_to_sim_index(
            root_pose=self.block.data.default_root_state.torch[env_ids, :7],
            env_ids=env_ids,
        )
        self.block.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((len(env_ids), 6), device=self.device),
            env_ids=env_ids,
        )
