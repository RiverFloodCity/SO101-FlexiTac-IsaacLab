# SPDX-License-Identifier: Apache-2.0
"""Original adapter to the separately installed BSD-licensed official TacSL SDF API."""

import torch
import warp as wp
from isaaclab.utils.math import quat_apply
from isaaclab_contrib.sensors.tacsl_sensor import VisuoTactileSensor

from ..spec import PAD
from ..tactile import taxel_samples


class FlexiTacSensor(VisuoTactileSensor):
    def __init__(self, cfg):
        self.history = None
        super().__init__(cfg)

    def _generate_tactile_points(self, *args, **kwargs):
        self._tactile_pos_local = torch.as_tensor(
            taxel_samples(), device=self._device, dtype=torch.float32
        )
        self.num_tactile_points = len(self._tactile_pos_local)
        # Local sensor XYZ axes match the established static-jaw pad, xyzw.
        self._tactile_quat_local = torch.tensor([-0.5, -0.5, 0.5, 0.5], device=self._device).repeat(
            self.num_tactile_points, 1
        )

    def _compute_tactile_forces_from_sdf(
        self,
        points_contact_object_local,
        sdf_values,
        sdf_gradients,
        contact_object_pos_w,
        contact_object_quat_w,
        elastomer_quat_w,
        env_ids,
    ):
        d = (-sdf_values[env_ids]).clamp_min(0)
        n = torch.nn.functional.normalize(sdf_gradients[env_ids], dim=-1)
        q = contact_object_quat_w[env_ids]
        p = contact_object_pos_w[env_ids]
        closest = points_contact_object_local[env_ids] + d[..., None] * n
        nworld = quat_apply(q[:, None].expand(-1, n.shape[1], -1), n)
        position = self._data.tactile_points_pos_w[env_ids]
        t = wp.to_torch(self._timestamp)[env_ids]
        v = torch.zeros_like(d)
        if self.history is not None:
            oldt, oldx, oldp, oldq = self.history
            delta = t - oldt[env_ids]
            valid = delta > 0
            oldsurface = (
                quat_apply(oldq[env_ids, None].expand(-1, n.shape[1], -1), closest)
                + oldp[env_ids, None]
            )
            surface = quat_apply(q[:, None].expand(-1, n.shape[1], -1), closest) + p[:, None]
            relative = (position - oldx[env_ids]) - (surface - oldsurface)
            v = torch.where(
                valid[:, None],
                -(relative * nworld).sum(-1) / delta.clamp_min(1e-9)[:, None],
                0.0,
            )
        else:
            self.history = (
                wp.to_torch(self._timestamp).clone(),
                self._data.tactile_points_pos_w.clone(),
                contact_object_pos_w.clone(),
                contact_object_quat_w.clone(),
            )
        signal = torch.where(d > 0, (PAD["kn"] * d + PAD["kd"] * v).clamp_min(0), 0.0)
        self._data.penetration_depth[env_ids] = d
        self._data.tactile_normal_force[env_ids] = signal
        self._data.tactile_shear_force[env_ids] = (
            0.0  # Single normal channel only; shear is not measured.
        )
        for dest, src in zip(self.history, (t, position, p, q)):
            dest[env_ids] = src.detach()

    def reset(self, *args, **kwargs):
        self.history = None
        super().reset(*args, **kwargs)

    def normal(self):
        return self.data.tactile_normal_force.reshape(-1, 12, 32, 9).mean(-1)
