# SPDX-License-Identifier: Apache-2.0
"""Original implementation of unilateral Kelvin–Voigt readout.

Reference: Huang et al., VT-Refine, arXiv:2510.14930v2, Appendix B.2.
No VT-Refine code is incorporated. Compression is positive; SI geometry,
uncalibrated signal units. The simulator adapter supplies SDF and pose samples.
"""

import numpy as np

from .spec import PAD


def taxel_samples():
    r, c, s = PAD["rows"], PAD["cols"], PAD["subsamples"]
    a = (np.arange(s) + 0.5) / s - 0.5
    out = np.empty((r, c, s, s, 3))
    out[..., 0] = PAD["face_x"]
    out[..., 1] = (
        (np.arange(r)[:, None, None, None] - (r - 1) / 2) + a[None, None, :, None]
    ) * PAD["pitch"]
    out[..., 2] = (
        PAD["center_z"]
        + ((np.arange(c)[None, :, None, None] - (c - 1) / 2) + a[None, None, None, :])
        * PAD["pitch"]
    )
    return out.reshape(-1, 3)


def normal_signal(depth, closing_velocity, kn=PAD["kn"], kd=PAD["kd"]):
    depth = np.asarray(depth)
    v = np.asarray(closing_velocity)
    if kn < 0 or kd < 0 or not np.isfinite([kn, kd]).all():
        raise ValueError("Invalid coefficients")
    signed = kn * depth + kd * v
    return signed, np.where(depth > 0, np.maximum(signed, 0), 0)


class SampleClock:
    """Integer physics-step stamps; never relabel a stale sensor frame."""

    def __init__(self, physics_hz=240):
        self.physics_hz = physics_hz
        if physics_hz % 60 or physics_hz % 30:
            raise ValueError("Rates must divide physics rate")

    def stamp(self, step):
        return dict(step=int(step), time_s=step / self.physics_hz)

    def due(self, step, hz):
        return step % (self.physics_hz // hz) == 0
