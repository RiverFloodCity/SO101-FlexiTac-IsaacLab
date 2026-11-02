# SPDX-License-Identifier: Apache-2.0
import numpy as np

from so101_flexitac.geometry import camera_parts, hex_mount_fasteners, overlap_volume


def test_selected_fasteners_fit_unchanged_bracket():
    mount, _ = camera_parts()
    fasteners = hex_mount_fasteners()
    assert len(fasteners) == 4
    for name, solid, mass in fasteners:
        assert solid.is_watertight and solid.is_volume
        assert mass > 0
        # CAD roundoff at a seated head is below 1e-5 mm^3.
        assert overlap_volume(mount, solid) * 1e9 < 1e-5
        if name.startswith("Screw"):
            # Correct 8 mm shaft + 3 mm head, with only community frame tilt.
            assert np.isclose(solid.extents[1], 0.011, atol=5e-6)
