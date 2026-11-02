"""Backend-independent public platform specification. No simulator import on import."""

from .spec import CAMERA, JOINTS, PAD, asset_dir

__version__ = "0.1.0"

__all__ = ["JOINTS", "PAD", "CAMERA", "asset_dir"]
