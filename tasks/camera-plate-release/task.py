"""Release a camera/plate from a passive cam mount and place it upright on the pad.

Install this task folder in the pinned Arena environment, then use the
`camera_release` Inspect Robots embodiment. See README.md for exact commands.
"""

from camera_release.config import INSTRUCTION, MAX_STEPS
from camera_release.environment import camera_release_embodiment

__all__ = ["INSTRUCTION", "MAX_STEPS", "camera_release_embodiment"]
