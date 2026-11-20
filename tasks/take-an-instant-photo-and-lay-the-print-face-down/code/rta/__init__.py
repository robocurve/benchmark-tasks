"""Remember to Act (rta): memory tasks for the bimanual YAM in Isaac Lab-Arena, run through Inspect Robots.

Importing this package is cheap: nothing here imports Isaac Sim. ``rta.embodiment`` starts Kit on the
first ``reset()``; ``rta.scene`` and ``rta.world`` must be imported only after the app has started.
"""

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
TASKS_DIR = PACKAGE_DIR.parent / "tasks"
