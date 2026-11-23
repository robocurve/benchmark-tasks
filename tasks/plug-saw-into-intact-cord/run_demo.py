"""Run cabledemo.py in this machine's venv.

The installed inspect_robots_yam 0.37.0 imports inspect_robots.approver, which inspect_robots 1.0.0 lacks. The
inspect-robots CLI survives this because its plugin loader tries inspect_robots_yam first: that import fails, but
only after inspect_robots_yam.config/.packing are cached, which is all yam_arena needs. cabledemo.py imports
yam_arena directly, so this launcher does the same warm-up first and then runs cabledemo.py unchanged.

    python run_demo.py TASK_FILE OUT_DIR [--max-steps N]
"""

import pathlib
import runpy
import sys

try:
    import inspect_robots_yam  # noqa: F401
except ModuleNotFoundError:
    pass

here = pathlib.Path(__file__).resolve().parent
sys.argv = [str(here / "cabledemo.py"), *sys.argv[1:]]
runpy.run_path(str(here / "cabledemo.py"), run_name="__main__")
