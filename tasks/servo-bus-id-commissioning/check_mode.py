#!/usr/bin/env python3
"""Run check_reach.py or check_collisions.py for one run.sh mode: ``python check_mode.py <demo|all_plugged|wrong_order|recover> <reach|collisions>``."""
import os
import runpy
import sys
from pathlib import Path

VARS = {"demo": None, "all_plugged": "DEMO_ALL_PLUGGED", "wrong_order": "DEMO_WRONG_ORDER", "recover": "DEMO_RECOVER"}
mode, which = sys.argv[1], sys.argv[2]
if VARS[mode]:
    os.environ[VARS[mode]] = "1"
script = Path(__file__).resolve().parent / f"check_{'reach' if which == 'reach' else 'collisions'}.py"
sys.argv = [str(script)]
runpy.run_path(str(script), run_name="__main__")
