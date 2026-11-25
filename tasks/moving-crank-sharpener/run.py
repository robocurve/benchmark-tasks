#!/usr/bin/env python3
"""Run this task in an activated, installed YAM/Arena environment."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parent
spec = json.loads((root/"task.json").read_text())
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("policy", choices=["demo", "dream", "idle"])
p.add_argument("--out", type=Path, required=True)
p.add_argument("--budget-ledger", type=Path)
a = p.parse_args()
cli = shutil.which("inspect-robots")
if cli is None:
    p.error("Activate the Arena environment and install this task first.")
policy = {"demo": "moving_crank_sharpener_demo", "idle": "moving_crank_sharpener_idle", "dream": "dream"}[a.policy]
if a.budget_ledger and a.policy == "dream":
    policy = "moving_crank_sharpener_dream_guard"
cmd = [cli, "run", "--embodiment", "moving_crank_sharpener_yam", "--policy", policy,
       "-E", "device=cuda:0",
       "-E", "task="+str(root/"scene.py"), "-E", "object_jitter=0", "-E", "control_hz=20",
       "-E", "cam_width=640", "-E", "cam_height=352", "--instruction", spec["instruction"],
       "--scorer", "task_score", "--max-steps", "1600", "--seed", "0",
       "--store-frames", "--log-dir", str(a.out.resolve())]
if a.policy == "dream":
    cmd += ["-P", "model=dreamlabs/dream-1-beta"]
if a.budget_ledger and a.policy == "dream":
    cmd += ["-P", "ledger_path="+str(a.budget_ledger.resolve())]
raise SystemExit(subprocess.call(cmd))
