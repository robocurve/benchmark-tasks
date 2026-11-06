"""Play one of the task's plans with ``inspect-robots run --policy yam_scripted``, as the README's command does, and
write ``summary.json`` to the log folder: the score, how the trial ended, its control steps, the last scored
quantities (``TASK_TRACE``), the versions, the GPU, the wall-clock time and the hashes of the task's source files
and of the video. The same record, renamed, goes in ``validation/``.

    python tests/run_plan.py PLAN LOG_DIR [--video]

``--video`` records the top camera to LOG_DIR/PLAN.mp4 (and its first frame to PLAN.png). Kit is told to render on
the first visible GPU only, which matters on machines with several GPUs; choose it with CUDA_VISIBLE_DEVICES.
"""

import hashlib
import importlib.metadata
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
plan, log_dir = sys.argv[1], Path(sys.argv[2]).resolve()
log_dir.mkdir(parents=True, exist_ok=True)
os.environ["DEMO_PLAN"] = plan
os.environ["TASK_TRACE"] = str(log_dir / "trace.jsonl")  # task.py appends one line per scored step
video = "--video" in sys.argv[3:]
if video:
    os.environ["TOP_CAM_VIDEO"] = str(log_dir / f"{plan}.mp4")
sys.path.insert(0, str(HERE))
import task  # noqa: E402

from inspect_robots.cli import main  # noqa: E402


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_hash() -> str:
    """One hash over the task's code and task.json (not runs, media or caches)."""
    digest = hashlib.sha256()
    for path in sorted(HERE.rglob("*")):
        rel = path.relative_to(HERE)
        if path.is_file() and path.suffix in (".py", ".json", ".ini") and rel.parts[0] not in ("runs", "validation") \
                and "__pycache__" not in rel.parts:
            digest.update(rel.as_posix().encode() + b"\0" + _sha256(path).encode())
    return digest.hexdigest()


def _versions() -> dict:
    out = {}
    for name in ("isaacsim", "isaaclab", "isaaclab_arena", "yam-arena", "inspect-robots", "inspect-robots-yam"):
        try:
            out[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            out[name] = None
    try:
        import yam_arena

        out["yam-arena revision"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=Path(yam_arena.__file__).resolve().parent, capture_output=True,
            text=True, check=True).stdout.strip()
    except Exception:  # not a git checkout
        out["yam-arena revision"] = None
    return out


args = ["run", "--embodiment", "yam_arena", "--policy", "yam_scripted", "-E", f"task={HERE / 'task.py'}",
        "-E", "object_jitter=0", "-E", "control_hz=20", "-E", "cam_width=320", "-E", "cam_height=176",
        "--instruction", task.INSTRUCTION, "--scorer", "task_score", "--max-steps", str(task.MAX_STEPS),
        "--store-frames", "--log-dir", str(log_dir)]
sys.argv = [sys.argv[0], "--/renderer/multiGpu/activeCudaGpus=0,"]  # Kit reads its settings from argv
started = time.time()
code = main(args)
wall = time.time() - started
(log_path,) = log_dir.glob("*.json")
data = json.loads(log_path.read_text())
sample = data["samples"][0]
(actions,) = log_dir.glob("actions/*/*.jsonl")
try:
    import torch

    gpu = torch.cuda.get_device_name(0)
except Exception:
    gpu = None
summary = {
    "plan": plan,
    "policy": "yam_scripted (the task's demo(), DEMO_PLAN=%s)" % plan,
    "instruction": task.INSTRUCTION,
    "status": data["status"],
    "task_score": data["results"]["metrics"]["task_score"],
    "termination": sample["termination_reasons"][0],
    "steps": sum(1 for _ in actions.open()) - 1,
    "max_steps": task.MAX_STEPS,
    "seed": "none (fixed layout, object_jitter=0)",
    "final": json.loads((log_dir / "trace.jsonl").read_text().splitlines()[-1]),
    "versions": _versions(),
    "gpu": gpu,
    "wall_clock_s": round(wall, 1),
    "source_sha256": _source_hash(),
    "video_sha256": _sha256(log_dir / f"{plan}.mp4") if video and (log_dir / f"{plan}.mp4").exists() else None,
}
(log_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps({k: summary[k] for k in ("plan", "task_score", "termination", "steps", "final")}), flush=True)
sys.exit(code)
