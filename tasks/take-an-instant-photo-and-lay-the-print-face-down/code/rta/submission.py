"""Build a Robocurve `benchmark-tasks` submission folder for one task.

    python -m rta.submission tasks/t01_box_retrieval --demo-log outputs/t01/demo/<log>.json --out submission/

Produces ``<out>/<task-name>/`` with ``task.json`` (from the task folder's ``meta.json`` + the scorer's scores),
``README.md`` (the task README plus how to run, score and reproduce), the code (``rta/``, the task file and its
``assets/`` models if it has any, pinned dependency notes), ``media/demo.mp4`` (H.264, the three cameras tiled), ``media/setup.png`` (the first
frame of the top camera) and, if given, ``media/model_run.mp4`` (the Dream-1 run). Media are uploaded
separately with the program's ``tools/upload_media.py``; nothing in the text identifies the team.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from rta import PACKAGE_DIR, spec

ROOT = PACKAGE_DIR.parent
TRACKED = ["rta", "pyproject.toml"]


def _frames_dir(log: Path) -> Path:
    data = json.loads(log.read_text())
    frames = data.get("stats", {}).get("frames_dir")
    if not frames:
        raise SystemExit(f"{log}: no frames_dir in stats (run with --store-frames)")
    return Path(frames)


def _scene_prefix(frames: Path, log: Path | None = None, require_success: bool = False) -> str:
    """The video prefix of the trial to show: the first trial that scored full success when ``log`` is given
    (the program wants a full-score demo), else the first trial on disk."""
    mp4 = sorted(frames.glob("*_top_cam.mp4"))
    if not mp4:
        raise SystemExit(f"{frames}: no *_top_cam.mp4 (render with `inspect-robots video <log>` first)")
    if log is not None:
        data = json.loads(log.read_text())
        winners = [s["scene_id"] for s in data.get("samples", []) if s.get("reduced", {}).get("rta_success") == 1.0]
        for scene in winners:
            for m in mp4:
                if m.name.startswith(f"{scene}-"):
                    return m.name[: -len("_top_cam.mp4")]
        if require_success:
            raise SystemExit(f"{log}: no trial with rta_success 1.0 — the demo must be a full-score run")
    return mp4[0].name[: -len("_top_cam.mp4")]


def tile_video(frames: Path, out: Path, log: Path | None = None, require_success: bool = False) -> None:
    """Top camera large on the left, the two wrist cameras stacked on the right; H.264 yuv420p."""
    prefix = _scene_prefix(frames, log, require_success)
    top, left, right = (frames / f"{prefix}_{c}.mp4" for c in ("top_cam", "left_cam", "right_cam"))
    out.parent.mkdir(parents=True, exist_ok=True)
    filt = (
        "[0:v]scale=-2:720:flags=lanczos[t];"
        "[1:v]scale=-2:360:flags=lanczos[l];[2:v]scale=-2:360:flags=lanczos[r];"
        "[l][r]vstack=inputs=2[w];[t][w]hstack=inputs=2,pad=ceil(iw/2)*2:ceil(ih/2)*2[v]"
    )
    cmd = ["ffmpeg", "-loglevel", "error", "-y", "-i", str(top), "-i", str(left), "-i", str(right), "-filter_complex", filt,
           "-map", "[v]", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-crf", "20", str(out)]
    subprocess.run(cmd, check=True)


def first_frame(frames: Path, out: Path, log: Path | None = None) -> None:
    prefix = _scene_prefix(frames, log)
    top = frames / f"{prefix}_top_cam.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(top), "-vf", "select=eq(n\\,0)", "-vframes", "1", str(out)], check=True)


def _score_text(v: float) -> str:
    """0.25 -> '0.25', 1 -> '1.0', 0 -> '0.0' (the program's example reads 'Dream-1 run: 0.25; demo: 1.0')."""
    text = f"{v:.2f}".rstrip("0")
    return text + "0" if text.endswith(".") else text


def demo_trial_score(log: Path) -> float:
    """The q_score of the trial shown as the demo: the first full-success trial if there is one (the same rule
    ``_scene_prefix`` uses for the video), else the run's mean."""
    data = json.loads(log.read_text())
    for s in data.get("samples", []):
        if s.get("reduced", {}).get("rta_success") == 1.0:
            return float(s["reduced"].get("q_score", 1.0))
    return scores_from_log(log).get("q_score", 0.0)


def scores_from_log(log: Path) -> dict[str, float]:
    data = json.loads(log.read_text())
    return {k: float(v) for k, v in (data.get("results", {}).get("metrics") or {}).items()}


def _git_head() -> str | None:
    """Short hash of the checkout's HEAD, read from .git directly (the container has no usable git for a bind mount)."""
    git = ROOT / ".git"
    try:
        if git.is_file():  # worktree: "gitdir: <path>"
            git = Path(git.read_text().split(":", 1)[1].strip())
        head = (git / "HEAD").read_text().strip()
        if head.startswith("ref: "):
            ref = head[5:]
            ref_file = git / ref
            if not ref_file.is_file():  # packed refs; a worktree's commondir holds them
                common = git / "commondir"
                base = (git / common.read_text().strip()).resolve() if common.is_file() else git
                ref_file = base / ref
                if not ref_file.is_file():
                    for line in (base / "packed-refs").read_text().splitlines():
                        if line.endswith(" " + ref):
                            return line.split()[0][:7]
            head = ref_file.read_text().strip()
        return head[:7]
    except (OSError, IndexError):
        return None


def build(task_dir: Path, out_root: Path, demo_log: Path | None, model_log: Path | None, name: str | None) -> Path:
    """Assemble ``<out_root>/<name>/`` (task.json, README, code/, media/) and write ``<out_root>/<name>.provenance.json``
    beside it (which logs and commit it came from; the PR folder itself stays as the format prescribes)."""
    task = spec.load(task_dir / "task.py")
    meta_path = task_dir / "meta.json"
    if not meta_path.is_file():
        raise SystemExit(f"{meta_path} is missing (title, category, keywords, context, setup, scoring, run_limit, fidelity)")
    meta = json.loads(meta_path.read_text())
    name = name or meta.get("folder") or task_dir.name.split("_", 1)[-1].replace("_", "-")
    dest = out_root / name
    if dest.exists():
        shutil.rmtree(dest)
    (dest / "media").mkdir(parents=True)
    # code: the framework, the task file, and the pins
    for item in TRACKED:
        src = ROOT / item
        if src.is_dir():
            shutil.copytree(src, dest / "code" / item, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        else:
            shutil.copy2(src, dest / "code" / item)
    (dest / "code" / "task").mkdir()
    shutil.copy2(task_dir / "task.py", dest / "code" / "task" / "task.py")
    # the task where the registered name and rta.local_assets look for it (TASKS_DIR/<folder>/), with its models
    # (USD, textures, layout) and the model sizes rta.assets reads
    in_tree = dest / "code" / "tasks" / task_dir.name
    in_tree.mkdir(parents=True)
    shutil.copy2(task_dir / "task.py", in_tree / "task.py")
    if (task_dir / "assets").is_dir():
        shutil.copytree(task_dir / "assets", in_tree / "assets", ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        (dest / "code" / "docs" / "assets").mkdir(parents=True)
        shutil.copy2(ROOT / "docs" / "assets" / "bounds.json", dest / "code" / "docs" / "assets" / "bounds.json")
    shutil.copy2(ROOT / "docker" / "Dockerfile", dest / "code" / "Dockerfile")
    # scores
    scores = {}
    if model_log is not None:
        scores["Dream-1 run"] = scores_from_log(model_log).get("q_score", 0.0)
    if demo_log is not None:
        scores["demo"] = demo_trial_score(demo_log)
    spec_json = {
        "title": meta["title"],
        "category": meta["category"],
        "keywords": meta.get("keywords", []),
        "instruction": task.instruction,
        "context": meta.get("context", ""),
        "setup": meta["setup"],
        "scoring": meta["scoring"],
        "run_limit": meta["run_limit"],
        "reset_notes": meta.get("reset_notes", ""),
        "fidelity": meta["fidelity"],
        "scores": "; ".join(f"{k}: {_score_text(v)}" for k, v in scores.items()) or "demo: pending",
    }
    if meta.get("media"):
        spec_json["media"] = meta["media"]
    (dest / "task.json").write_text(json.dumps(spec_json, indent=2, ensure_ascii=False) + "\n")
    # README: the task's README + reproduction
    readme = (task_dir / "README.md").read_text()
    readme += (
        "\n\n## Reproduce\n\n"
        "Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`\n"
        "builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:\n\n"
        "```bash\n"
        "uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code\n"
        f"inspect-robots run --task {meta['registered']} -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo\n"
        f"inspect-robots run --task {meta['registered']} -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1\n"
        "inspect-robots view out/demo\n"
        "```\n\n"
        "Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one\n"
        "`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event\n"
        "log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden\n"
        "choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: about 30 s of simulated time\n"
        "at 10 Hz control; roughly 3 minutes of wall time on an RTX 4090 after a one-time 2-minute simulator start.\n"
    )
    (dest / "README.md").write_text(readme)
    # media
    if demo_log is not None:
        frames = _frames_dir(demo_log)
        tile_video(frames, dest / "media" / "demo.mp4", demo_log, require_success=True)
        first_frame(frames, dest / "media" / "setup.png", demo_log)
    if model_log is not None:
        tile_video(_frames_dir(model_log), dest / "media" / "model_run.mp4")
    provenance = {
        "task_dir": str(task_dir.relative_to(ROOT)) if task_dir.is_relative_to(ROOT) else str(task_dir),
        "demo_log": str(demo_log) if demo_log is not None else None,
        "model_log": str(model_log) if model_log is not None else None,
        "commit": _git_head(),
        "built": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    (out_root / f"{name}.provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return dest


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task_dir", type=Path)
    ap.add_argument("--demo-log", type=Path, help="Inspect Robots eval log of the full-score demo (frames stored, videos rendered)")
    ap.add_argument("--model-log", type=Path, help="eval log of the Dream-1 run")
    ap.add_argument("--out", type=Path, default=ROOT / "submission")
    ap.add_argument("--name", help="folder name under tasks/ (default from meta.json or the task dir)")
    a = ap.parse_args(argv)
    dest = build(a.task_dir, a.out, a.demo_log, a.model_log, a.name)
    print(f"built {dest}")
    for p in sorted(dest.rglob("*")):
        if p.is_file():
            print(f"  {p.relative_to(dest)}  {p.stat().st_size / 1e6:.1f} MB" if p.suffix in (".mp4", ".png") else f"  {p.relative_to(dest)}")


if __name__ == "__main__":
    main(sys.argv[1:])
