"""ourkit.testkit: the checks behind the kit's 7-test template (``test_template.py`` in the kit's folder).

1. text matches code: every number the task text states (``TEXT_NUMBERS`` in the task file) is in the text and
   equals the code's value;
2. blind text: no team, person, institution, username, file path, URL or PR number in task.json or the README;
3. json: the required fields, a valid category, a parsable run limit at least the demo's length;
4. cheats score low: each cheat mode's physcheck run (``CHEATS = {mode: max score}``) scores at most its cap;
5. the demo scores 1.0 in physcheck (a result newer than the task's code);
6. the mechanism holds its state with no command (``holdcheck``: arms at home, every joint and body still);
7. limits: title <= 55 characters, context <= 1,000 with "Done by:", fidelity <= 500.

Checks 4-6 need our pipeline (``.planning/pipeline`` above the task, or ``KIT_PIPELINE``); in a copy without it they
skip. Run the physcheck modes first with ``.planning/kit/kit_check.sh <task>``.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

REQUIRED = ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media")
CATEGORIES = ("Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance",
              "Societal relatability", "Other")
TITLE_MAX, CONTEXT_MAX, FIDELITY_MAX = 55, 1000, 500
BLIND = [r"/Users/", r"/home/\w+", r"~/", r"\bjasontang", r"\bghchris", r"github\.com", r"\bPR\s*#?\d+",
         r"#\d{2,}", r"[\w.+-]+@[\w-]+\.\w+", r"\bBlueDot\b", r"\bour team\b", r"\bT-\d+\b", r"\bClaude\b",
         r"\bAnthropic\b", r"\bDream ?labs?\b"]
UNITS = {"mm": 1e-3, "cm": 1e-2, "m": 1.0, "km": 1e3, "N": 1.0, "kN": 1e3, "kg": 1.0, "g": 1e-3, "s": 1.0,
         "ms": 1e-3, "min": 60.0, "deg": 1.0, "°": 1.0, "%": 0.01, "Hz": 1.0, "N/m": 1.0, "mm/s": 1e-3,
         "cm/s": 1e-2, "m/s": 1.0, "Nm": 1.0, "N m": 1.0, "in": 0.0254}


def task_file_of(task_dir: Path) -> Path:
    task_dir = Path(task_dir)
    py = task_dir / f"{task_dir.name.replace('-', '_')}.py"
    if not py.exists():
        raise FileNotFoundError(f"no {py.name} in {task_dir} (physcheck needs the task file named after its folder)")
    return py


def spec(task_dir: Path) -> dict:
    return json.loads((Path(task_dir) / "task.json").read_text())


def text_of(s: dict) -> str:
    return "\n".join(str(s.get(k, "")) for k in ("title", "instruction", "context", "setup", "scoring", "run_limit",
                                                  "reset_notes", "fidelity"))


# -- 1. text matches code -------------------------------------------------------------------------------------------

_Q = re.compile(r"(-?\d[\d,]*(?:\.\d+)?)\s*(mm/s|cm/s|m/s|N/m|N m|Nm|kN|mm|cm|km|kg|ms|min|Hz|deg|°|%|m|N|g|s|in)\b|"
                r"(-?\d[\d,]*(?:\.\d+)?)\s*(°|%)")


def quantity(snippet: str) -> float:
    """The first number with a unit in ``snippet``, in SI (degrees stay degrees): "6.2 cm" -> 0.062."""
    m = _Q.search(snippet)
    if not m:
        raise ValueError(f"no number with a unit in {snippet!r}")
    num, unit = (m.group(1), m.group(2)) if m.group(1) else (m.group(3), m.group(4))
    return float(num.replace(",", "")) * UNITS[unit]


def check_text_numbers(s: dict, pairs) -> list[str]:
    """``pairs``: (snippet as the text states it, the code's value in SI[, relative tolerance]). Problems found."""
    text = text_of(s)
    bad = []
    for p in pairs:
        snippet, value = p[0], float(p[1])
        tol = p[2] if len(p) > 2 else 0.005
        if snippet not in text:
            bad.append(f"{snippet!r} is not in the task text")
            continue
        got = quantity(snippet)
        if abs(got - value) > tol * max(abs(value), 1e-9) + 1e-12:
            bad.append(f"{snippet!r} says {got:g} but the code has {value:g}")
    return bad


# -- 2. blind text --------------------------------------------------------------------------------------------------


def check_blind(s: dict, readme: str = "", extra=()) -> list[str]:
    text = text_of(s) + "\n" + readme
    return [f"{pat!r} matches {m.group(0)!r}" for pat in (*BLIND, *extra) for m in [re.search(pat, text, re.I)] if m]


# -- 3. json --------------------------------------------------------------------------------------------------------


def run_limit_steps(s: dict) -> int:
    m = re.match(r"\s*([\d,]+)\s*(steps|model calls|calls)", str(s.get("run_limit", "")))
    if not m:
        raise ValueError(f"run_limit {s.get('run_limit')!r}: start with '<n> steps (...)'")
    return int(m.group(1).replace(",", ""))


def demo_steps(module, hz: float = 20.0) -> int:
    plan = module.demo()
    return int(round(max(sum(w.duration for w in ws) for ws in plan.values()) * hz))


def check_json(s: dict, module=None) -> list[str]:
    bad = [f"missing {k}" for k in REQUIRED if not str(s.get(k, "")).strip()]
    if s.get("category") not in CATEGORIES:
        bad.append(f"category {s.get('category')!r} not in {CATEGORIES}")
    try:
        limit = run_limit_steps(s)
        if module is not None and getattr(module, "demo", None) is not None:
            need = demo_steps(module)
            if limit < need:
                bad.append(f"run_limit {limit} steps is shorter than the demo's {need}")
    except ValueError as err:
        bad.append(str(err))
    if not isinstance(s.get("keywords", []), list):
        bad.append("keywords must be a list")
    return bad


# -- 7. limits ------------------------------------------------------------------------------------------------------


def check_limits(s: dict) -> list[str]:
    bad = []
    if len(s.get("title", "")) > TITLE_MAX:
        bad.append(f"title is {len(s['title'])} characters (max {TITLE_MAX})")
    ctx = s.get("context", "")
    if len(ctx) > CONTEXT_MAX:
        bad.append(f"context is {len(ctx)} characters (max {CONTEXT_MAX})")
    if "Done by:" not in ctx:
        bad.append('context has no "Done by:"')
    if len(s.get("fidelity", "")) > FIDELITY_MAX:
        bad.append(f"fidelity is {len(s['fidelity'])} characters (max {FIDELITY_MAX})")
    return bad


# -- 4-6. physcheck -------------------------------------------------------------------------------------------------


def pipeline_dir(task_dir: Path) -> Path | None:
    env = os.environ.get("KIT_PIPELINE")
    if env and (Path(env) / "physcheck.py").exists():
        return Path(env)
    for d in Path(task_dir).resolve().parents:
        p = d / ".planning" / "pipeline"
        if (p / "physcheck.py").exists():
            return p
    return None


def physcheck_out(task_dir: Path, mode: str) -> Path | None:
    pipe = pipeline_dir(task_dir)
    return None if pipe is None else pipe.parent / "physcheck" / Path(task_dir).name / f"kit-{mode}"


def newest_source(task_dir: Path) -> float:
    """The newest modification time of the task's code and json (and its kit copy)."""
    task_dir = Path(task_dir)
    files = [*task_dir.glob("*.py"), task_dir / "task.json", *task_dir.glob("ourkit/*.py")]
    return max((f.stat().st_mtime for f in files if f.exists() and not f.name.startswith("test_")), default=0.0)


def physcheck_result(task_dir: Path, mode: str) -> tuple[dict | None, str]:
    """(result.json of the kit-<mode> physcheck run, a problem or ""). None if there is no pipeline or no run."""
    out = physcheck_out(task_dir, mode)
    if out is None:
        return None, "no pipeline"
    f = out / "result.json"
    if not f.exists():
        return None, f"no physcheck run for mode {mode!r}: run .planning/kit/kit_check.sh {task_dir} {mode}"
    r = json.loads(f.read_text())
    if f.stat().st_mtime < newest_source(task_dir):
        return r, f"the {mode!r} physcheck run is older than the task's code: rerun it"
    return r, ""


def run_hold(task_dir: Path, seconds: float = 5.0, rigid: bool = True, timeout: float = 600.0) -> dict | None:
    """Run ``holdcheck`` in a fresh interpreter at low priority: the arms stay home ``seconds`` after the reset and
    every joint and mech body is watched. None without the pipeline."""
    pipe = pipeline_dir(task_dir)
    if pipe is None:
        return None
    here = Path(__file__).resolve().parent
    cmd = ["nice", "-n", "15", sys.executable, str(here / "holdcheck.py"), str(Path(task_dir).resolve()),
           "--seconds", str(seconds), "--pipeline", str(pipe)] + ([] if rigid else ["--soft"])
    envv = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
            "KIT_DEMO": "none"}
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=envv)
    if p.returncode != 0:
        raise RuntimeError(f"holdcheck failed:\n{p.stdout[-2000:]}\n{p.stderr[-4000:]}")
    return json.loads(p.stdout.strip().splitlines()[-1])
