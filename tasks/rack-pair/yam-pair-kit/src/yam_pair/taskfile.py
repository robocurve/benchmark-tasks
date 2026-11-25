"""Load a yam_pair task file. Pure Python: loading one needs no Isaac Sim (see CONTRACT.md §2)."""

from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from yam_arena.task_file import SCORE_OVER, SUCCESS_HOLD_S, Asset, Box, Cylinder, Sphere

from yam_pair.layout import STATIONS, Layout
from yam_pair.task import Shape, UsdBody

PHYSICS_HZ = 240.0
HOOKS = ("success", "distance", "score", "report", "demo", "isaac_objects", "on_build", "before_reset", "on_reset",
         "on_step")
SCORING_HOOKS = ("success", "distance", "score", "report")


def reserved_names() -> frozenset[str]:
    names = {"floor", "table", "light", "dome_light", "lamp_a", "lamp_b", "overview_cam", "ground_plane"}
    for s in STATIONS:
        names |= {f"{s}_{side}_robot" for side in ("left", "right")}
        names |= {f"{s}_{side}_contacts" for side in ("left", "right")}
        names |= {f"{s}_{cam}_cam" for cam in ("top", "left", "right")}
        names |= {f"{s}_stand_{part}" for part in ("foot", "mast", "arm", "housing")}
    return frozenset(names)


RESERVED_NAMES = reserved_names()


def _arity(fn: Callable) -> int:
    """How many positional arguments a hook takes (1 or 2 for scoring hooks, 0 or 1 for demo/isaac_objects)."""
    try:
        params = [p for p in inspect.signature(fn).parameters.values()
                  if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD) and p.default is p.empty]
        var = any(p.kind == p.VAR_POSITIONAL for p in inspect.signature(fn).parameters.values())
    except (TypeError, ValueError):
        return 1
    return 99 if var else len(params)


@dataclass(frozen=True)
class PairTask:
    path: Path
    name: str
    sha256: str
    objects: tuple
    layout: Layout
    table: dict | None
    physics_hz: float
    instructions: dict[str, str]
    docs: str | None
    hooks: dict[str, Callable | None]
    success_hold_s: float = SUCCESS_HOLD_S
    score_over: str = "end"
    meta: dict = field(default_factory=dict)  # task.json
    module: Any = field(default=None, repr=False, compare=False)

    @property
    def folder(self) -> Path:
        return self.path.parent

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(o.name for o in self.objects)

    @property
    def free_objects(self) -> tuple[str, ...]:
        return tuple(o.name for o in self.objects if getattr(o, "mass", None) is not None)

    def call(self, hook: str, objects, pair):
        """Call a scoring hook with (objects) or (objects, pair), as it declares."""
        fn = self.hooks.get(hook)
        if fn is None:
            return None
        return fn(objects, pair) if _arity(fn) >= 2 else fn(objects)

    def call_layout(self, hook: str, layout: Layout):
        fn = self.hooks.get(hook)
        if fn is None:
            return None
        return fn(layout) if _arity(fn) >= 1 else fn()


def load(path: str | Path) -> PairTask:
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"task file not found: {path}")
    source = path.read_bytes()
    folder = str(path.parent)
    if folder not in sys.path:
        sys.path.insert(0, folder)
    spec = importlib.util.spec_from_file_location(f"yam_pair_task_{path.stem}", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot import {path} as a Python module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    meta = {}
    meta_path = path.with_name("task.json")
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text())

    objects = tuple(getattr(module, "OBJECTS", ()))
    has_isaac = callable(getattr(module, "isaac_objects", None))
    if not objects and not has_isaac:
        raise ValueError(f"{path.name}: OBJECTS (or isaac_objects()) must define at least one object")
    for obj in objects:
        if not isinstance(obj, Box | Cylinder | Sphere | Asset):
            raise TypeError(f"{path.name}: OBJECTS holds {obj!r}; use Box, Cylinder, Sphere, Asset, UsdBody or Shape")
        obj._check()
        usd = getattr(obj, "usd", None)
        if usd and not (path.parent / usd).is_file() and type(obj).__module__.startswith("yam_pair"):
            raise FileNotFoundError(f"{path.name}: {obj.name}: {path.parent / usd} not found")
    names = [o.name for o in objects]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise ValueError(f"{path.name}: object names must be unique, repeated: {duplicates}")
    clashes = sorted(set(names) & RESERVED_NAMES)
    if clashes:
        raise ValueError(f"{path.name}: {clashes} already name part of the room or robots; rename them")

    layout = getattr(module, "LAYOUT", None) or Layout()
    if isinstance(layout, dict):
        layout = Layout(**layout)
    if not isinstance(layout, Layout):
        raise TypeError(f"{path.name}: LAYOUT must be a yam_pair.task.Layout (or a dict of its fields)")

    table = getattr(module, "TABLE", {})
    if table is not None:
        table = dict(table)
        unknown = set(table) - {"size", "center", "color", "thickness"}
        if unknown:
            raise ValueError(f"{path.name}: TABLE keys are size, center, color, thickness; got {sorted(unknown)}")

    physics_hz = float(getattr(module, "PHYSICS_HZ", PHYSICS_HZ))
    if not (math.isfinite(physics_hz) and physics_hz > 0):
        raise ValueError(f"{path.name}: PHYSICS_HZ must be positive")

    instructions = dict(meta.get("instructions") or {})
    instructions.update(dict(getattr(module, "INSTRUCTIONS", None) or {}))
    bad = set(instructions) - set(STATIONS)
    if bad:
        raise ValueError(f"{path.name}: INSTRUCTIONS / task.json instructions keys are 'a' and 'b', got {sorted(bad)}")

    hooks = {}
    if getattr(module, "metrics", None) is not None:
        raise ValueError(f"{path.name}: metrics() is not read; use report(objects, pair) for extra numbers")
    for hook in HOOKS:
        fn = getattr(module, hook, None)
        if fn is not None and not callable(fn):
            raise TypeError(f"{path.name}: {hook} must be a function")
        hooks[hook] = fn
    for hook in SCORING_HOOKS:
        if hooks[hook] is not None and _arity(hooks[hook]) not in (1, 2, 99):
            raise TypeError(f"{path.name}: {hook}() takes (objects) or (objects, pair)")
    hold = float(getattr(module, "SUCCESS_HOLD_S", SUCCESS_HOLD_S))
    if not hold >= 0:
        raise ValueError(f"{path.name}: SUCCESS_HOLD_S must be >= 0")
    score_over = getattr(module, "SCORE_OVER", "end")
    if score_over not in SCORE_OVER:
        raise ValueError(f"{path.name}: SCORE_OVER must be one of {SCORE_OVER}, got {score_over!r}")
    return PairTask(
        path=path,
        name=path.stem if path.stem != "task" else path.parent.name,
        sha256=hashlib.sha256(source).hexdigest(),
        objects=objects,
        layout=layout,
        table=table,
        physics_hz=physics_hz,
        instructions=instructions,
        docs=(module.__doc__ or "").strip() or None,
        hooks=hooks,
        success_hold_s=hold,
        score_over=score_over,
        meta=meta,
        module=module,
    )


# The task each yam_pair embodiment loaded (and the live embodiment), by environment_id, so the pair policies (which
# Inspect Robots binds to that embodiment's info) can read the same task's instructions and demo.
_LOADED: dict[str, Any] = {}


def register(environment_id: str, task: PairTask, embodiment: Any = None) -> None:
    _LOADED[environment_id] = (task, embodiment)


def lookup(environment_id: str | None) -> tuple[PairTask | None, Any]:
    return _LOADED.get(environment_id or "", (None, None))


def resolve_instructions(task: PairTask | None, scene_instruction: str | None, instruction_a: str | None,
                         instruction_b: str | None) -> dict[str, str]:
    """Each station's own instruction (CONTRACT.md §3)."""
    from_task = task.instructions if task is not None else {}
    if instruction_a:
        a = instruction_a
    elif instruction_b and scene_instruction:
        a = scene_instruction
    else:
        a = from_task.get("a") or scene_instruction
    b = instruction_b or from_task.get("b")
    missing = [s for s, v in (("A", a), ("B", b)) if not v]
    if missing:
        raise ValueError(
            f"no instruction for station {' and '.join(missing)}: give -P instruction_a=/-P instruction_b=, or put "
            "\"instructions\": {\"a\": ..., \"b\": ...} in task.json (or INSTRUCTIONS in task.py)")
    return {"a": a, "b": b}
