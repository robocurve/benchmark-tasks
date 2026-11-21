"""ourkit.run_record: the GPU run trace. On the simulator machine, if ``~/runs`` exists, a task's hooks append a line
every half second to ``~/runs/<task file stem>-trace.txt`` (collect.sh copies it back); elsewhere nothing happens and
nothing in the trial changes. A trace line never ends a trial.

    REC = RunRecord(__file__, "shade")
    @on_reset
    def _reset(states): REC.reset()
    @on_step
    def _watch(states): REC.tick(states, lambda: f"bar {...} score {score(states):g}")

``reset`` writes each joint's physics as the simulator holds it (stiffness, damping, friction, target, limits): the
line that shows whether the kit's gains and friction reached PhysX.
"""

from __future__ import annotations

import pathlib

import numpy as np

from . import scene

RUN_DIR = pathlib.Path.home() / "runs"
EVERY = 10  # control steps between lines: twice a second at 20 Hz


class RunRecord:
    def __init__(self, task_file: str, *mechs: str, every: int = EVERY, folder: pathlib.Path | None = None):
        self.stem = pathlib.Path(task_file).stem
        self.mechs = mechs
        self.every = every
        self.folder = folder or RUN_DIR
        self.steps = 0

    @property
    def on(self) -> bool:
        return self.folder.is_dir()

    def write(self, line: str) -> None:
        if self.on:
            with open(self.folder / f"{self.stem}-trace.txt", "a") as f:
                f.write(line + "\n")

    def reset(self, extra: str = "") -> None:
        self.steps = 0
        if self.on:
            self.write(f"reset: {joint_physics(*self.mechs)}" + (f" | {extra}" if extra else ""))

    def tick(self, states, line) -> None:
        """After a control step: every ``every`` steps, ``line()`` (a function, so nothing is built off the GPU)."""
        self.steps += 1
        if self.steps % self.every or not self.on:
            return
        try:
            body = line() if callable(line) else str(line)
            self.write(f"{self.steps / 20.0:.1f}s {body} | {events_line()}")
        except Exception as err:  # noqa: BLE001  (the record is diagnostic)
            self.write(f"{self.steps / 20.0:.1f}s trace failed: {type(err).__name__}: {err}")


def _array(x) -> np.ndarray:
    try:
        return scene._np(x)
    except Exception:  # noqa: BLE001  (a warp array without a torch view)
        return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=float)


def joint_physics(*mechs: str) -> str:
    """Each joint's stiffness, damping, friction, target and limits as the simulator holds them."""
    e = scene.env()
    if e is None:
        return "no simulator"
    out = []
    for name in mechs or tuple(scene.MECHS):
        try:
            asset = e.unwrapped.scene[name]
            names = list(asset.joint_names)
            for j in scene.MECHS[name].joints:
                i = names.index(j.joint_name)
                vals = []
                for attr in ("joint_stiffness", "joint_damping", "joint_friction_coeff", "joint_dynamic_friction_coeff",
                             "joint_pos_target", "joint_pos_limits"):
                    v = getattr(asset.data, attr, None)
                    if v is not None:
                        vals.append(f"{attr.removeprefix('joint_')} {np.round(_array(v)[0][i], 4).tolist()}")
                out.append(f"{name}.{j.name}: " + ", ".join(vals))
        except Exception as err:  # noqa: BLE001
            out.append(f"{name}: unreadable ({type(err).__name__}: {err})")
    return " | ".join(out)


def events_line(n: int = 3) -> str:
    """The last ``n`` rule events of every mech."""
    ev = sorted((e for rs in scene._RULES.values() for e in rs.events()), key=lambda e: e[1])[-n:]
    return " ".join(f"{e[4]}:{e[0]}@{e[1]:.2f}:q{e[2]},v{e[3]}" for e in ev)
