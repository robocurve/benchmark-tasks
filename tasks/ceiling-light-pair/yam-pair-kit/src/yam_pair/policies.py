"""Inspect Robots policies for ``yam_pair``: ``dream_pair`` and ``yam_pair_scripted``.

``dream_pair``: two independent Dream-1 sessions, one per station, each with its own client, frame window and
instruction, asked concurrently every control step::

    inspect-robots run --embodiment yam_pair -E task=task.py --policy dream_pair --instruction "$INSTR" ...
        [-P instruction_a=...] [-P instruction_b=...] [-P only=a|b] [-P rows_per_request=48] [-P model=...]

``yam_pair_scripted``: the task file's ``demo(layout)`` for both stations (``-P hold_s=3``).

Each station's policy sees only ``yam_pair.boundary.station_observation(...)`` of its own station.
"""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.spaces import CameraSpec, ObservationSpace
from inspect_robots.types import Action, ActionChunk, Observation

from yam_pair import boundary, taskfile
from yam_pair.layout import STATIONS

DEFAULT_HZ = 20.0
# Dream evicts a session that sits idle for roughly 8-10 s and restarts its episode memory (DreamSessionExpired, which
# inspect_robots_dream answers by asking again on the new session). A four-arm step takes ~0.13-0.23 s, so Dream's
# default 48 rows per request leaves each session idle ~10 s between requests (measured: 11-13 s between request
# steps, 3 of 8 requests expired). 24 rows keeps the gap near 5 s.
DEFAULT_ROWS_PER_REQUEST = 24


def _pair_info(name: str, cam_h: int = 176, cam_w: int = 320, checkpoint: str | None = None) -> PolicyInfo:
    cams = tuple(CameraSpec(boundary.camera_name(s, c), cam_h, cam_w) for s in STATIONS for c in boundary.ARM_CAMERAS)
    return PolicyInfo(
        name=name,
        action_space=boundary.pair_action_box(),
        observation_space=ObservationSpace(cameras=cams, state_keys=frozenset(boundary.state_key(s) for s in STATIONS)),
        control_hz=DEFAULT_HZ,
        checkpoint=checkpoint,
    )


class _PairBase(PolicyBase):
    """Shared: binds to the yam_pair task, resolves each station's instruction."""

    def __init__(self, name: str, instruction_a: str | None, instruction_b: str | None, checkpoint=None):
        self.info = _pair_info(name, checkpoint=checkpoint)
        self.instruction_a, self.instruction_b = instruction_a, instruction_b
        self._task: taskfile.PairTask | None = None
        self._embodiment: Any = None
        self._pair_info: Any = None
        self._hz = DEFAULT_HZ
        self.instructions: dict[str, str] = {}

    def bind(self, embodiment_info: Any) -> None:
        if embodiment_info.name != "yam_pair":
            raise ValueError(f"{self.info.name} drives the yam_pair embodiment, not {embodiment_info.name!r}")
        self._pair_info = embodiment_info
        self._hz = float(embodiment_info.control_hz or DEFAULT_HZ)
        self._task, self._embodiment = taskfile.lookup(embodiment_info.environment_id)

    def on_trial_start(self, scene_id: str, epoch: int, log_dir: str, run_id: str) -> None:
        self._trial_t0 = time.monotonic()
        self._act_ticks: list[float] = []

    def _tick(self) -> None:
        if hasattr(self, "_act_ticks"):
            self._act_ticks.append(time.monotonic())

    def on_trial_end(self, record: Any, log_dir: str, run_id: str) -> None:
        """Put the trial's yam_pair summary (score, cross-station contact, report, step wall time) into
        ``record.metadata["yam_pair"]`` (Inspect keeps it in the eval log) and its per-step series in
        ``<log_dir>/yam_pair/<run_id>_<scene>-e<epoch>.json``."""
        import json
        from pathlib import Path

        from yam_pair.scoring import summarize

        summary, series = summarize(record)
        summary["instructions"] = dict(self.instructions)
        ticks = getattr(self, "_act_ticks", [])
        if len(ticks) > 2:
            gaps = np.diff(ticks)
            summary["step_wall_s"] = {"mean": float(gaps.mean()), "median": float(np.median(gaps)),
                                      "p95": float(np.percentile(gaps, 95)), "max": float(gaps.max())}
        emb = self._embodiment
        if emb is not None and getattr(emb, "timing", {}).get("steps"):
            summary["sim_step_s_mean"] = emb.timing["step_s"] / emb.timing["steps"]
        if emb is not None:
            summary["layout"] = emb.layout.describe()
        record.metadata["yam_pair"] = summary
        try:
            out = Path(log_dir) / "yam_pair"
            out.mkdir(parents=True, exist_ok=True)
            name = f"{run_id}_{record.scene_id}-e{record.epoch}.json"
            (out / name).write_text(json.dumps({"summary": summary, "steps": series}, default=float))
            record.metadata["yam_pair_series"] = f"yam_pair/{name}"
        except OSError:
            pass

    def _resolve(self, scene: Any) -> dict[str, str]:
        self.instructions = taskfile.resolve_instructions(self._task, scene.instruction, self.instruction_a,
                                                          self.instruction_b)
        return self.instructions


# -- Dream ------------------------------------------------------------------------------------------------------------
class DreamPairPolicy(_PairBase):
    """Two Dream-1 sessions (``inspect_robots_dream.DreamPolicy`` each), one per station."""

    def __init__(self, *, instruction_a: str | None = None, instruction_b: str | None = None, only: str | None = None,
                 session_factory: Any = None, **dream_kwargs: Any) -> None:
        from inspect_robots_dream import DreamPolicy

        model = dream_kwargs.get("model")
        super().__init__("dream_pair", instruction_a, instruction_b, checkpoint=model or "dreamlabs/dream-1-beta")
        dream_kwargs.setdefault("rows_per_request", DEFAULT_ROWS_PER_REQUEST)
        self.rows_per_request = dream_kwargs["rows_per_request"]
        if only not in (None, "", "a", "b"):
            raise ValueError(f"only must be 'a' or 'b', got {only!r}")
        self.only = only or None
        self.active = (self.only,) if self.only else STATIONS
        # One DreamPolicy per station: its own DreamSession (client, session id, frame window, action queue).
        self.policies = {}
        for s in self.active:
            kwargs = dict(dream_kwargs)
            if session_factory is not None:
                kwargs["session"] = session_factory(s)
            self.policies[s] = DreamPolicy(**kwargs)
        self._pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dream_pair")
        self._hold: dict[str, np.ndarray] = {}
        self._error: str | None = None
        self._wall: list[float] = []

    def bind(self, embodiment_info: Any) -> None:
        super().bind(embodiment_info)
        # Each station's policy is bound to a standard single-station YAM (14-D, three 320x176 cameras), concurrently
        # (each checks its own Dream key/model).
        self._run({s: (lambda s=s: self.policies[s].bind(boundary.station_info(embodiment_info, s)))
                   for s in self.active}, "bind")

    def reset(self, scene: Any) -> None:
        from inspect_robots import Scene

        inst = self._resolve(scene)
        self._hold, self._error, self._wall = {}, None, []
        calls = {}
        for s in self.active:
            sub = Scene(id=f"{scene.id}-{s}", instruction=inst[s], init_seed=getattr(scene, "init_seed", None))
            calls[s] = lambda s=s, sub=sub: self.policies[s].reset(sub)
        self._run(calls, "reset")

    def act(self, observation: Observation) -> ActionChunk:
        self._tick()
        views = {s: boundary.station_observation(observation, s, self.instructions[s]) for s in STATIONS}
        tick = time.monotonic()
        chunks = self._run({s: (lambda s=s: self.policies[s].act(views[s])) for s in self.active}, "act")
        self._wall.append(time.monotonic() - tick)
        rows, latency = {}, None
        for s in STATIONS:
            if s in chunks:
                rows[s] = np.asarray(chunks[s].actions[0].data, dtype=np.float64)
                lat = chunks[s].inference_latency_s
                if lat is not None:
                    latency = max(latency or 0.0, lat)
            else:  # the station without a session holds its first measured pose
                self._hold.setdefault(s, np.asarray(views[s].state["joint_pos"], dtype=np.float64).copy())
                rows[s] = self._hold[s]
        data = boundary.join_actions(rows["a"], rows["b"])
        return ActionChunk(actions=[Action(data=data)], control_hz=self._hz, inference_latency_s=latency)

    def _run(self, calls: dict, what: str) -> dict:
        """Run each station's call concurrently; wait for both, then raise if either failed (naming the station)."""
        futures = {s: self._pool.submit(fn) for s, fn in calls.items()}
        out, errors = {}, {}
        for s, fut in futures.items():
            try:
                out[s] = fut.result()
            except Exception as error:  # noqa: BLE001 - re-raised below with the station named
                errors[s] = error
        if errors:
            parts = [f"station {s.upper()} Dream session ({what}): {type(e).__name__}: {e}" for s, e in errors.items()]
            self._error = "; ".join(parts)
            first = next(iter(errors.values()))
            raise RuntimeError(self._error) from first
        return out

    def transcript(self) -> dict[str, Any]:
        out: dict[str, Any] = {"instructions": dict(self.instructions), "only": self.only, "error": self._error}
        for s, p in self.policies.items():
            try:
                out[s] = p.transcript()
            except Exception as error:  # noqa: BLE001
                out[s] = {"transcript_error": repr(error)}
        out["session_ids"] = {s: sorted({r.get("session_id") for r in out[s].get("requests", []) if r.get("session_id")})
                              for s in self.policies if isinstance(out.get(s), dict)}
        out["session_audit"] = self.session_audit(out)
        if self._wall:
            out["paired_act_wall_s"] = {"mean": float(np.mean(self._wall)), "max": float(np.max(self._wall)),
                                        "steps": len(self._wall)}
        return out

    def session_audit(self, transcript: dict | None = None) -> dict:
        """Per station: requests, session restarts (the session id changed between two requests: Dream restarted its
        episode memory), retried requests and round-trip times."""
        out = {}
        for s in self.policies:
            t = (transcript or {}).get(s) if transcript else None
            if t is None:
                try:
                    t = self.policies[s].transcript()
                except Exception:  # noqa: BLE001
                    continue
            reqs = t.get("requests", []) if isinstance(t, dict) else []
            ids = [r.get("session_id") for r in reqs]
            rtts = [r["rtt_s"] for r in reqs if r.get("rtt_s") is not None]
            out[s] = {"requests": len(reqs), "session_restarts": sum(1 for x, y in zip(ids, ids[1:]) if x != y),
                      "retried_requests": sum(1 for r in reqs if r.get("attempts", 1) > 1),
                      "rtt_s_mean": float(np.mean(rtts)) if rtts else None,
                      "rtt_s_max": float(np.max(rtts)) if rtts else None}
        return out

    def on_trial_end(self, record: Any, log_dir: str, run_id: str) -> None:
        super().on_trial_end(record, log_dir, run_id)
        audit = self.session_audit()
        meta = record.metadata.setdefault("yam_pair", {})
        meta["dream"] = {"rows_per_request": self.rows_per_request, "only": self.only, "sessions": audit,
                         "error": self._error}
        restarts = {s: a["session_restarts"] for s, a in audit.items() if a["session_restarts"]}
        if restarts:
            import sys

            print(f"dream_pair: Dream restarted a session mid-trial {restarts} (idle eviction); the model's episode "
                  "memory started over there. Lower -P rows_per_request or the step time.", file=sys.stderr)

    def close(self) -> None:
        for p in self.policies.values():
            try:
                p.close()
            except Exception:  # noqa: BLE001 - closing one session must not stop closing the other
                pass
        self._pool.shutdown(wait=True)


def dream_pair_policy(**kwargs: Any) -> DreamPairPolicy:
    """Policy ``dream_pair``: one Dream-1 session per station. ``-P instruction_a=`` / ``-P instruction_b=`` override
    the task's per-station instructions; ``-P only=a`` (or b) runs one session while the other station holds still;
    other ``-P`` (model, rows_per_request [default 24, not dream's 48], timeout_s, record, dry_run, preroll_steps) go
    to both sessions."""
    return DreamPairPolicy(**kwargs)


# -- scripted -----------------------------------------------------------------------------------------------------------
class ScriptedPairPolicy(_PairBase):
    """Plays the task file's ``demo(layout)`` for both stations; stops ``hold_s`` after both are done."""

    def __init__(self, hold_s: float = 3.0, instruction_a: str | None = None, instruction_b: str | None = None):
        super().__init__("yam_pair_scripted", instruction_a, instruction_b)
        self.hold_s = float(hold_s)
        self._controllers: dict = {}
        self._done_at: float | None = None

    def reset(self, scene: Any) -> None:
        from yam_pair import demo

        if self._task is None:
            raise ValueError("yam_pair_scripted needs the yam_pair embodiment's task file")
        if self._task.hooks.get("demo") is None:
            raise ValueError(f"{self._task.path.name} has no demo() for yam_pair_scripted to play")
        try:
            self._resolve(scene)
        except ValueError:
            self.instructions = {}
        layout = self._embodiment.layout if self._embodiment is not None else self._task.layout
        plan = dict(self._task.call_layout("demo", layout))
        unknown = sorted(set(plan) - set(STATIONS))
        if unknown:
            raise ValueError(f"demo() keys are stations {STATIONS}, got {unknown}")
        self._controllers = {s: demo.controller(plan.get(s), s, layout, self._hz) for s in STATIONS}
        self._layout = layout
        self._done_at = None

    def act(self, observation: Observation) -> ActionChunk:
        from yam_pair.demo import StationView

        self._tick()
        t = float(observation.state_time) + 1.0 / self._hz
        objects = self._embodiment.object_states if self._embodiment is not None else None
        rows = {}
        for s in STATIONS:
            state = np.asarray(observation.state[boundary.state_key(s)], dtype=np.float64)
            rows[s] = self._controllers[s].act(StationView(s, t, state, self._layout, objects))
        if self._done_at is None and all(c.finished for c in self._controllers.values()):
            self._done_at = t
        meta = {"request_stop": True} if self._done_at is not None and t > self._done_at + self.hold_s else {}
        return ActionChunk(actions=[Action(data=boundary.join_actions(rows["a"], rows["b"]), meta=meta)],
                           control_hz=self._hz)

    def transcript(self) -> dict[str, Any]:
        return {"instructions": dict(self.instructions), "done_at_s": self._done_at,
                "phases": {s: getattr(getattr(c, "obj", c), "phase", None) for s, c in self._controllers.items()}}


def yam_pair_scripted_policy(**kwargs: Any) -> ScriptedPairPolicy:
    """Policy ``yam_pair_scripted``; ``-P hold_s=3`` sets how long it holds after both stations finish."""
    return ScriptedPairPolicy(**kwargs)
