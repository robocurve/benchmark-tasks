#!/usr/bin/env python3
"""PhysX trials for sink_stopper: the start (its pictures; it settles), the thread's catch where the nut stands, a crooked
start that never catches, turning back off the start, the nut turned on to snug (0.5), the rod pressed down and let go
(1.0, with the stopper's drift over 30 s), and both falls (0).

    scripts/rta simpy <id> mech/tasks/sink_stopper/trials.py /workspace/robocurve/outputs/<id>

Only here are poses written by hand: the rod, stopper and nut are put in place with write_pose, the rod is held level
by writing it each step until the trial lets go, and the nut is pressed, turned or pressed down with push() forces and
torques the way fingers would. The arms stay at ready.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

PRESS_ON = 3.0  # N: the nut pressed onto the washer while it is set on the start (a fingertip's push)
PAIRS = {"nut_rod": ("pivot_nut", "pivot_rod"), "rod_drain": ("pivot_rod", "drain"), "rod_stopper": ("pivot_rod", "stopper"),
         "nut_drain": ("pivot_nut", "drain"), "stopper_drain": ("stopper", "drain")}


def ready_action():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


class Trial:
    def __init__(self, emb, name: str):
        from inspect_robots import Scene

        self.emb, self.m = emb, emb.task.module
        self.mk = self.m.mk
        emb.reset(Scene(id=name, instruction=self.m.INSTRUCTION), seed=0)
        self.t, self.rows, self.res, self.peaks = 0.0, [], None, dict.fromkeys(PAIRS, 0.0)
        self.hold_rod = True

    def seat(self, name: str, pos, rot=None) -> None:
        q = None if rot is None else tuple(float(v) for v in self.mk.matrix_to_quat(np.asarray(rot, dtype=float)))
        self.mk.write_pose(name, tuple(float(v) for v in pos), q)

    def rod_level(self) -> None:
        self.seat("pivot_rod", (P.X_DRAIN + P.SEAT_OFFSET_X, P.Y_DRAIN, self.m.TOP + P.PORT_Z), np.eye(3))

    def nut_on_start(self, gap: float, tilt_deg: float = 0.0) -> None:
        """The nut's front face ``gap`` from the thread start (negative: pressed on), on the port's axis, turned
        ``tilt_deg`` about the vertical."""
        rot = self.mk.axis_angle((0.0, 0.0, 1.0), math.radians(tilt_deg))
        self.seat("pivot_nut", (P.X_DRAIN + P.U_START + gap, P.Y_DRAIN, self.m.TOP + P.PORT_Z), rot)

    def reading(self) -> dict:
        m = self.m
        return {"t": round(self.t, 2), "score": float(self.res.info.get("score", 0.0)) if self.res is not None else None,
                "engaged": m.thread_engaged(), "cw_deg": round(m.thread_cw_deg(), 1), "land_gap_mm": round(1000.0 * m.land_gap(), 3),
                "nut_off_mm": round(1000.0 * m.nut_xy_off(), 3), "nut_tilt_deg": round(m.nut_tilt_deg(), 2),
                "gap_mm": round(1000.0 * m.stopper_gap(), 2), "pitch_deg": round(m.rod_pitch_deg(), 2),
                "ball_seat_mm": round(1000.0 * m.ball_seat_offset(), 2), "rod_in": m.rod_in_port(), "hold": m.STATE.get("hold", 0),
                "forces_N": {k: round(self.mk.contact_force(a, b), 2) for k, (a, b) in PAIRS.items()}}

    def step(self, seconds: float, load=None, every: int = 4, until=None) -> dict:
        """Step with the arms at ready; ``load(t)`` puts the trial's pushes on before each step; stops early once
        ``until()`` holds."""
        hz = float(self.emb.info.control_hz)
        act = ready_action()
        for i in range(int(round(seconds * hz))):
            if self.hold_rod:  # a hand holding the rod level: put level and its weight carried (it pivots on its ball)
                self.rod_level()
                self.mk.push("pivot_rod", force=(0.0, 0.0, P.ROD_MASS * P.GRAVITY))
            if load is not None:
                load(self.t)
            self.res = self.emb.step(act)
            self.t += 1.0 / hz
            r = self.reading()
            for k, v in r["forces_N"].items():
                self.peaks[k] = max(self.peaks[k], v)
            if i % every == 0:
                self.rows.append(r)
            if until is not None and until():
                self.rows.append(r)
                break
        return self.reading()

    def out(self, **kw) -> dict:
        return {**kw, "events": self.m.STATE.get("events", []), "catch": self.m.STATE.get("catch"),
                "peak_forces_N": {k: round(v, 2) for k, v in self.peaks.items()}, "end": self.reading(), "rows": self.rows}


def press_nut(trial: Trial, along: float = -PRESS_ON):
    """A hand's push on the nut toward the drain, carrying its weight (until the thread holds it)."""
    def load(_t: float) -> None:
        trial.mk.push("pivot_nut", force=(along, 0.0, P.NUT_MASS * P.GRAVITY))
    return load


def caught_at_start(emb, name: str) -> tuple[Trial, dict]:
    tr = Trial(emb, name)
    tr.rod_level()
    tr.nut_on_start(-0.00005)
    first = tr.step(0.05, load=press_nut(tr))
    return tr, {"first_step": first, "set_at_land_gap_mm": -0.05}


def trial_catch_where_it_stands(emb) -> dict:
    """Set square on the start, pressed 0.05 mm onto the washer: the thread catches at the first step, the nut does not
    move for it, and it stays caught with no load (0.25: the rod is in the port, the nut not snug)."""
    tr, info = caught_at_start(emb, "catch_where_it_stands")
    c = tr.m.STATE.get("catch") or {}
    after_catch = tr.step(1.0, load=lambda _t: tr.mk.push("pivot_nut"))
    ok = (info["first_step"]["engaged"] and abs(c.get("land_gap_mm", 9.0) + 0.05) <= 0.1 and after_catch["engaged"]
          and abs(after_catch["cw_deg"] - c.get("cw_deg", 99.0)) <= 3.0 and after_catch["score"] == 0.25)
    return tr.out(pass_=bool(ok), **info)


def trial_crooked_never_catches(emb) -> dict:
    """Held on the start and pressed on, but 3 deg and then 2 deg out of square: the thread never catches."""
    tr = Trial(emb, "crooked_never_catches")
    tr.rod_level()
    for tilt in (3.0, 2.0):
        def load(_t: float, tilt=tilt) -> None:
            tr.nut_on_start(-0.00005, tilt)
            press_nut(tr)(_t)
        tr.step(1.0, load=load)
    engaged_ever = any(r["engaged"] for r in tr.rows)
    return tr.out(pass_=bool(not engaged_ever and tr.reading()["score"] == 0.25))


def trial_turn_back_off_the_start(emb) -> dict:
    """Caught at the start, then turned back (0.06 N m the loosening way): the thread lets go at 3 deg back past the
    start; the nut stays on the rod behind the ball."""
    tr, info = caught_at_start(emb, "turn_back_off_the_start")
    tr.step(2.0, load=lambda _t: tr.mk.push("pivot_nut", torque=(0.06, 0.0, 0.0)), until=lambda: not tr.m.thread_engaged())
    released = not tr.m.thread_engaged()
    tr.step(0.5, load=lambda _t: tr.mk.push("pivot_nut"))
    ok = info["first_step"]["engaged"] and released and tr.m.STATE.get("zero") is None
    return tr.out(pass_=bool(ok), **info)


def turn_to_snug(tr: Trial) -> dict:
    """A turning-on torque on the nut that rises 0.15 N m/s from 0.02 N m (at most 0.40 N m) until the thread stops;
    returns the torque when it passed 340 deg (snug) and when it reached the stop."""
    seen: dict = {}

    def load(t: float) -> None:
        tau = min(0.02 + 0.15 * (t - t0), 0.40)
        cw = tr.m.thread_cw_deg()
        if cw >= P.STOP_DEG - P.SNUG_DEG and "snug_at_Nm" not in seen:
            seen["snug_at_Nm"] = round(tau, 3)
        if cw >= P.STOP_DEG - 2.0 and "stop_at_Nm" not in seen:
            seen["stop_at_Nm"] = round(tau, 3)
        tr.mk.push("pivot_nut", torque=(-tau, 0.0, 0.0))

    t0 = tr.t
    tr.step(4.0, load=load, until=lambda: "stop_at_Nm" in seen)
    tr.step(1.0, load=lambda _t: tr.mk.push("pivot_nut"))
    seen["nut_rod_force_at_snug_N"] = tr.reading()["forces_N"]["nut_rod"]
    return seen


def trial_turned_on_snug(emb) -> dict:
    """Caught at the start and turned on by a rising torque to the thread's stop: snug, 0.5 (stopper still closed)."""
    tr, info = caught_at_start(emb, "turned_on_snug")
    torque = turn_to_snug(tr)
    end = tr.reading()
    ok = end["engaged"] and end["cw_deg"] >= P.STOP_DEG - P.SNUG_DEG and end["score"] == 0.5
    return tr.out(pass_=bool(ok), torque=torque, **info)


def trial_pressed_open_and_let_go(emb) -> dict:
    """Snug, then the rod's rear pressed down (a torque rising 0.2 N m/s from 0.05 N m, as a hand on the clevis) until
    the rod is 20 deg down, then let go: the washer's friction on the ball holds it; the stopper's lift is read over
    30 s. 1.0 once held still for 1 s with the hands (at ready) clear."""
    tr, info = caught_at_start(emb, "pressed_open_and_let_go")
    torque = turn_to_snug(tr)
    tr.hold_rod = False
    t0 = tr.t
    tr.step(4.0, load=lambda t: tr.mk.push("pivot_rod", torque=(0.0, min(0.05 + 0.2 * (t - t0), 0.6), 0.0)),
            until=lambda: tr.m.rod_pitch_deg() >= 20.0)
    pressed = tr.reading()
    marks, drift = {}, []
    t_let = tr.t
    for upto in (1.0, 5.0, 10.0, 20.0, 30.0):
        tr.step(upto - (tr.t - t_let), load=lambda _t: tr.mk.push("pivot_rod"), every=20)
        r = tr.reading()
        marks[f"{upto:.0f}s"] = {"gap_mm": r["gap_mm"], "pitch_deg": r["pitch_deg"], "score": r["score"]}
        drift.append(r["gap_mm"])
    end = tr.reading()
    ok = (pressed["pitch_deg"] >= 19.5 and end["score"] == 1.0 and P.GAP_OPEN_MIN * 1000.0 <= end["gap_mm"] <= P.GAP_OPEN_MAX * 1000.0
          and max(drift) - min(drift) <= 1.0)
    return tr.out(pass_=bool(ok), torque=torque, pressed=pressed, after_let_go=marks, **info)


def trial_start(emb, out: Path) -> dict:
    """The start as reset: the three cameras' pictures, and 2 s at ready: the stopper, rod and nut settle within 2.5 mm
    of where they were built, nothing scores, nothing falls."""
    import cv2
    from inspect_robots import Scene

    m = emb.task.module
    obs = emb.reset(Scene(id="start", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / f"{stem}.png"), bgr)
        cv2.imwrite(str(out / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    res = None
    for _ in range(40):
        res = emb.step(ready_action())
    moved = {n: round(1000.0 * float(np.linalg.norm(m.mk.pose(n)[0] - np.asarray(m.mk.built_pose(n)[0]))), 2)
             for n in ("stopper", "pivot_rod", "pivot_nut")}
    score = float(res.info.get("score", 0.0)) if res is not None else None
    ok = score == 0.0 and m.STATE.get("zero") is None and max(moved.values()) <= 2.5 and not m.thread_engaged()
    return {"pass_": bool(ok), "moved_from_built_mm": moved, "score_after_2s": score, "nut_front_u_mm": round(1000.0 * m.nut_front_u(), 2),
            "nut_tilt_deg": round(m.nut_tilt_deg(), 2)}


def trial_falls(emb) -> dict:
    """The nut, then (fresh) the rod's ball, put on the bench: 0 for the whole trial."""
    res = {}
    for name, pos in (("pivot_nut", (0.36, 0.10, None)), ("pivot_rod", (0.38, 0.08, None))):
        tr = Trial(emb, f"fall_{name}")
        tr.hold_rod = name == "pivot_nut"
        tr.seat(name, (pos[0], pos[1], tr.m.TOP + 0.016))
        tr.step(1.0)
        zero = tr.m.STATE.get("zero")
        res[name] = {"zero": zero, "end_score": tr.reading()["score"]}
    ok = all(v["zero"] is not None and v["end_score"] == 0.0 for v in res.values())
    return {"pass_": bool(ok), **res}


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    trials = {
        "start": lambda e: trial_start(e, out),
        "catch_where_it_stands": trial_catch_where_it_stands,
        "crooked_never_catches": trial_crooked_never_catches,
        "turn_back_off_the_start": trial_turn_back_off_the_start,
        "turned_on_snug": trial_turned_on_snug,
        "pressed_open_and_let_go": trial_pressed_open_and_let_go,
        "falls": trial_falls,
    }
    ok = True
    for name, fn in trials.items():
        res = fn(emb)
        res["pass"] = res.pop("pass_")
        (out / f"{name}.json").write_text(json.dumps(res, indent=1, default=str))
        print(f"TRIAL {name} " + json.dumps({k: v for k, v in res.items() if k != "rows"}, default=str), flush=True)
        ok &= bool(res["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
