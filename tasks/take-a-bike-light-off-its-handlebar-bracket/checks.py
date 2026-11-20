"""Simulator checks of the bike-light task (run with ``scripts/rta simpy <id> mech/tasks/bike_light/checks.py
outputs/<id> <check>``). Each writes its pictures and a JSON of readings into the output folder.

start  the start picture (seed 0): top and wrist cameras as rendered and at the policy's size; 3 s of rest (how far
       the light, lever, button and bowl move, the joints' readings); then both hands to their poses before the work
       (the lever hand 6 cm behind its touch on the tab, the light hand 6 cm over the light, open): the tab in the
       right wrist camera, the light in the left.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _pack(left, right):
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    return Action(data=pack(np.asarray(left, dtype=float), np.asarray(right, dtype=float)))


def _hold():
    from yam_arena.constants import READY

    return _pack(np.r_[READY, 1.0], np.r_[READY, 1.0])


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(kit.policy_picture(frame), cv2.COLOR_RGB2BGR))


def _readings(m) -> dict:
    mk = m.mk
    return {
        "light": [round(float(v), 5) for v in mk.pose("light")[0]],
        "bowl": [round(float(v), 5) for v in mk.pose("bowl")[0]],
        "rail_mm": round(1000 * mk.joint("rail"), 3),
        "lever_mm": round(1000 * mk.joint("lever"), 3),
        "button_mm": round(1000 * mk.joint("button"), 3),
        "on": m.light_on(),
        "score": m.score(None),
    }


def check_start(out: Path) -> dict:
    import demo_kit as kit
    import plan
    from inspect_robots import Scene
    from yam_arena import station

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    observation = embodiment.reset(Scene(id="start", instruction=m.INSTRUCTION), seed=0)
    for cam, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, observation.images[cam])
    rows = []
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
        rows.append(_readings(m))
    first, last = rows[0], rows[-1]
    moved = {n: round(1000 * float(np.linalg.norm(np.subtract(last[n], first[n]))), 3) for n in ("light", "bowl")}
    result = {
        "after 3 s": last,
        "moved from the first step to 3 s, mm": moved,
        "joints over 3 s, mm (least, most)": {j: (min(r[j] for r in rows), max(r[j] for r in rows)) for j in ("rail_mm", "lever_mm", "button_mm")},
        "light over the rail at 3 s, mm": round(1000 * (last["light"][2] - m.LIGHT[2] / 2 - m.RAIL_TOP), 3),
        "events": m.STATE.get("events", []),
    }
    # both hands to their poses before the work, from the ready pose (4 s), then 1 s still
    lever_pre = plan.lever_touch(m) - 0.06 * plan.lever_rot()[:, 2]
    light_pre = plan.light_grasp(m) + np.array([0.0, 0.0, 0.06])
    arms = {
        "left": kit.Arm("left", station.BASES["left"], [kit.Move(4.0, pos=light_pre, rot=plan.light_rot(), grip=1.0, label="over the light"), kit.Move(1.0)]),
        "right": kit.Arm("right", station.BASES["right"], [kit.Move(4.0, pos=lever_pre, rot=plan.lever_rot(), grip=0.0, label="behind the tab"), kit.Move(1.0)]),
    }
    seconds = max(a.seconds for a in arms.values())
    for i in range(int(round(seconds * hz)) + 1):
        t = i / hz
        step = embodiment.step(_pack(arms["left"].action(t)[0], arms["right"].action(t)[0]))
    for cam, name in (("top_cam", "pre_top"), ("left_cam", "pre_left_wrist"), ("right_cam", "pre_right_wrist")):
        _save(out, name, step.observation.images[cam])
    result["hands before the work"] = {
        side: {"grasp point": [round(float(v), 4) for v in mk.gripper(side)[0]], "wanted": [round(float(v), 4) for v in want], "off by mm": round(1000 * float(np.linalg.norm(mk.gripper(side)[0] - want)), 2)}
        for side, want in (("left", light_pre), ("right", lever_pre))
    }
    result["after the hands moved"] = _readings(m)
    result["ik worst mm"] = {side: round(a.worst_mm, 2) for side, a in arms.items()}
    return result


def check_catch(out: Path) -> dict:
    """The mechanism without the arms, by loads put on the bodies directly (``mk.push``, renewed every control step):
    the light pulled forward with the lever not held (10 N, then 40 N: it stays), with the tab pushed down 5 N (2.5 N:
    the rail's friction holds it; 5 N: it slides off and is cut free), a tab let go at 10 mm with the light still
    pulled (it stops there, also under 40 N), and the button pushed down 4 N for 0.9 s (still on) and 1.1 s (off)."""
    import demo_kit as kit
    from inspect_robots import Scene

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)

    def reset():
        embodiment.reset(Scene(id="catch", instruction=m.INSTRUCTION), seed=0)
        for _ in range(int(round(0.5 * hz))):
            embodiment.step(_hold())

    def load(seconds: float, light: float = 0.0, tab: float = 0.0, button: float = 0.0, stop=None) -> dict:
        """Push for ``seconds`` (or until ``stop()``); the light's rail travel (least, most) and the end's readings."""
        travel, t0, step = [], mk.now(), None
        for _ in range(int(round(seconds * hz))):
            if not m.off_rail():
                mk.push("light", (light, 0.0, 0.0))
            else:
                mk.push("light")
            mk.push("lever", (0.0, 0.0, -tab))
            mk.push("button", (0.0, 0.0, -button))
            step = embodiment.step(_hold())
            travel.append(1000 * mk.joint("rail"))
            if stop is not None and stop():
                break
        for name in ("light", "lever", "button"):
            mk.push(name)
        return {"for s": round(mk.now() - t0, 2), "rail mm (least, most)": (round(min(travel), 3), round(max(travel), 3)), **_readings(m), "_step": step}

    def clean(r: dict) -> dict:
        return {k: v for k, v in r.items() if not k.startswith("_")}

    result = {}
    reset()
    result["pulled 10 N, lever not held"] = clean(load(1.0, light=10.0))
    result["pulled 40 N, lever not held"] = clean(load(1.0, light=40.0))
    result["tab pushed 5 N, no pull"] = clean(load(0.5, tab=5.0))
    result["pulled 2.5 N, tab pushed 5 N"] = clean(load(1.0, light=2.5, tab=5.0))
    slid = load(3.0, light=5.0, tab=5.0, stop=m.off_rail)
    result["pulled 5 N, tab pushed 5 N (until off the rail)"] = clean(slid)
    after = load(1.5)
    result["1.5 s after it came off (where it fell)"] = clean(after)
    _save(out, "fell_top", after["_step"].observation.images["top_cam"])
    result["events, first trial"] = m.STATE.get("events", [])

    reset()
    load(3.0, light=5.0, tab=5.0, stop=lambda: mk.joint("rail") >= 0.010)
    result["slid to 10 mm (tab pushed, pulled 5 N)"] = round(1000 * mk.joint("rail"), 3)
    result["tab let go, pulled 5 N for 1 s"] = clean(load(1.0, light=5.0))
    result["then pulled 40 N for 1 s"] = clean(load(1.0, light=40.0))
    result["events, second trial"] = m.STATE.get("events", [])

    reset()
    result["button pushed 4 N for 0.9 s"] = clean(load(0.9, button=4.0))
    clean(load(0.5))
    result["button pushed 4 N for 1.1 s"] = clean(load(1.1, button=4.0))
    off = load(1.0)
    _save(out, "off_top", off["_step"].observation.images["top_cam"])
    result["1 s after"] = clean(off)
    result["lens and indicator drawn"] = (mk.shown("light", m.LENS_LIT), mk.shown("button", m.BUTTON_LIT))
    result["events, third trial"] = m.STATE.get("events", [])
    return result


def check_letgo(out: Path) -> dict:
    """The tab let go mid-slide. The light is drawn along the rail at DRAW m/s by a computed pull (the rail's friction
    plus a spring of 100 N/m to a point moving at that speed, damped with 4 N s/m: stable when read at 20 Hz on its
    130 g), the tab pushed down 5 N; at 10 mm the tab is let go and the pull goes on (at most 10 N) for 1 s: where the
    catch stops the light. Then the same at 25 mm."""
    import demo_kit as kit
    from inspect_robots import Scene

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    draw, k, c, most = 0.02, 100.0, 4.0, 10.0
    result = {}
    for stop_at in (0.010, 0.025):
        embodiment.reset(Scene(id="letgo", instruction=m.INSTRUCTION), seed=0)
        for _ in range(int(round(0.5 * hz))):
            embodiment.step(_hold())
        target, rows, let_go = mk.joint("rail"), [], None
        for i in range(int(round(4.0 * hz))):
            s, v = mk.joint("rail"), float(mk.velocity("light")[0][0])
            if let_go is None and s >= stop_at:
                let_go = (round(mk.now(), 2), round(1000 * s, 3))
            target += draw / hz
            pull = min(most, max(0.0, m.RAIL_FRICTION + k * (target - s) - c * v))
            mk.push("light", (pull, 0.0, 0.0))
            mk.push("lever", (0.0, 0.0, -5.0 if let_go is None else 0.0))
            embodiment.step(_hold())
            rows.append((round(mk.now(), 2), round(1000 * mk.joint("rail"), 3), round(pull, 2), round(1000 * mk.joint("lever"), 3)))
            if let_go is not None and mk.now() - let_go[0] >= 1.0:
                break
        mk.push("light")
        mk.push("lever")
        after = [r[1] for r in rows if let_go and r[0] > let_go[0]]
        result[f"let go at {1000 * stop_at:.0f} mm"] = {
            "let go (s, rail mm as read)": let_go,
            "rail mm after it (least, most)": (min(after), max(after)) if after else None,
            "past the reading, mm": round(max(after) - let_go[1], 3) if after else None,
            "off the rail": m.off_rail(),
            "events": m.STATE.get("events", []),
            "last rows (t, rail mm, pull N, lever mm)": rows[-6:],
        }
    return result


def check_yank(out: Path) -> dict:
    """The tab pushed down 5 N and the light pulled forward with a constant 20 N from rest (more than any hand gives
    in one control step): how far the slide had taken it when it was cut free, against the open rail's front stop
    (RAIL_OFF + RAIL_OVER): it must never reach it."""
    import demo_kit as kit
    from inspect_robots import Scene

    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    embodiment.reset(Scene(id="yank", instruction=m.INSTRUCTION), seed=0)
    for _ in range(int(round(0.5 * hz))):
        embodiment.step(_hold())
    rows = []
    for _ in range(int(round(1.0 * hz))):
        mk.push("lever", (0.0, 0.0, -5.0))
        mk.push("light", (0.0 if m.off_rail() else 20.0, 0.0, 0.0))
        embodiment.step(_hold())
        rows.append((round(mk.now(), 2), round(1000 * mk.joint("rail"), 3), m.off_rail(), round(float(mk.velocity("light")[0][0]), 3)))
        if m.off_rail():
            break
    mk.push("lever")
    mk.push("light")
    return {"rows (t, rail mm, off the rail, speed m/s)": rows, "front stop at mm": round(1000 * (m.RAIL_OFF + m.RAIL_OVER), 1),
            "most travel mm": max(r[1] for r in rows), "events": m.STATE.get("events", [])}


def main() -> int:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    which = sys.argv[2] if len(sys.argv) > 2 else "start"
    checks = {"start": check_start, "catch": check_catch, "letgo": check_letgo, "yank": check_yank}
    if which not in checks:
        raise SystemExit(f"checks here: {sorted(checks)}; got {which}")
    result = checks[which](out)
    (out / f"{which}.json").write_text(json.dumps(result, indent=1, default=str))
    print(f"CHECK_{which.upper()} " + json.dumps(result, default=str), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
