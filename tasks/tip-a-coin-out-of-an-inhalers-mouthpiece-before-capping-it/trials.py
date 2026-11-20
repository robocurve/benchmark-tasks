#!/usr/bin/env python3
"""Checks of the inhaler task in the simulator, no arm (task.py).

    scripts/rta simpy <id> mech/tasks/inhaler_coin/trials.py /workspace/robocurve/outputs/<id> [rest] [seed=N]

``rest``  the first pictures (top camera and both wrist cameras, as rendered and at the size a policy gets) and 3 s
          with nobody touching anything: how far every body moves, where the lodged coin rests in the mouthpiece.
``hold``  the right hand takes the inhaler and holds it up as demo.py does, holds it still, turns the mouthpiece up
          and jerks it: whether the lodged coin stays held, the largest jolt read, where it goes when let go.
``cap``   (alone; cap_task.py, the inhaler welded to the world) the cap's fit under known loads, no arm: the cap put
          lined up with its rim 1 mm over the mouthpiece's end, pushed on with 0.5 N (it slides on and is welded at
          its seat), let be, pulled with 2 N (it stays). A first episode with the coin moved out of the way (the safe
          close), a second with the coin where it starts, sticking out (the unsafe close: the cap's end drives the
          coin in). The force is on the cap alone, put on again every control step.
Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy_x3.png"), cv2.cvtColor(cv2.resize(small, (small.shape[1] * 3, small.shape[0] * 3), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))


def rest(embodiment, out: Path, seed: int) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=seed)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    embodiment.step(_hold())
    names = list(mk.MECH.bodies)
    at = {name: mk.pose(name)[0].copy() for name in names}
    coin0 = m.rel("coin")[0].copy()
    hz = float(embodiment.info.control_hz)
    speeds = []
    for _ in range(int(round(3.0 * hz))):
        embodiment.step(_hold())
        speeds.append(float(np.linalg.norm(mk.velocity("coin")[0])))
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - at[name])), 3) for name in names}
    found = {"seed": seed, "moved over 3 s (mm)": moved, "coin in I at first (mm)": (1000 * coin0).round(2).tolist(),
             "coin in I after 3 s (mm)": (1000 * m.rel("coin")[0]).round(2).tolist(), "coin speed max (mm/s)": round(1000 * max(speeds), 2),
             "coin out past the mouth (mm)": round(1000 * (m.rel("coin")[0][0] + m.P.LODGED_R - m.P.MOUTH_X), 2), "score": m.score(None)}  # fmt: skip
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def centre(m, name: str) -> np.ndarray:
    """Body ``name``'s centre of mass in the world, where a push acts."""
    return m.mk.MECH.environment.unwrapped.scene.rigid_objects[name].data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float)


PUSH_DAMPING = 0.0  # N s/m: none. A push read from the last step's speed is a damper held for 50 ms: above 2 m / dt (0.56 N s/m
# for the 14 g cap and carrier) it swings up; 20 N s/m sent the cap to NaN in runs w9i10 and w9i14


def cap_on_pose(m, over: float = 0.001) -> tuple[np.ndarray, tuple]:
    """The cap lined up on the mouthpiece, its rim ``over`` past the mouth's end: world position and quaternion."""
    p, r = m.F.i_pose()
    r_rel = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    p_rel = np.array([m.P.MOUTH_X - over + 0.5 * m.P.CAP_DEPTH, 0.0, 0.0])
    return p + r @ p_rel, m.mk.matrix_to_quat(r @ r_rel)


def cap(embodiment, out: Path, seed: int) -> dict:
    """cap_task.py: the inhaler welded to the world where task.py lays it, the cap put lined up with its rim 1 mm over the
    mouthpiece's end, slid on as a hand holds it (3 cm/s, lined up: a push at its middle alone tips it and it jams on
    the 0.3 mm play, run w9i17) until seated, let be, pulled back with 2 N. A first episode with the coin moved aside,
    a second with it lodged as at the start (the unsafe close)."""
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    found: dict = {}
    for label, coin_aside in (("safe: coin moved aside", True), ("unsafe: coin where it starts", False)):
        embodiment.reset(Scene(id="cap", instruction=m.INSTRUCTION), seed=seed)
        embodiment.step(_hold())
        if coin_aside:
            mk.cut("lodged")  # let go of first (lodge.py), then put on the bench
            mk.write_pose("coin", (0.30, -0.12, m.TOP + 0.5 * m.P.DIME_T + 0.001), (0.0, 0.0, 0.0, 1.0))
        pos, quat = cap_on_pose(m)
        mk.write_pose("cap", tuple(pos), quat)
        rows = []

        def phase(name: str, newtons: float, seconds: float) -> None:
            for _ in range(int(round(seconds * hz))):
                u = -m.F.i_pose()[1][:, 0]  # along the mouthpiece, towards the barrel
                mk.push("cap", force=tuple(newtons * u))
                embodiment.step(_hold())
            mk.push("cap")
            c_rel = m.rel("cap")[0]
            row = {"phase": name, "gap_mm": round(1000 * m.gap(), 2), "seated": m.seated(), "coin_in": m.coin_in(), "coin_in_I_mm": (1000 * m.rel("coin")[0]).round(2).tolist(),
                   "cap_off_axis_mm": round(1000 * float(np.hypot(c_rel[1], c_rel[2])), 2), "cap_speed": round(float(np.linalg.norm(mk.velocity("cap")[0])), 4),
                   "t": round(mk.now(), 2)}  # fmt: skip
            rows.append(row)
            print("CAP", label, json.dumps(row), flush=True)

        def slide_on(speed: float, seconds: float) -> None:
            """As a hand holds it: each control step the cap put lined up a little further along the mouthpiece, until the
            task's check has seated it or its end has come to the mouth's end (a coin in the way is pushed by the cap)."""
            over = 0.001
            for _ in range(int(round(seconds * hz))):
                if m.seated() or m.gap() <= 0.0005:
                    break
                over += speed / hz
                p, q = cap_on_pose(m, over=over)
                mk.write_pose("cap", tuple(p), q)
                embodiment.step(_hold())
            c_rel = m.rel("cap")[0]
            row = {"phase": f"slid on at {100 * speed:.0f} cm/s", "gap_mm": round(1000 * m.gap(), 2), "seated": m.seated(), "coin_in": m.coin_in(),
                   "coin_in_I_mm": (1000 * m.rel("coin")[0]).round(2).tolist(), "cap_off_axis_mm": round(1000 * float(np.hypot(c_rel[1], c_rel[2])), 2), "t": round(mk.now(), 2)}  # fmt: skip
            rows.append(row)
            print("CAP", label, json.dumps(row), flush=True)

        phase("lined up, let be", 0.0, 0.3)
        slide_on(0.03, 1.0)
        phase("let be", 0.0, 0.5)
        phase("pulled with 2 N", -2.0, 0.5)
        phase("let be", 0.0, 0.5)
        found[label] = {"rows": rows, "events": [f"{t:.2f} s {w}" for t, w in m.EVENTS]}
    print(f"CAPFIT {json.dumps({k: v['rows'][-1] for k, v in found.items()}, default=str)}", flush=True)
    return found

def hold(embodiment, out: Path, seed: int) -> dict:
    """The right hand takes the inhaler and holds it up as demo.py does (its first two steps, the pinch demo.SQUEEZE),
    holds it still 4 s, turns the mouthpiece up 20 deg about the barrel (2 s), holds it 1 s, jerks it 15 mm out along
    the mouthpiece in 0.12 s and back, and holds it 1 s. Where the lodged coin is in the mouthpiece every half second,
    whether it is still held (lodge.py), the largest jolt read while it was held. Before lodge.py (runs w9i24 at 0.15,
    w9i26 at 0.25 and 0.29, w9i27 with compliant barrel contacts) the coin crept 23 mm in, or 9 mm out, while the
    inhaler was held level within 0.5 deg."""
    import demo
    import demo_kit as kit

    m = embodiment.task.module
    mk = m.mk
    plan = demo.Plan(m, live=True)
    script = plan.steps()
    state = {"steps": 0}
    Move = plan.Move

    def later():
        held, hand = plan.held, None
        grasp = held[0] + held[1] @ np.array([0.0, 0.0, demo.GRASP_Z])
        up = demo.ry(np.radians(20.0)) @ held[1]
        _, hand = plan.right_on(grasp - up @ np.array([0.0, 0.0, demo.GRASP_Z]), up)
        out_dir = up[:, 0]  # out of the mouth
        return [Move(4.0, label="holds it still"), Move(2.0, rot=hand, label="mouth up 20 deg"), Move(1.0, label="holds it up"),
                Move(0.12, pos=grasp + 0.015 * out_dir, label="jerks it out"), Move(0.3, pos=grasp, label="back"), Move(1.0, label="holds it")]  # fmt: skip

    def director(t, arms, pictures):
        if pictures is None or not mk.MECH.poses:
            return
        while state["steps"] < 3 and all(t >= arms[side].seconds - 1e-6 for side in demo.SIDES):
            step = next(script) if state["steps"] < 2 else plan.both([], later())
            state["steps"] += 1
            for side, moves in step.items():
                arms[side].moves.extend(moves)

    def extra():
        return {"coin x (mm)": round(1000 * float(m.rel("coin")[0][0]), 2), "lodged": m.L.lodged(), "mouthpiece up (deg)": round(float(np.degrees(np.arcsin(m.F.i_pose()[1][2, 0]))), 2),
                "load angle (deg)": None if m.L.STATE["load_in_i"] is None else round(float(np.degrees(np.arctan2(np.hypot(m.L.STATE["load_in_i"][0], m.L.STATE["load_in_i"][2]), -m.L.STATE["load_in_i"][1]))), 2)}  # fmt: skip

    result = kit.run(embodiment, {}, out / "hold", seed=seed, director=director, extra=extra, tail_s=1.0, limit_s=30.0, record=False)
    rows = result["rows"]
    track = [(r["t"], r["right"], r["extra"]["coin x (mm)"], r["extra"]["lodged"], r["extra"]["mouthpiece up (deg)"], r["extra"]["load angle (deg)"]) for r in rows if round(r["t"] * 20) % 10 == 0]
    still = [r["extra"]["load angle (deg)"] for r in rows if r["right"] in ("holds it up", "holds it still", "turns it", "lifts it") and r["extra"]["load angle (deg)"] is not None and r["extra"]["lodged"]]
    found = {"track (t, step, coin x mm, held, mouthpiece up deg, load angle deg)": track, "released": m.L.STATE["released"],
             "while held: most load angle (deg), most jolt (g)": [round(float(np.degrees(m.L.STATE["most_angle"])), 2), round(m.L.STATE["most_jolt"], 3)],
             "most load angle while held up or still (deg)": max(still) if still else None}  # fmt: skip
    print("HOLD", json.dumps(found, default=str), flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "cap": cap, "hold": hold}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    seed = 0
    for arg in sys.argv[3:]:
        key, value = arg.split("=", 1)
        if key != "seed":
            raise SystemExit(f"unknown argument {arg!r}")
        seed = int(value)
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    alone = {"cap"}
    if alone & set(trials) and len(trials) != 1:
        raise SystemExit("the cap trials run alone (they open cap_task.py)")
    embodiment = kit.open_task(HERE / ("cap_task.py" if trials[0] in alone else "task.py"))
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out, seed)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {name} FAILED: {error!r}", flush=True)
            failed += 1
            continue
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
