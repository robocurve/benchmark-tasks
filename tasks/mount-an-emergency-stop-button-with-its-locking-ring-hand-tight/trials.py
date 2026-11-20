#!/usr/bin/env python3
"""Checks of the emergency-stop task in the simulator, no arm.

    scripts/rta simpy <id> mech/tasks/estop_mount/trials.py /workspace/robocurve/outputs/<id> [rest,thread,seat,search] [ESTOP_...=value]

``rest``    3 s with nobody touching anything: the first pictures (top camera and both wrist cameras, as rendered and at
            the size a policy gets), and how far anything moves.
``thread``  the operator held where it stands (the trials' weld "op_hold"), the ring turned off by a torque: how far
            it goes a turn and where it comes off; then set back on the thread's end, pushed on and turned on 9.5
            turns to the tab's end, tightened with 1.0 and 2.0 N m and let go: where it holds.
``seat``    the operator written level in front of the hole, its tab up, and driven in at 10 mm/s (at most 3 N):
            taken into the hole? Seated? Then, held in with 2 N, the ring set on the thread's end behind the panel,
            pushed on and turned on to the panel's back, tightened with 2.0 N m and let go; then the head pulled
            with 10 N (the scorer's pull): how far the flange comes off the panel. Then with the ring taken off
            again, pulled out with 3 N: let go? And once more with its tab 10 degrees off the notch: stopped?
``search``  the operator brought in line with its tab off the notch, pushed in lightly until the tab stands on the
            panel's face, then turned under the push: does it go in when the tab meets the notch?

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
RIDERS = ("operator", "ring", "carrier", "turner")  # the operator and what rides on it as built (the tightener is in the world)


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


def settle(embodiment, seconds: float) -> None:
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        embodiment.step(_hold())


def fresh(embodiment, m, name: str):
    from inspect_robots import Scene

    observation = embodiment.reset(Scene(id=name, instruction=m.INSTRUCTION), seed=0)
    embodiment.step(_hold())
    return observation


def push(m, name: str, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    """A push on ``name`` on top of the one that carries its extra weight."""
    lift = m.CARRIED.get(name, 0.0) * m.GRAVITY
    m.mk.push(name, force=(float(force[0]), float(force[1]), float(force[2]) + lift), torque=tuple(float(v) for v in torque))


def rest(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    observation = fresh(embodiment, m, "rest")
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))
    settle(embodiment, 3.0)
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in m.PARTS}
    found = {"moved from where they were built after 3 s (mm)": moved, "after 3 s": m.report()}
    print(f"REST {json.dumps(found, default=str)}", flush=True)
    return found


def thread_row(m, label: str, steps: int | None = None) -> dict:
    mk = m.mk
    return {"after": label, "steps": steps, "turn (turns)": round(mk.joint("turn") / (2 * math.pi), 3), "ring along (mm)": round(1000 * m.ring_along(), 3),
            "ring on": m.STATE["ring_on"], "tight": m.STATE["tight"], "tight (deg)": round(math.degrees(mk.joint("tight")), 2), "lock (deg)": round(math.degrees(m.STATE["lock"]), 2),
            "ring off axis (mm, deg)": [round(1000 * m.ring_off_axis()[0], 3), round(math.degrees(m.ring_off_axis()[1]), 2)],
            "operator moved (mm)": round(1000 * float(np.linalg.norm(mk.pose("operator")[0] - mk.built_pose("operator")[0])), 2), "events": m.STATE["events"][-2:]}  # fmt: skip


def turn_ring(embodiment, m, torque: float, seconds: float, until=None, label: str = "", speed: float | None = None, also=None) -> dict:
    """Turn the ring about the operator's axis with ``torque`` (N m, + screwing on), and if ``speed`` is given drive
    it along that axis at ``speed`` m/s (+ towards the head; at most 0.5 N), for ``seconds`` or until ``until()``;
    ``also()`` runs before every step."""
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    steps = 0
    for steps in range(1, int(round(seconds * hz)) + 1):
        axis = mk.pose("operator")[1] @ m.Z
        force = np.zeros(3)
        if speed is not None:  # driven along the axis, its real weight carried too (run e1g: 2 g sank it off the axis under the drive)
            closing = float((mk.velocity("ring")[0] - mk.velocity("operator")[0]) @ axis) if steps > 1 else 0.0
            force = max(-0.5, min(0.5, m.RING_MASS * (speed - closing) * hz)) * axis + np.array([0.0, 0.0, m.RING_REAL * m.GRAVITY])
        push(m, "ring", force=force, torque=torque * axis)
        if also is not None:
            also()
        embodiment.step(_hold())
        if until is not None and until():
            break
    push(m, "ring")
    row = thread_row(m, label, steps)
    print(f"THREAD {json.dumps(row, default=str)}", flush=True)
    return row


def ring_to_thread_end(m, before: float = 0.0005) -> None:
    """The free ring written in line on the operator's axis, its front face ``before`` short of the thread's end."""
    mk = m.mk
    p, r = mk.pose("operator")
    mk.write_pose("ring", p + r @ np.array([0.0, 0.0, m.A_END - before]), mk.matrix_to_quat(r))


def thread(embodiment, out: Path) -> dict:
    """The operator held where it stands, the ring turned by a torque about the operator's axis."""
    m = embodiment.task.module
    mk = m.mk
    fresh(embodiment, m, "thread")
    mk.make("op_hold")
    rows = [thread_row(m, "start")]
    print(f"THREAD {json.dumps(rows[-1], default=str)}", flush=True)
    rows.append(turn_ring(embodiment, m, -0.02, 1.0, label="unscrewing with 0.02 N m for 1 s"))
    rows.append(turn_ring(embodiment, m, -0.02, 8.0, until=lambda: not m.STATE["ring_on"], label="unscrewing on until it comes off"))
    ring_to_thread_end(m)  # set back on the thread's end in line, pushed on at 5 mm/s and turned on with 0.03 N m
    rows.append(turn_ring(embodiment, m, 0.03, 2.0, until=lambda: m.STATE["ring_on"], speed=0.005, label="set on the thread's end, pushed on and turned"))
    rows.append(turn_ring(embodiment, m, 0.03, 30.0, until=lambda: m.STATE["at_end"], label="screwing on with 0.03 N m until the tab's end"))
    rows.append(turn_ring(embodiment, m, 0.5, 1.0, label="turned on with 0.5 N m at the tab's end (out of the panel: it turns there)"))
    rows.append(turn_ring(embodiment, m, -0.03, 1.0, label="unscrewed with 0.03 N m for 1 s (it comes back at once)"))
    return {"steps": rows, "report": m.report()}


def spring_table(embodiment, m, loads, where: str, also=None) -> list:
    """The tightening spring read with known loads on the ring, rising, 1 s each: the angle past the touch it holds
    (the lock follows it), and the torque its stiffness (TIGHT_K) says for that angle."""
    table = []
    for load in loads:
        row = turn_ring(embodiment, m, load, 1.0, also=also, label=f"tightened with {load:g} N m ({where})")
        angle = math.radians(row["tight (deg)"])
        table.append({"load (N m)": load, "angle (deg)": row["tight (deg)"], "rate (N m/rad)": round(load / angle, 3) if angle > 1e-6 else None,
                      "spring says (N m)": round(m.TIGHT_K * angle, 3), "lock (deg)": row["lock (deg)"]})  # fmt: skip
        print(f"SPRING {where}: {json.dumps(table[-1])}", flush=True)
    return table


def drive_operator(embodiment, m, speed: float, seconds: float, until=None, most: float = 3.0) -> list:
    """Drive the operator along +y (into the hole) at ``speed`` m/s, with at most ``most`` N, its weight carried; a
    row every 0.25 s."""
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    riders = m.OPERATOR_MASS + 2 * m.HIDDEN_MASS + (m.HIDDEN_MASS if m.STATE["seated"] else 0.0)  # with the carrier and the turner
    rows = []
    for k in range(int(round(seconds * hz))):
        v = float(mk.velocity("operator")[0][1]) if k else 0.0
        # once seated, the seat's slide holds with its friction: pushed on past it (without, 0.04 N at a stop never
        # moved it again: e1k, taken in 19.2 mm out and kept there)
        f = max(-most, min(most, riders * (speed - v) * hz + (m.SEAT_FRICTION if m.STATE["seated"] else 0.0)))
        push(m, "operator", force=(0.0, f, m.OPERATOR_MASS * m.GRAVITY))
        embodiment.step(_hold())
        if k % 5 == 4:
            g = m.seat_reading()
            rows.append({"t": round(mk.now(), 2), "seated": m.STATE["seated"], "seat (mm)": round(1000 * mk.joint("seat"), 2) if m.STATE["seated"] else None,
                         "off (mm)": round(1000 * g["off"], 3), "tilt (deg)": round(math.degrees(g["tilt"]), 2), "tab (deg)": round(math.degrees(g["turn"]), 2), "rear in (mm)": round(1000 * g["rear in"], 2)})  # fmt: skip
        if until is not None and until():
            break
    return rows


def seat(embodiment, out: Path) -> dict:
    """The operator written level in front of the hole and driven in along the hole's axis; the ring put on behind."""
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    found: dict = {}
    for label, tab_off in (("tab in the notch", 0.0), ("tab 10 degrees off", math.radians(10.0))):
        fresh(embodiment, m, f"seat {label}")
        rot = mk.axis_angle(np.array([0.0, -1.0, 0.0]), tab_off) @ m.SEATED_ROT
        start = m.SEAT_AT + np.array([0.0, -0.025, 0.0])  # the stem's end 5.5 mm in front of the panel
        for name in RIDERS:  # the ring and the thread's bodies ride along, as built on it
            position, rotation = mk.built_pose(name)
            rel_p, rel_r = mk.built_pose("operator")[1].T @ (position - mk.built_pose("operator")[0]), mk.built_pose("operator")[1].T @ rotation
            mk.write_pose(name, start + rot @ rel_p, mk.matrix_to_quat(rot @ rel_r))
        mk.cut(m.ring_weld())  # the ring off, aside on the bench
        m.STATE["ring_on"] = False
        mk.write_pose("ring", np.array([0.40, -0.15, m.TOP + 0.005]), mk.matrix_to_quat(np.eye(3)))
        result: dict = {"driven in at 10 mm/s": drive_operator(embodiment, m, 0.010, 3.0, until=lambda: m.STATE["seated"] and mk.joint("seat") > -0.0002)}
        print(f"SEAT {label}: driven in: {json.dumps(result['driven in at 10 mm/s'][-3:], default=str)} events {m.STATE['events']}", flush=True)
        if tab_off == 0.0 and m.STATE["seated"]:
            hold_in = lambda: push(m, "operator", force=(0.0, 2.0, m.OPERATOR_MASS * m.GRAVITY))  # noqa: E731 - the right hand keeps it seated
            ring_to_thread_end(m)
            rows = [turn_ring(embodiment, m, 0.03, 2.0, until=lambda: m.STATE["ring_on"], speed=0.005, also=hold_in, label="ring set on the thread's end behind the panel, pushed on and turned")]
            rows.append(turn_ring(embodiment, m, 0.03, 30.0, until=lambda: m.STATE["tight"], also=hold_in, label="screwed on with 0.03 N m until it touches"))
            hand = m.hand_on_ring
            m.hand_on_ring = lambda: True  # the thread's hold off, as while a hand is on the ring: the spring alone holds it (with the hold on, e1k read 2.74 degrees under every load)
            result["spring table"] = spring_table(embodiment, m, (0.1, 0.5, 1.0, 1.5, 2.0), "seated, held in with 2 N, the hold off", also=hold_in)
            m.hand_on_ring = hand
            rows.append(turn_ring(embodiment, m, 0.0, 1.0, label="let go of ring and operator"))
            before = mk.joint("seat")
            gaps = []
            for k in range(int(round(1.0 * hz))):  # the scorer's pull: 10 N on the head, out of the hole
                push(m, "operator", force=(0.0, -m.PULL_N, m.OPERATOR_MASS * m.GRAVITY))
                embodiment.step(_hold())
                gaps.append(-mk.joint("seat"))
            push(m, "operator")
            result["ring"] = rows
            result["pull"] = {"flange off the panel before (mm)": round(-1000 * before, 4), "most under 10 N (mm)": round(1000 * max(gaps), 4), "after 1 s (mm)": round(1000 * gaps[-1], 4),
                              "seated": m.STATE["seated"], "report": m.report()}  # fmt: skip
            print(f"SEAT {label}: pull {json.dumps(result['pull'], default=str)}", flush=True)
            mk.cut(m.ring_weld())  # the ring taken off again (its seat's stops opened), then the operator pulled out
            mk.set_limits("seat", lower=m.SEAT_OUT, upper=0.0)
            m.STATE.update(ring_on=False, tight=False)
            mk.write_pose("ring", np.array([0.40, -0.15, m.TOP + 0.005]), mk.matrix_to_quat(np.eye(3)))
        for k in range(int(round(1.0 * hz))):
            push(m, "operator", force=(0.0, -3.0, m.OPERATOR_MASS * m.GRAVITY))
            embodiment.step(_hold())
        push(m, "operator")
        result["pulled out"] = {"seated": m.STATE["seated"], "events": m.STATE["events"]}
        found[label] = result
        print(f"SEAT {label}: pulled out {json.dumps(result['pulled out'], default=str)}", flush=True)
    return found


def place_operator(m, rot, at) -> None:
    """The operator (and what rides on it, as built) written with its frame at ``at`` turned as ``rot``; the ring cut off
    and put aside on the bench."""
    mk = m.mk
    p0, r0 = mk.built_pose("operator")
    for name in RIDERS:
        position, rotation = mk.built_pose(name)
        mk.write_pose(name, at + rot @ (r0.T @ (position - p0)), mk.matrix_to_quat(rot @ (r0.T @ rotation)))
    mk.cut(m.ring_weld())
    m.STATE["ring_on"] = False
    mk.write_pose("ring", np.array([0.40, -0.15, m.TOP + 0.005]), mk.matrix_to_quat(np.eye(3)))


def search(embodiment, out: Path) -> dict:
    """The tab found by feel: the operator in front of the hole with its tab 20 degrees off the notch, pushed in lightly
    (at most 1 N) until the tab stands on the panel's face; then, held in line with the hole as a hand holds it (run
    e1h, free: it leant 4.4 degrees in the bore's play and contact undid most of a written turn), turned towards the
    notch 1 degree a control step (20 degrees a second, written) under that push until it drops in and the gate takes
    it; then pushed home."""
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    fresh(embodiment, m, "search")
    place_operator(m, mk.axis_angle(np.array([0.0, -1.0, 0.0]), math.radians(20.0)) @ m.SEATED_ROT, m.SEAT_AT + np.array([0.0, -0.025, 0.0]))
    pushed = drive_operator(embodiment, m, 0.010, 2.5, most=1.0)
    stopped = m.seat_reading()
    turning = []
    for k in range(int(round(5.0 * hz))):
        if m.STATE["seated"]:
            break
        p, r = mk.pose("operator")
        g = m.seat_reading()
        tab = g["turn"] - math.copysign(min(abs(g["turn"]), math.radians(1.0)), g["turn"])
        rot = mk.axis_angle(np.array([0.0, -1.0, 0.0]), tab) @ m.SEATED_ROT  # in line with the hole, turned about it
        at = np.array([m.HOLE[0], p[1], m.HOLE[2]])
        for name in ("operator", "carrier", "turner"):  # with what rides on it
            q, s = mk.pose(name)
            mk.write_pose(name, at + rot @ (r.T @ (q - p)), mk.matrix_to_quat(rot @ (r.T @ s)))
        push(m, "operator", force=(0.0, 1.0, m.OPERATOR_MASS * m.GRAVITY))
        embodiment.step(_hold())
        if k % 5 == 4 or m.STATE["seated"]:
            g = m.seat_reading()
            turning.append({"t": round(mk.now(), 2), "tab (deg)": round(math.degrees(g["turn"]), 2), "rear in (mm)": round(1000 * g["rear in"], 3),
                            "off (mm)": round(1000 * g["off"], 3), "tilt (deg)": round(math.degrees(g["tilt"]), 2), "seated": m.STATE["seated"]})  # fmt: skip
    home = drive_operator(embodiment, m, 0.010, 1.5, most=1.0, until=lambda: m.STATE["seated"] and mk.joint("seat") > -0.0002)
    found = {"pushed in lightly": pushed[-2:], "where the tab stopped it": {"tab (deg)": round(math.degrees(stopped["turn"]), 2), "rear in (mm)": round(1000 * stopped["rear in"], 3),
             "gasket off the panel (mm)": round(1000 * stopped["gasket gap"], 3)}, "turned": turning, "pushed home": home[-2:],
             "seated": m.STATE["seated"], "seat (mm)": round(1000 * mk.joint("seat"), 3) if m.STATE["seated"] else None, "events": m.STATE["events"]}  # fmt: skip
    print(f"SEARCH {json.dumps(found, default=str)}", flush=True)
    return found


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    known = {"rest": rest, "thread": thread, "seat": seat, "search": search}
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    for arg in sys.argv[3:]:
        key, value = arg.split("=", 1)
        if not key.startswith("ESTOP_"):
            raise SystemExit(f"unknown argument {arg!r}")
        os.environ[key] = str(float(value))
    if any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    failed = 0
    for name in trials:
        try:
            result = known[name](embodiment, out)
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
