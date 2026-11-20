#!/usr/bin/env python3
"""Checks of the cake task in the simulator.

    scripts/rta simpy <id> mech/tasks/cake_smooth/trials.py /workspace/robocurve/outputs/<id> [rest,cells]

``rest``   reset, the first pictures of the three cameras (as rendered and at the size a policy gets), then 3 s with
           nobody touching anything: how far each body has moved, the stand's top's spin, the icing as it starts.
``cells``  what drawing the icing costs: control steps with no tile moved, then steps with 170 tiles moved each
           (about what the edge changes while it works: five columns of 34 a step at a turn a second), then every
           tile moved each step; the median step time of each.

Every trial writes <out dir>/<trial>.json.
"""

from __future__ import annotations

import json
import math
import sys
import time
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


def steps(embodiment, seconds: float, before=None) -> list[float]:
    """Hold both arms ready for ``seconds``; ms each step took (``before()`` runs first each step and is timed too)."""
    spent = []
    for _ in range(int(round(seconds * float(embodiment.info.control_hz)))):
        start = time.perf_counter()
        if before is not None:
            before()
        embodiment.step(_hold())
        spent.append(1000.0 * (time.perf_counter() - start))
    return spent


def rest(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    mk = m.mk
    observation = embodiment.reset(Scene(id="rest", instruction=m.INSTRUCTION), seed=0)
    for camera, name in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        _save(out, name, np.asarray(observation.images[camera]))

    def where() -> dict:  # the smoother's and the block's place from where they were built (mm), the smoother's speed (m/s)
        return {name: [round(1000 * float(v), 2) for v in mk.pose(name)[0] - np.asarray(mk.built_pose(name)[0], dtype=float)] for name in ("smoother", "block")} | {
            "smoother speed (m/s)": round(float(np.linalg.norm(mk.velocity("smoother")[0])), 4)}

    early = []
    for i in range(20):  # the first second, step by step (a pose is read from the first control step on)
        embodiment.step(_hold())
        early.append({"t": round(0.05 * (i + 1), 2), **where()})
        print(f"REST {json.dumps(early[-1])}", flush=True)
    spent = steps(embodiment, 2.0)
    moved = {name: round(1000 * float(np.linalg.norm(mk.pose(name)[0] - mk.built_pose(name)[0])), 3) for name in ("stand", "turntable", "cake", "smoother", "block")}
    tilt = {name: round(float(np.degrees(np.arccos(np.clip(mk.pose(name)[1][2, 2], -1.0, 1.0)))), 3) for name in ("stand", "cake", "smoother", "block")}
    found = {"the first second (mm from built)": early, "moved from where they were built after 3 s (mm)": moved, "tilted from upright (deg)": tilt, "top's spin (deg)": round(float(np.degrees(mk.joint("spin"))), 4),
             "report": m.report(), "median step (ms)": round(float(np.median(spent)), 1), "hands' hull corners read": {s: len(m.STATE["hands"][s]) for s in m.SIDES} if m.STATE.get("hands") else None}  # fmt: skip
    return found


def cells(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    m = embodiment.task.module
    embodiment.reset(Scene(id="cells", instruction=m.INSTRUCTION), seed=0)
    steps(embodiment, 1.0)
    rng = np.random.default_rng(1)
    found = {}
    for label, count in (("no tile moved", 0), ("170 tiles moved a step", 170), ("every tile moved a step", m.COLUMNS * m.ROWS)):

        def shuffle(count=count) -> None:
            if not count:
                return
            picks = rng.choice(m.COLUMNS * m.ROWS, size=count, replace=False)
            for p in picks:
                j, k = divmod(int(p), m.ROWS)
                m.STATE["field"][j, k] = m.THICK + rng.uniform(-0.0015, 0.0015)
                m.STATE["changed"].add((j, k))

        spent = steps(embodiment, 2.0, before=shuffle)
        found[label] = {"median step (ms)": round(float(np.median(spent)), 1), "90th percentile (ms)": round(float(np.percentile(spent, 90)), 1)}
        print(f"CELLS {label}: {found[label]}", flush=True)
    observation = embodiment.step(_hold()).observation
    _save(out, "cells_top", np.asarray(observation.images["top_cam"]))
    return found


HOLD_OFF = 0.020  # m the held edge stands off the icing's mean surface: no icing is touched in the hold check
LOADS = (("outward", 0.2), ("outward", 0.5), ("outward", 1.0), ("along the turning", 0.2), ("along the turning", 0.5), ("along the turning", 1.0), ("both", 0.44))
LOAD_S, UNLOAD_S, READ_S = 1.5, 1.0, 0.5


def hold(out: Path) -> dict:
    """The hand's hold of the smoother at its working pose, with known loads on the edge's middle: the smoother is built
    there welded to the world (CAKE_SMOOTHER_AT), the right hand comes in from behind with its fingers open, closes,
    the weld is cut; then each load in LOADS for LOAD_S with UNLOAD_S unloaded after it. Read over the last READ_S of
    each: how far the edge's middle (at the band's mid-height) has gone outward and along the turning from where it
    stood unloaded, and how far the plate has slipped in the fingers."""
    import os

    import demo
    import demo_kit as kit
    from yam_arena import task_file

    plain = task_file.load(str(HERE / "task.py")).module
    plan0 = demo.Plan(plain)
    plate_p, plate_r = plan0.plate_at_cake(plain.THICK + HOLD_OFF)
    os.environ["CAKE_SMOOTHER_AT"] = ",".join(f"{v:.6f}" for v in (*plate_p, *plain.mk.matrix_to_quat(plate_r)))
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    assert m.HELD_AT is not None
    plan = demo.Plan(m, lambda: {"poses": {n: mk.pose(n) for n in ("smoother", "block", "cake", "turntable")}, "spin": mk.joint("spin"), "mean": float(m.band().mean())})
    grasp, rot = plan.hold_pose(plate_p, plate_r)
    behind = grasp - demo.BEHIND * plate_r[:, 0]
    Move = kit.Move
    moves = [Move(2.5, pos=behind + np.array([0.0, 0.0, 0.04]), rot=rot, grip=demo.OPEN, label="above behind"), Move(1.0, pos=behind, label="behind"),
             Move(1.5, pos=grasp, label="slide over"), Move(0.6, grip=demo.CLOSED, label="close"), Move(60.0, label="holds")]  # fmt: skip
    cut_at, settled_at = 6.0, 8.0
    schedule = []
    t = settled_at
    for kind, newtons in LOADS:
        schedule.append((t, t + LOAD_S, kind, newtons))
        t += LOAD_S + UNLOAD_S
    end = t
    radial = np.array([math.cos(demo.SIDE), math.sin(demo.SIDE), 0.0])
    along = np.cross(np.array([0.0, 0.0, 1.0]), radial)  # the icing's way past the edge with the cake turning anticlockwise from above
    mid_z = m.CAKE_AT[2] + m.DUMMY_H / 2
    rows: list = []

    def edge_mid() -> np.ndarray:
        a, b = m.edge_points()
        s = (mid_z - a[2]) / (b[2] - a[2])
        return a + s * (b - a)

    def gripper_pose() -> tuple[np.ndarray, np.ndarray]:
        base = mk.MECH.environment.unwrapped
        robot = base.scene["right_robot"]
        index = robot.find_bodies("gripper")[0][0]
        pose = robot.data.body_link_pose_w.torch[0, index].detach().cpu().numpy().astype(float)
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        return pose[:3] - origin, mk.quat_to_matrix(pose[3:7])

    # the cue check afterwards: the right wrist camera from this working pose, the icing set to each state directly
    cues = [(end + 3.0 * i, name) for i, name in enumerate(("ridged", "smoothed", "bare"))]
    combed = m.STATE["field"].copy() if "field" in m.STATE else None
    near = [j for j in range(m.COLUMNS) if abs(((m.column_angle(j) - demo.SIDE + math.pi) % (2 * math.pi)) - math.pi) <= math.radians(35.0)]  # the columns either side of the edge

    def set_icing(name: str) -> None:
        field = m.start_field(0)[0] if combed is None else combed.copy()
        if name in ("smoothed", "bare"):
            for j in near:
                if m.column_angle(j) % (2 * math.pi) >= (demo.SIDE % (2 * math.pi)):  # the side the turning has passed: smoothed
                    field[j, :] = m.THICK - 0.0005
        if name == "bare":
            patch = [j for j in near if m.column_angle(j) % (2 * math.pi) >= (demo.SIDE % (2 * math.pi))][2:6]
            for j in patch:
                field[j, 12:22] = 0.0005
        m.STATE["field"] = field
        m.STATE["changed"] = {(j, k) for j in range(m.COLUMNS) for k in range(m.ROWS)}
        mk.redraw(6)

    def director(t: float, arms: dict, pictures, state={"cut": False, "cue": -1}) -> None:
        if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
            return
        if not state["cut"] and t >= cut_at:
            mk.cut("smoother_held")
            state["cut"] = True
        for i, (start, name) in enumerate(cues):
            if t >= start and state["cue"] < i:
                set_icing(name)
                state["cue"] = i
            if abs(t - (start + 2.5)) < 0.026:
                _save(out, f"cue_{name}", np.asarray(pictures["right_cam"]))
        load = next(((kind, newtons) for start, stop, kind, newtons in schedule if start <= t < stop), None)
        force = np.zeros(3)
        if load is not None:
            kind, newtons = load
            force = newtons * {"outward": radial, "along the turning": along, "both": radial + along}[kind]
        at = edge_mid()
        com = mk.MECH.environment.unwrapped.scene.rigid_objects["smoother"].data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float) - mk.MECH.environment.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(float)
        mk.push("smoother", force=tuple(force), torque=tuple(np.cross(at - com, force)))
        g_p, g_r = gripper_pose()
        plate_p_now, _ = mk.pose("smoother")
        rows.append({"t": t, "load": load, "edge": at.tolist(), "plate in the hand": (g_r.T @ (plate_p_now - g_p)).tolist()})

    result = kit.run(embodiment, {"right": moves}, out / "hold", seed=0, director=director, limit_s=end + 9.0, tail_s=0.0, sheet_frames=12, extra=lambda: {"cut": "smoother_held" in mk.MECH.cut})

    def mean_over(start: float, stop: float, key: str) -> np.ndarray:
        picked = [np.asarray(r[key]) for r in rows if start <= r["t"] < stop]
        return np.mean(picked, axis=0)

    rest_edge = mean_over(settled_at - READ_S, settled_at, "edge")
    rest_hand = mean_over(settled_at - READ_S, settled_at, "plate in the hand")
    table = []
    for start, stop, kind, newtons in schedule:
        e = mean_over(stop - READ_S, stop, "edge") - rest_edge
        h = mean_over(stop - READ_S, stop, "plate in the hand") - rest_hand
        back = mean_over(stop + UNLOAD_S - READ_S, stop + UNLOAD_S, "edge") - rest_edge
        table.append({"load": f"{newtons:g} N {kind}", "edge outward (mm)": round(1000 * float(e @ radial), 3), "edge along the turning (mm)": round(1000 * float(e @ along), 3),
                      "edge up (mm)": round(1000 * float(e[2]), 3), "slip in the fingers (mm)": round(1000 * float(np.linalg.norm(h)), 3),
                      "after it, unloaded: outward, along (mm)": [round(1000 * float(back @ radial), 3), round(1000 * float(back @ along), 3)]})  # fmt: skip
        print(f"HOLD {table[-1]}", flush=True)
    return {"edge off the icing's mean surface (mm)": 1000 * HOLD_OFF, "rest: edge's middle (m)": rest_edge.tolist(), "loads": table, "worst IK": result["worst_ik"],
            "edge's middle before the cut and at rest after it (mm apart)": round(1000 * float(np.linalg.norm(mean_over(cut_at - READ_S, cut_at, "edge") - rest_edge)), 3)}  # fmt: skip


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    trials = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rest"]
    if trials == ["hold"]:  # its own scene: the smoother built in the hand's working pose
        found = hold(out)
        (out / "hold.json").write_text(json.dumps(found, indent=1, default=str))
        print(f"TRIAL hold: {json.dumps(found, default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    failed = []
    for trial in trials:
        try:
            found = {"rest": rest, "cells": cells}[trial](embodiment, out)
            (out / f"{trial}.json").write_text(json.dumps(found, indent=1, default=str))
            print(f"TRIAL {trial}: {json.dumps(found, default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"TRIAL {trial} FAILED: {error!r}", flush=True)
            failed.append(trial)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
