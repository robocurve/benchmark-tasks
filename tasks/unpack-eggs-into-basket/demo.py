"""Two-handed demo for the egg-unpacking task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record] [--one-hand | --drop]

1. The left hand, closed, presses its fingertips on the base's thumb tab. It leans 40 degrees out to its own side,
   which keeps its wrist clear of the right arm: the two tabs are 8 cm apart and two hands pointing straight down
   need about 17 cm. The right hand pinches the lid's pull tab by its two side edges and carries it up and over
   along the lid's arc; the tab turns freely between the finger pads, so the hand stays pointing down. Past the
   top it lets go and the lid falls open toward the arms.
2. The left hand moves away. The right hand takes the eggs one at a time, starting with the column furthest from
   the basket: its fingers come down on either side of the egg's cap (one between the two rows, one outside the
   carton), close on the cap and lift the egg clear of its cup. The arm cannot lift an egg over the eggs still
   standing in the carton, so it carries each one out over the open lid, along the front of the carton and into the
   basket. There it lowers the egg until its end touches the cloth, then moves sideways and down along the arc the
   cap follows as the egg rolls onto its side (the egg turns freely between the two finger pads), and opens only
   when the egg is lying down. An egg let go standing falls over and rolls into the next egg's place.
3. With one column left the carton weighs under 150 g and comes up with an egg that rubs its cup, so the left hand
   presses on the thumb tab again while the right hand takes the last two eggs.

Two controls that must not score:
  --one-hand   the right hand pulls the lid's tab without the left hand holding the base, then stops.
  --drop       the full sequence, but each egg is let go 6 cm above the cloth.

Open loop: every pose comes from the layout constants in the task file. Object states are read only to report
where things ended up. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0
TIP = 0.013                        # the pads' tips reach this far past the tool point

PRESS = 0.015                      # how far below the thumb tab's top the left fingertips are sent: the arm gives
                                   # about 0.3 N per mm, so this presses with about 5 N
TAB_OPEN, TAB_GRIP = 0.045, 0.025  # finger gap around / on the lid's 30 mm tab
ARC_END = 105.0                    # degrees of lid angle at which the right hand lets the tab go
EGG_OPEN = 0.050                   # finger gap coming down around an egg's cap
EGG_GRIP = 0.039                   # finger gap commanded on the cap (the cap is 41.6 mm wide where the pads touch)
EGG_RELEASE = 0.055
CAP = 0.003                        # the tool point sits this far below the egg's top (fingertips 16 mm below it)
SET_PRESS = 0.001                  # the egg's end is sent this far into the cloth before it is laid down
LAY_END = 85.0                     # degrees from upright at which the egg is let go
FRONT = 0.045                      # the carrying lane runs this far in front of the carton, over the open lid
CARRY = 0.110                      # tool height above the table for carrying: the egg clears cups and rim by 1 cm
DROP_EXTRA = 0.060                 # --drop lets go this much higher


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# Pointing straight down. Each arm's wrist reaches only one of the two ways of closing along an axis.
RIGHT_X = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))    # right hand, fingers closing front to back
RIGHT_Y = frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))   # right hand, fingers closing left to right
HOLD_LEAN = math.radians(40.0)                        # the holding hand leans this far out to its own side
LEFT_HOLD = frame((0.0, -math.sin(HOLD_LEAN), -math.cos(HOLD_LEAN)), (-1.0, 0.0, 0.0))


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

    def path(self, point, duration: float):
        """Generator: follow ``point(s)`` for s from 0 to 1 at an even pace, keeping the tool's direction."""
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(point(k / n), self.rot)
            yield

    def set_grip(self, grip: float, duration: float = 0.6):
        yield from self.move(self.pos, duration, grip=grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def poses(T) -> dict:
    """Every pose the demo uses, in world coordinates, from the task file's layout."""
    bx, by = T.BASKET_XY
    edge = -T.CARTON_D / 2                                    # the carton's side opposite the hinge, in its own frame

    def tab(deg: float) -> np.ndarray:
        """Centre of the pinch on the lid's tab when the lid is open by ``deg``."""
        a = math.radians(deg)
        return np.array(T.carton_point(T.CARTON_D / 2 - T.SNAP_ARM * math.cos(a) + T.TAB_T / 2 * math.sin(a),
                                       T.LID_TAB_Y, T.BASE_H + T.SNAP_ARM * math.sin(a) + T.TAB_T / 2 * math.cos(a)))

    # Fingertips on the middle of the thumb tab's top, then driven PRESS straight down into it.
    hold = (np.array(T.carton_point(edge - T.BASE_TAB_DEPTH / 2, T.BASE_TAB_Y, T.BASE_H)) - LEFT_HOLD[:, 2] * TIP
            - [0.0, 0.0, PRESS])
    eggs = [np.array(T.carton_point(x, y, T.EGG_TOP - CAP)) for x, y in T.CELLS]
    # Furthest column from the basket first; in each column the egg nearer the arms first, so that no egg is ever
    # carried over one still standing.
    order = sorted(range(T.N), key=lambda i: (-round(eggs[i][1], 4), eggs[i][0]))
    lane = T.carton_point(T.CARTON_D / 2, 0.0)[0] - FRONT
    # Each egg is stood on the cloth and laid down toward +y; it ends up centred LAY_SHIFT further along. The row of
    # three is filled from the +y end, so the hand never works beside an egg already lying there.
    set_z = T.TOP + T.BASKET_FLOOR - SET_PRESS + T.EGG_LEN - CAP
    slots = [np.array([bx + sx * T.SLOT_DX / 2, by + 0.010 + sy * T.SLOT_DY - lay_shift(T), set_z])
             for sy in (1, 0, -1) for sx in (1, -1)]
    return dict(tab=tab, hold=hold, hold_above=hold + [0.0, 0.0, PRESS + 0.05], eggs=eggs, order=order, slots=slots,
                lane=lane,
                carry=T.TOP + CARRY,
                left_park=np.array([0.26, 0.20, T.TOP + 0.14]), right_park=np.array([0.20, -0.20, T.TOP + 0.15]))


def at(p, z) -> np.ndarray:
    return np.array([p[0], p[1], z])


def lay_shift(T) -> float:
    """How far an egg's centre moves when it rolls from standing on its end to lying on its side."""
    return T.EGG_R * math.pi / 2 + T.EGG_CYL / 2


def lay_path(T, start):
    """The tool's path while an egg standing at ``start`` (tool point) rolls over toward +y without slipping: the
    egg's lower round end rolls on the cloth and the finger pads stay on the same two points of its cap."""
    arm = T.EGG_LEN - T.EGG_R - (CAP + TIP)             # from the lower round end's centre to the pads, along the egg
    start = np.asarray(start, float)

    def point(s: float) -> np.ndarray:
        a = math.radians(LAY_END) * s
        return start + [0.0, T.EGG_R * a + arm * math.sin(a), arm * (math.cos(a) - 1.0)]

    return point


def open_lid(T, P, left, right, hold_base: bool = True):
    tab0 = P["tab"](0.0)
    reach = [chain(right.move(tab0 + [0, 0, 0.06], 2.0, rot=RIGHT_Y, grip=gap(TAB_OPEN)), right.move(tab0, 1.0))]
    if hold_base:
        reach.append(chain(left.move(P["hold_above"], 2.0, rot=LEFT_HOLD, grip=0.0), left.move(P["hold"], 1.2)))
    yield from together(*reach)
    yield from chain(wait(0.3), right.set_grip(gap(TAB_GRIP)), wait(0.3))
    yield from right.path(lambda s: P["tab"](8.0 * s), 1.6)                       # slowly through the snap
    yield from right.path(lambda s: P["tab"](8.0 + (ARC_END - 8.0) * s), 3.6)
    yield from chain(right.set_grip(gap(0.070), 0.5), wait(0.5))                  # the lid falls open
    yield from right.move(right.pos + [0.03, 0.0, 0.0], 0.6)                      # back away from the falling lid


def move_egg(T, P, right, i: int, slot: int, drop: bool = False):
    egg, place, z = P["eggs"][i], P["slots"][slot].copy(), P["carry"]
    if drop:
        place[2] += DROP_EXTRA
    yield from right.move(at(egg, z), 1.5, rot=RIGHT_X, grip=gap(EGG_OPEN))
    yield from right.move(egg, 0.9)
    yield from chain(right.set_grip(gap(EGG_GRIP), 0.5), wait(0.2))
    yield from right.move(at(egg, z), 1.0)
    yield from right.move([P["lane"], egg[1], z], 0.9)
    yield from right.move([P["lane"], place[1], z], 1.8)
    yield from chain(right.move(at(place, z), 0.9), wait(0.4))          # let the hand settle before the egg lands
    yield from right.move(place, 1.0)
    if drop:
        yield from chain(right.set_grip(gap(EGG_RELEASE), 0.5), wait(0.3))
    else:
        yield from chain(wait(0.2), right.path(lay_path(T, place), 1.6), wait(0.2),
                         right.set_grip(gap(EGG_RELEASE), 0.5), wait(0.3))
    yield from right.move(at(right.pos, z), 0.7)


def script(T, left, right, mode: str = "unpack"):
    P = poses(T)
    yield from wait(0.5)
    if mode == "one-hand":
        yield from open_lid(T, P, left, right, hold_base=False)
        yield from right.move(P["right_park"], 1.5)
        return
    yield from open_lid(T, P, left, right)
    away = chain(left.move(P["hold_above"], 1.0), left.move(P["left_park"], 1.5))
    if mode == "lid":
        yield from together(away, right.move(P["right_park"], 1.5))
        return
    yield from away
    for slot, i in enumerate(P["order"]):
        if slot == T.N - 2:                                  # one column left: hold the base down again
            yield from chain(left.move(P["hold_above"], 1.5, rot=LEFT_HOLD, grip=0.0), left.move(P["hold"], 1.2))
        yield from move_egg(T, P, right, i, slot, drop=mode == "drop")
    yield from together(chain(left.move(P["hold_above"], 1.0), left.move(P["left_park"], 1.5)),
                        right.move(P["right_park"], 1.5))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "unpack", every: int = 20) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="egg-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode)
    frames: dict[str, list] = {}
    trace = []

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, emb.task.names)

    def hands() -> dict:
        """Where each gripper really is and how far its fingers really are apart (for the result file only)."""
        from eggkit import _np, finger_points

        out = {}
        for name, point in zip(("left", "right"), finger_points(emb._env)):
            robot = emb._env.unwrapped.scene[f"{name}_robot"]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out[name] = {"tool_cm": [round(100 * float(v), 2) for v in point],
                         "gap_mm": round(1000 * sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8")), 1)}
        return out

    snap(obs)
    start = T.report(states())
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % every == 0:
            r, h = T.report(states()), hands()
            trace.append({"step": steps, "lid_deg": r["lid_deg"], "carton_tilt_deg": r["carton_tilt_deg"],
                          "carton_pos_cm": r["carton_pos_cm"], "score": info.get("score"),
                          "eggs": "".join(e["where"][0] + ("x" if e["broken"] else "") for e in r["eggs"].values()),
                          "out": {n: {"pos_cm": e["pos_cm"], "lean_deg": e["lean_deg"], "hit": e["hardest_hit_m_s"]}
                                  for n, e in r["eggs"].items() if e["where"] != "carton"},
                          "left": h["left"], "right": h["right"],
                          "want": {x.name: {"tool_cm": [round(100 * float(v), 2) for v in x.pos],
                                            "gap_mm": round(1000 * x.grip * FULL_OPEN, 1)} for x in (left, right)}})
        if steps % 100 == 0 and trace:
            print(f"step {steps} score {info.get('score')} lid {trace[-1]['lid_deg']} eggs {trace[-1]['eggs']}",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": T.report(states()), "trace": trace}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--one-hand", action="store_true", help="control: pull the lid's tab without holding the base")
    ap.add_argument("--drop", action="store_true", help="control: let each egg go 6 cm above the cloth")
    ap.add_argument("--lid-only", action="store_true", help="stop once the lid is open")
    ap.add_argument("--trace-every", type=int, default=20, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, every=a.trace_every,
        mode="one-hand" if a.one_hand else "drop" if a.drop else "lid" if a.lid_only else "unpack")
