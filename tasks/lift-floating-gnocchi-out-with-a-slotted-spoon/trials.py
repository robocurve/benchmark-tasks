#!/usr/bin/env python3
"""Checks of the gnocchi task in the simulator with nobody touching anything: does the scene rest, do the pieces
float up when they are done and stay afloat, how far do pan and hob move by themselves, and can the floating
pieces and the hob's lamp be seen by the top camera at the size a policy gets.

    scripts/rta simpy <id> mech/tasks/gnocchi_lift/trials.py /workspace/robocurve/outputs/<id> [rest,cue,lamp,hide,float] [window=<from>,<to>[,<late>]]

``rest``   2 s: how far each body has moved from where the reset put it; the three cameras' pictures.
``lamp``   the hob's lamp in the top camera's picture as the heat is switched off and on.
``hide``   which shapes ``mechkit.show`` takes out of the picture (the key on the hob's glass it does not).
``cue``    the cue check: half the pieces made to float and half left on the floor, seen by the top camera at
           320 x 176; the same with all down and all up; the lamp lit and out.
``scoop``  four pieces made to float; the right arm takes the spoon, lifts each from where the simulator has it,
           carries it to the plate and tips it out (a mechanics trial: it reads the simulator, not a picture).
``dig``    can the spoon get a piece off the pan's floor? One piece, three tilts of the bowl.
``float``  the trial as the seed draws it, until 10 s after the last piece is due at the surface: when each piece
           is done and when it reached the surface (inside the task's window? as long on the way up as the task
           file's RISE_TRAVEL_S says?), how it floats at the end, the most the pan moved and tilted.
``window=<from>,<to>[,<late>]``: other timing numbers than the task file's, for a what-if (task.set_timing).

Every trial writes <out dir>/<trial>.json; ``rest`` and ``cue`` also their pictures.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # demo_kit

FLOAT_TAIL_S = 10.0  # the float trial goes on this long after the last piece is due at the surface
NEVER = 1e9


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _reset(embodiment):
    from inspect_robots import Scene

    m = embodiment.task.module
    return embodiment.reset(Scene(id="trial", instruction=m.INSTRUCTION), seed=0)


def _save(out: Path, name: str, frame: np.ndarray) -> np.ndarray:
    """Write a camera frame as rendered and at the policy's size; returns the latter (RGB)."""
    import cv2
    import demo_kit as kit

    small = kit.policy_picture(frame)
    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
    return small


def patches(picture: np.ndarray, reference: np.ndarray, by: int = 30, least: int = 3) -> list[dict]:
    """The separate patches where ``picture`` differs from ``reference`` by more than ``by`` in some channel:
    their sizes in pixels, middles and mean colours, largest first."""
    import cv2

    changed = (np.abs(picture.astype(int) - reference.astype(int)).max(axis=-1) > by).astype(np.uint8)
    count, labels, stats, middles = cv2.connectedComponentsWithStats(changed, connectivity=8)
    found = [
        {"pixels": int(stats[k, cv2.CC_STAT_AREA]), "at": [round(float(v), 1) for v in middles[k]], "colour": [int(v) for v in picture[labels == k].mean(axis=0)]}
        for k in range(1, count) if stats[k, cv2.CC_STAT_AREA] >= least
    ]  # fmt: skip
    return sorted(found, key=lambda p: -p["pixels"])


def red_pixels(picture: np.ndarray) -> int:
    r, g, b = (picture[..., k].astype(int) for k in range(3))
    return int(((r > 150) & (g < 0.5 * r) & (b < 0.5 * r)).sum())


def afloat(m) -> list[dict]:
    """How each piece lies in the water now: its top over the surface, its tilt, its distance from the pan's
    middle, its speed."""
    mk = m.mk
    pan_p, pan_r = mk.pose("pan")
    rows = []
    for name in m.NAMES:
        position, rotation = mk.pose(name)
        heights = (m.OCTANTS @ rotation.T + position - pan_p) @ pan_r[:, 2]
        local = pan_r.T @ (position - pan_p)
        rows.append({
            "top over the surface (mm)": round(1000 * (float(heights.max()) + m.PART_H / 2 - m.SURFACE), 2),
            "tilt (deg)": round(math.degrees(m.tilt(pan_r.T @ rotation)), 1),
            "from the pan's middle (mm)": round(1000 * math.hypot(local[0], local[1]), 1),
            "speed (mm/s)": round(1000 * float(np.linalg.norm(mk.velocity(name)[0])), 1),
        })  # fmt: skip
    return rows


def rest(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    observation = _reset(embodiment)
    for cam in ("top_cam", "left_cam", "right_cam"):
        _save(out, f"setup_{cam}", observation.images[cam])
    step = None
    for _ in range(40):
        step = embodiment.step(_hold())
    moved = {}
    for name in mk.MECH.bodies:
        position, rotation = mk.pose(name)
        built_p, built_r = mk.built_pose(name)
        if name in m.NAMES:  # the reset has put each piece at a place of its own
            k = m.NAMES.index(name)
            built_p = np.array([m.PAN_XY[0] + m.STATE["place"][k][0], m.PAN_XY[1] + m.STATE["place"][k][1], built_p[2]])
            built_r = mk.quat_to_matrix(mk.yaw_quat(float(m.STATE["yaw"][k])))
        turn = math.degrees(math.acos(float(np.clip((np.trace(built_r.T @ rotation) - 1) / 2, -1.0, 1.0))))
        moved[name] = {"moved (mm)": round(1000 * float(np.linalg.norm(position - built_p)), 2), "turned (deg)": round(turn, 2),
                       "speed (mm/s)": round(1000 * float(np.linalg.norm(mk.velocity(name)[0])), 2)}  # fmt: skip
    spoon_p, spoon_r = mk.pose("spoon")
    result = {"after 2 s": moved, "pieces": afloat(m), "report": m.report(), "info": {k: float(v) for k, v in step.info.items()},
              "spoon": {"position": [round(float(v), 5) for v in spoon_p], "quat (x, y, z, w)": [round(float(v), 5) for v in mk.matrix_to_quat(spoon_r)],
                        "its x (nose to handle end) in the world": [round(float(v), 4) for v in spoon_r[:, 0]], "handle's ends over the bench (mm)": [round(1000 * float(end[2] - m.TOP), 1) for end in m.spoon_handle()]}}  # fmt: skip
    _save(out, "rest", step.observation.images["top_cam"])
    print(f"REST {json.dumps(result, default=str)}", flush=True)
    return result


def cue(embodiment, out: Path) -> dict:
    """Can the floating pieces be counted, and the lamp's state be seen, in the top camera's picture at the size a
    policy gets? Half the pieces (drawn with a fixed seed) are done at 2 s, the others not before the second half."""
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    _reset(embodiment)
    first = np.sort(np.random.default_rng(1).permutation(m.PIECES)[: m.PIECES // 2])
    m.STATE["done_at"][:] = NEVER
    m.STATE["done_at"][first] = 2.0
    pictures, result = {}, {"first half": [int(k) for k in first]}
    step = None

    def run(seconds: float):
        nonlocal step
        for _ in range(int(round(seconds * hz))):
            step = embodiment.step(_hold())

    run(1.5)
    pictures["all down"] = _save(out, "cue_all_down", step.observation.images["top_cam"])
    run(10.5)
    pictures["half up"] = _save(out, "cue_half_up", step.observation.images["top_cam"])
    result["half up"] = {"floated": [int(k) for k in np.flatnonzero(~np.isnan(m.STATE["floated"]))], "pieces": [afloat(m)[k] for k in first]}
    m.STATE["done_at"][:] = np.minimum(m.STATE["done_at"], mk.now())
    run(10.0)
    pictures["all up"] = _save(out, "cue_all_up", step.observation.images["top_cam"])
    result["all up"] = {"floated": int((~np.isnan(m.STATE["floated"])).sum()), "pieces": afloat(m)}
    m.switch_heat()  # the task sinks the lamp into the hob at the next step
    run(1.0)
    pictures["lamp out"] = _save(out, "cue_lamp_out", step.observation.images["top_cam"])
    for name in ("half up", "all up"):
        found = patches(pictures[name], pictures["all down"])
        result[name]["patches against all down"] = found
        result[name]["patches"] = len(found)
    water = patches(pictures["all up"], pictures["all down"])
    result["red pixels, lamp lit / out"] = [red_pixels(pictures["all up"]), red_pixels(pictures["lamp out"])]
    result["patches the lamp changes"] = patches(pictures["all up"], pictures["lamp out"])
    result["the water where no piece is (mean R, G, B)"] = [int(v) for v in pictures["all down"][np.abs(pictures["all up"].astype(int) - pictures["all down"].astype(int)).max(axis=-1) > 30].mean(axis=0)] if water else None
    print(f"CUE {json.dumps(result, default=str)}", flush=True)
    return result


def lamp(embodiment, out: Path) -> dict:
    """The lamp in the top camera's picture as the heat is switched off and on again, twice (``switch_heat``, as
    the key does it): red pixels at the policy's size 1, 2, 5 and 20 control steps after each switch, and how
    much of the rest of the picture differs from the picture before the switch at each of those steps (the
    renderer is told to forget its earlier frames for two steps: what does that do to the picture?)."""
    m = embodiment.task.module
    _reset(embodiment)
    result, step = {}, None
    for _ in range(20):
        step = embodiment.step(_hold())
    reference = step.observation.images["top_cam"]
    _save(out, "lamp_lit", reference)
    result["lit"] = {"red pixels": red_pixels(_policy(reference))}
    for turn in ("out 1", "lit 2", "out 2", "lit 3"):
        m.switch_heat()
        rows, before = {}, step.observation.images["top_cam"]
        for k in range(1, 21):
            step = embodiment.step(_hold())
            if k in (1, 2, 3, 5, 20):
                frame = step.observation.images["top_cam"]
                rows[f"{k} steps after"] = {"red pixels": red_pixels(_policy(frame)), "pixels (640 x 360) that differ from before by more than 25": differing(frame, before, 25),
                                           "by more than 60": differing(frame, before, 60)}  # fmt: skip
                if k in (1, 2, 20):
                    _save(out, f"lamp_{turn.replace(' ', '_')}_{k}", frame)
        result[turn] = rows
        print(f"LAMP {turn}: {rows}", flush=True)
    return result


def _policy(frame: np.ndarray) -> np.ndarray:
    import demo_kit as kit

    return kit.policy_picture(frame)


def lamp_moves(embodiment, out: Path) -> dict:
    """Does the top camera's picture follow the lamp's tile when it is moved? Along the glass, down into the hob
    (and then with the hob's box hidden for a moment), up into the air. Red pixels (640 x 360) and where they are;
    the renderer's settings that have to do with blending frames."""
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    _reset(embodiment)
    import carb

    settings = carb.settings.get_settings()

    def walk(prefix: str, node, found: dict) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                walk(f"{prefix}/{key}", value, found)
        elif any(word in prefix.lower() for word in ("accum", "histor", "denois", "temporal", "dlss", "/aa/", "rendermode", "taa", "nrd", "reblur", "fsr", "reset")):
            found[prefix] = node

    found: dict = {}
    for root in ("/rtx", "/rtx-transient", "/renderer"):
        try:
            walk(root, settings.get_settings_dictionary(root).get_dict(), found)
        except Exception as error:  # noqa: BLE001 - a root that is not there
            found[root] = f"not read: {error}"
    result = {"settings": found}
    home = np.array([m.HOB_AT[0] + m.LAMP_XY[0], m.HOB_AT[1] + m.LAMP_XY[1], m.TOP + m.HOB_H + 0.0003])
    place = {"at": home.copy()}
    m.lamp_place = lambda: place["at"]  # the task writes the lamp there every step
    step = None

    def look(tag: str, seconds: float = 1.0) -> None:
        nonlocal step
        for _ in range(int(round(seconds * hz))):
            step = embodiment.step(_hold())
        frame = step.observation.images["top_cam"]
        _save(out, f"moves_{tag}", frame)
        r, g, b = (frame[..., k].astype(int) for k in range(3))
        red = (r > 150) & (g < 0.5 * r) & (b < 0.5 * r)
        ys, xs = np.nonzero(red)
        result[tag] = {"red pixels": int(red.sum()), "their middle": [round(float(xs.mean()), 1), round(float(ys.mean()), 1)] if red.any() else None,
                       "from": [int(xs.min()), int(ys.min())] if red.any() else None, "to": [int(xs.max()), int(ys.max())] if red.any() else None,
                       "the lamp in the simulator (mm from its place)": [round(1000 * float(v), 1) for v in mk.pose("hob_lamp")[0] - home]}  # fmt: skip
        print(f"MOVES {tag}: {result[tag]}", flush=True)

    look("1_lit")
    place["at"] = home + np.array([0.0, -0.03, 0.0])
    look("2_aside")
    place["at"] = home
    look("3_back")
    place["at"] = home + np.array([0.0, 0.0, -0.02])
    look("4_sunk")
    mk.show("hob", 0, False)
    look("5a_sunk_box_hidden", 0.25)
    mk.show("hob", 0, True)
    look("5b_sunk_box_back")
    place["at"] = home
    look("6_up_again")
    place["at"] = home + np.array([0.0, 0.0, 0.03])
    look("7_raised")
    place["at"] = home
    look("8_back")
    return result


def hide(embodiment, out: Path) -> dict:
    """Which shapes does ``mechkit.show`` take out of the top camera's picture here? One shape of each body in
    turn, hidden for 1 s and drawn again: pixels that differ from the picture before by more than 25."""
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    _reset(embodiment)
    m.STATE["done_at"][:] = 1.0
    step = None

    def picture(seconds: float) -> np.ndarray:
        nonlocal step
        for _ in range(int(round(seconds * hz))):
            step = embodiment.step(_hold())
        return step.observation.images["top_cam"]

    before = picture(5.0)
    _save(out, "hide_before", before)
    result = {}
    targets = (("hob", 1, "key"), ("pan", len(mk.MECH.bodies["pan"].shapes) - 1, "water's surface"), ("spoon", len(mk.MECH.bodies["spoon"].shapes) - 2, "a slot"),
               ("spoon", 10, "handle"), ("plate", 0, "plate's disc"), ("rest", 0, "rest"), (m.NAMES[5], 0, "a floating piece"), ("hob", 0, "hob's box"))  # fmt: skip
    for name, shape, what in targets:
        mk.show(name, shape, False)
        gone = picture(1.0)
        mk.show(name, shape, True)
        back = picture(1.0)
        tag = f"{name}_{shape}"
        _save(out, f"hide_{tag}", gone)
        result[f"{what} ({name}, shape {shape}, {mk.MECH.bodies[name].shapes[shape].kind})"] = {"pixels changed while hidden": differing(gone, before, 25), "pixels changed after it is drawn again": differing(back, before, 25)}
        print(f"HIDE {what}: {result[list(result)[-1]]}", flush=True)
    return result


def differing(a: np.ndarray, b: np.ndarray, by: int = 40) -> int:
    """Pixels whose colour differs by more than ``by`` in some channel."""
    return int((np.abs(a.astype(int) - b.astype(int)).max(axis=-1) > by).sum())


SCOOP_TARGETS = (0, 1, 2, 3)  # the pieces made to float for the scoop trial, in the order they are taken
CARRY_HEADING, TIP_HEADING, TIP_DOWN = math.radians(225), math.radians(285), math.radians(60)


def plan_scoop(layout, place, q, start, margin: float = 8.0, under: float | None = None):
    """The first scoop under a piece at ``place`` that the arm can do from joints ``q``: (moves, what was chosen)
    or (None, why not). Tried in order: nose down 10, 15, 20 degrees; the piece 30 or 20 mm nose-side of the bowl's
    middle; handle headings from 225 degrees outwards."""
    import skim

    under = skim.UNDER if under is None else under
    tried = 0
    for pitch in (10, 15, 20):
        for ahead in (0.03, 0.02):
            for heading in sorted(range(150, 301, 15), key=lambda h: abs(h - 225)):
                poses = layout.scoop_poses(place, math.radians(heading), math.radians(pitch), ahead, under)
                if min(layout.spoon_gap(poses[k]) for k in ("in", "under")) < 0.001 or min(layout.spoon_gap(poses[k]) for k in ("over", "out", "level")) < skim.CLEAR:
                    continue
                tried += 1
                moves = layout.scoop(place, math.radians(heading), math.radians(pitch), ahead, under)
                got = layout.check(moves, start_q=q, start=start)
                if layout.passes(got, margin=margin):
                    return moves, {"heading (deg)": heading, "nose down (deg)": pitch, "piece ahead of the bowl's middle (mm)": int(ahead * 1000), "joint margin (deg)": got["margin_deg"][0],
                                   "arm's gap (mm)": got["arm_gap_mm"][0], "spoon's gap (mm)": got["spoon_gap_mm"][0]}  # fmt: skip
    return None, f"no way among {tried} that fit the pan"


def scoop_clear(embodiment, out: Path) -> dict:
    """The scoop trial with the pan's floor cleared: the twelve pieces that never float are put on the bench
    beside the hob at the start."""
    return scoop(embodiment, out, clear=True)


def scoop(embodiment, out: Path, clear: bool = False, under: float | None = None) -> dict:
    """Can the spoon lift a floating piece and serve it? A mechanics trial, not a reference run: four pieces are
    made to float at 1 s, the others never; the right arm takes the spoon and, for each of the four in turn, reads
    from the simulator where the piece floats, goes under it, lifts, carries, tips out over the plate and comes
    back. Written per piece: what was planned, whether it left the water in the bowl, where it ended; per step,
    where the spoon and every piece are."""
    import demo_kit as kit
    import skim
    from demo import TAKE_SHORT  # what the arm falls short by where the spoon lies

    m = embodiment.task.module
    mk = m.mk
    layout = skim.Layout(m)
    flat = skim.spoon_rotation(CARRY_HEADING)
    home = skim.Pose(np.array([layout.pan[0], layout.pan[1], layout.carry_height(flat)]), flat)
    over_rest = skim.Pose(np.array([m.REST_XY[0], m.REST_XY[1], layout.carry_height(flat) + 0.03]), skim.spoon_rotation(m.SPOON_YAW))
    first = [*layout.take_spoon(TAKE_SHORT), layout.move(1.5, over_rest, label="level over the rest"), layout.move(2.0, home, label="home")]
    state = {"set": False, "queue": list(SCOOP_TARGETS), "plans": {}, "now": None, "began": {}}

    def director(t: float, arms: dict, pictures) -> None:
        if not state["set"]:
            m.STATE["done_at"][:] = NEVER
            m.STATE["done_at"][list(SCOOP_TARGETS)] = 1.0
            if clear:
                others = [k for k in range(m.PIECES) if k not in SCOOP_TARGETS]
                for n, k in enumerate(others):
                    mk.write_pose(m.NAMES[k], (0.47 + 0.05 * (n % 3), 0.22 + 0.05 * (n // 3), m.TOP + m.PIECE[2] / 2 + 0.002), (0.0, 0.0, 0.0, 1.0))
            state["set"] = True
        arm = arms["right"]
        if t < arm.seconds - 1e-6 or not state["queue"]:
            return
        k = state["queue"].pop(0)
        at = mk.pose(m.NAMES[k])[0]
        if math.hypot(at[0] - m.PAN_XY[0], at[1] - m.PAN_XY[1]) > m.PAN_R:  # it has left the pan already, with another piece
            state["plans"][k] = {"plan": "already out of the pan", "at (s)": round(t, 2)}
            return
        moves, chosen = plan_scoop(layout, (float(at[0]), float(at[1])), arm.q, arm.state(t)[:2], under=under)
        from_middle = [round(1000 * float(at[0] - m.PAN_XY[0]), 1), round(1000 * float(at[1] - m.PAN_XY[1]), 1)]
        state["plans"][k] = {"piece at (mm from the pan's middle)": from_middle, "top over the surface (mm)": round(1000 * float(at[2] + m.PIECE[2] / 2 - m.PAN_AT[2] - m.SURFACE), 1), "plan": chosen, "at (s)": round(t, 2)}
        print(f"SCOOP plan for piece {k} at {from_middle} mm: {chosen}", flush=True)
        if moves is None:
            return
        state["now"], state["began"][k] = k, t
        arm.moves.extend([*moves, *layout.serve(CARRY_HEADING, TIP_HEADING, TIP_DOWN), layout.move(1.8, home, label="back home")])

    arms_seen = {}

    def extra() -> dict:
        spoon_p, spoon_r = mk.pose("spoon")
        row = {"spoon": [round(float(v), 4) for v in spoon_p], "spoon x": [round(float(v), 4) for v in spoon_r[:, 0]], "spoon z": [round(float(v), 4) for v in spoon_r[:, 2]],
               "pan moved (mm)": round(1000 * float(np.linalg.norm((mk.pose("pan")[0] - np.array(m.PAN_AT))[:2])), 2), "pan tilt (deg)": round(math.degrees(m.tilt(mk.pose("pan")[1])), 2)}  # fmt: skip
        for k in SCOOP_TARGETS:
            row[f"piece {k}"] = [round(float(v), 4) for v in mk.pose(m.NAMES[k])[0]]
        row["others"] = [[round(float(v), 3) for v in mk.pose(m.NAMES[k])[0]] for k in range(m.PIECES) if k not in SCOOP_TARGETS]
        return row

    result = kit.run(embodiment, {"right": first}, out / ("scoop_clear" if clear else "scoop"), seed=0, director=director, extra=extra, limit_s=110.0, sheet_frames=24, crop={"top_cam": (200, 60, 520, 330)})
    rows = result["rows"]
    summary = {"plans": state["plans"], "report": result["report"], "worst_ik": result["worst_ik"], "seconds": result["seconds"],
               "pan moved (mm), most": max(r["extra"]["pan moved (mm)"] for r in rows), "pan tilt (deg), most": max(r["extra"]["pan tilt (deg)"] for r in rows)}  # fmt: skip
    plate = np.array([*m.PLATE_XY, m.TOP])
    for k in SCOOP_TARGETS:
        end = np.array(rows[-1]["extra"][f"piece {k}"])
        highest = max(r["extra"][f"piece {k}"][2] for r in rows) - m.TOP
        where = "on the plate" if np.hypot(*(end - plate)[:2]) <= m.PLATE_R and end[2] - m.TOP < 0.06 else ("in the pan" if np.hypot(end[0] - m.PAN_XY[0], end[1] - m.PAN_XY[1]) <= m.PAN_R else "elsewhere")
        summary[f"piece {k}"] = {"ended": where, "at": [round(float(v), 3) for v in end], "highest over the bench (mm)": round(1000 * highest, 1),
                                 "left the water at (s)": result["report"]["left the water at (s)"][k], "floated at (s)": result["report"]["floated at (s)"][k]}  # fmt: skip
    # how far the bowl was from the plan while the spoon was held: the spoon's origin against the planned one, at the labelled moments
    print(f"SCOOP {json.dumps(summary, default=str)}", flush=True)
    return summary


DIG_PITCHES = (27, 30, 35)  # nose down, degrees: from 26.6 on the rim of the bowl's nose is its lowest part (at 15 the handle would go through the pan's wall)


def dig(embodiment, out: Path) -> dict:
    """Can the spoon get a piece off the pan's floor (the rule about serving a piece that never floated)? A
    mechanics trial: no piece is ever done; all but one are taken out of the pan; the right arm brings the bowl
    down on the floor 7 cm behind the piece, pushes on until its nose is 6 mm from the wall, lifts and levels.
    Once for each tilt in DIG_PITCHES, each time on a piece put back at the same place; a tilt whose way does not
    pass the arm's check (skim.py; joints 3 degrees inside their limits) is not tried, so that the spoon is not
    knocked in the hand for the tries after it (that spoiled run l7s). Written per try: how high the piece came,
    where it ended, and how far the spoon had shifted in the hand."""
    import demo_kit as kit
    import skim
    from demo import TAKE_SHORT  # what the arm falls short by where the spoon lies

    m = embodiment.task.module
    mk = m.mk
    layout = skim.Layout(m)
    heading = math.radians(225)
    away = np.array([*m.AWAY, 0.0])
    flat = skim.spoon_rotation(heading)
    home = skim.Pose(np.array([layout.pan[0], layout.pan[1], layout.carry_height(flat)]), flat)
    over_rest = skim.Pose(np.array([m.REST_XY[0], m.REST_XY[1], layout.carry_height(flat) + 0.03]), skim.spoon_rotation(m.SPOON_YAW))
    first = [*layout.take_spoon(TAKE_SHORT), layout.move(1.5, over_rest, label="level over the rest"), layout.move(2.0, home, label="home")]
    place = np.array([m.PAN_XY[0] + 0.02, m.PAN_XY[1] + 0.02, m.PAN_AT[2] + m.PAN_BASE + m.PIECE[2] / 2 + 0.001])  # the one piece, 2.8 cm beyond the pan's middle
    state = {"set": False, "tries": list(DIG_PITCHES), "now": None, "log": {}, "highest": 0.0, "lowest": 1e9}

    def director(t: float, arms: dict, pictures) -> None:
        if not state["set"]:
            m.STATE["done_at"][:] = NEVER
            for n, k in enumerate(range(1, m.PIECES)):
                mk.write_pose(m.NAMES[k], (0.47 + 0.05 * (n % 3), 0.22 + 0.05 * (n // 3), m.TOP + m.PIECE[2] / 2 + 0.002), (0.0, 0.0, 0.0, 1.0))
            mk.write_pose(m.NAMES[0], place, mk.yaw_quat(heading))  # its long side across the bowl's way
            state["set"] = True
        arm = arms["right"]
        if t < arm.seconds - 1e-6:
            return
        if state["now"] is not None:
            at = mk.pose(m.NAMES[0])[0]
            held = layout.spoon_of(*arm.state(t)[:2])  # where the spoon would be if it had not shifted in the hand
            spoon_p, spoon_r = mk.pose("spoon")
            state["log"][state["now"]].update({"piece's middle came this high over the pan's floor (mm)": round(1000 * (state["highest"] - m.PAN_AT[2] - m.PAN_BASE), 1),
                                               "it ended (mm from the pan's middle; over the floor)": [round(1000 * float(at[0] - m.PAN_XY[0]), 1), round(1000 * float(at[1] - m.PAN_XY[1]), 1), round(1000 * float(at[2] - m.PAN_AT[2] - m.PAN_BASE), 1)],
                                               "the bowl's middle from where the hand's pose puts it, afterwards (mm)": round(1000 * float(np.linalg.norm(spoon_p - held.origin)), 1),
                                               "the bowl's lowest point over the pan's floor, least (mm)": round(1000 * state["lowest"], 1)})  # fmt: skip
            print(f"DIG nose down {state['now']}: {state['log'][state['now']]}", flush=True)
            state["now"] = None
        if not state["tries"]:
            return
        pitch = state["tries"].pop(0)
        mk.write_pose(m.NAMES[0], place, mk.yaw_quat(heading))
        state["highest"] = 0.0
        rot = skim.spoon_rotation(heading, math.radians(pitch))
        low = layout.top + layout.floor + 0.0015 + layout.lowest(rot)  # the bowl's lowest corner 1.5 mm over the pan's floor
        reach = float(((layout.bowl @ rot.T)[:, :2] @ np.array(m.AWAY)).max())  # how far the bowl reaches ahead of its middle
        start = np.array([place[0], place[1], low]) - 0.07 * away
        end_r = m.PAN_R - 0.006 - reach  # the bowl's middle this far from the pan's middle when its nose is 6 mm from the wall
        end = np.array([m.PAN_XY[0], m.PAN_XY[1], low]) + end_r * away
        high = layout.carry_height(rot)
        moves = [
            layout.move(1.5, skim.Pose(np.array([start[0], start[1], high]), rot), label="over the water"),
            layout.move(1.2, skim.Pose(start, rot), label="down to the floor"),
            layout.move(3.0, skim.Pose(end, rot), label="along the floor"),
            layout.move(1.5, skim.Pose(np.array([end[0], end[1], high]), rot), label="lift out"),
            layout.move(1.0, skim.Pose(np.array([end[0], end[1], layout.carry_height(flat)]), flat), label="level"),
            layout.move(1.5, home, label="home"),
        ]
        got = layout.check(moves, start_q=arm.q, start=arm.state(t)[:2])
        state["log"][pitch] = {"joint margin (deg)": got["margin_deg"], "spoon's gap (mm)": got["spoon_gap_mm"], "arm's gap (mm)": got["arm_gap_mm"], "ik (mm, deg)": [got["ik_mm"], got["ik_deg"]],
                               "nose rim over the floor at that tilt (mm)": round(1000 * (float((rot @ np.array([-m.BOWL_L / 2, 0.0, 0.0]))[2]) + low - layout.top - layout.floor), 1)}  # fmt: skip
        if not layout.passes(got, margin=3.0):
            state["log"][pitch]["tried"] = False
            print(f"DIG nose down {pitch}: not tried, the way does not pass the arm's check: {state['log'][pitch]}", flush=True)
            return
        state["now"], state["lowest"] = pitch, 1e9
        arm.moves.extend(moves)

    def extra() -> dict:
        at = mk.pose(m.NAMES[0])[0]
        state["highest"] = max(state["highest"], float(at[2]))
        spoon_p, spoon_r = mk.pose("spoon")
        if state["now"] is not None:
            state["lowest"] = min(state["lowest"], float((layout.bowl @ spoon_r.T + spoon_p)[:, 2].min()) - layout.top - layout.floor)
        return {"piece": [round(float(v), 4) for v in at], "spoon": [round(float(v), 4) for v in spoon_p], "spoon x": [round(float(v), 4) for v in spoon_r[:, 0]],
                "pan moved (mm)": round(1000 * float(np.linalg.norm((mk.pose("pan")[0] - np.array(m.PAN_AT))[:2])), 2)}  # fmt: skip

    result = kit.run(embodiment, {"right": first}, out / "dig", seed=0, director=director, extra=extra, limit_s=60.0, sheet_frames=24, crop={"top_cam": (200, 60, 520, 330)})
    summary = {"tries": state["log"], "pan moved (mm), most": max(r["extra"]["pan moved (mm)"] for r in result["rows"]), "report": {k: v for k, v in result["report"].items() if k in ("taken out raw", "zero", "score")}}
    print(f"DIG {json.dumps(summary, default=str)}", flush=True)
    return summary


def float_up(embodiment, out: Path) -> dict:
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    _reset(embodiment)
    embodiment.step(_hold())  # poses can be read from the first step on
    pan_from, hob_from = mk.pose("pan")[0].copy(), np.array(m.HOB_AT)
    most = {"pan moved (mm)": 0.0, "pan tilted (deg)": 0.0, "hob moved (mm)": 0.0, "a piece's speed (mm/s)": 0.0, "a piece from the pan's middle (mm)": 0.0, "a piece's top over the surface (mm)": -1e9}
    step = None
    came_up: dict[int, np.ndarray] = {}  # where each piece was when it reached the surface
    wandered = np.zeros(m.PIECES)  # the farthest it has been from there since
    least_inside = np.full(m.PIECES, np.inf)  # the least it has been inside the region's straight edge since
    run_s = m.FLOAT_TO + FLOAT_TAIL_S
    for _ in range(int(round(run_s * hz))):
        step = embodiment.step(_hold())
        pan_p, pan_r = mk.pose("pan")
        rows = afloat(m)
        for k in range(m.PIECES):
            if np.isnan(m.STATE["floated"][k]):
                continue
            at = (mk.pose(m.NAMES[k])[0] - pan_p)[:2]
            came_up.setdefault(k, at.copy())
            wandered[k] = max(wandered[k], float(np.linalg.norm(at - came_up[k])))
            least_inside[k] = min(least_inside[k], float(at @ np.array(m.AWAY)) - m.REGION_EDGE)
        for key, value in (("pan moved (mm)", 1000 * float(np.linalg.norm((pan_p - pan_from)[:2]))), ("pan tilted (deg)", math.degrees(m.tilt(pan_r))),
                           ("hob moved (mm)", 0.0), ("a piece's speed (mm/s)", max(r["speed (mm/s)"] for r in rows)),
                           ("a piece from the pan's middle (mm)", max(r["from the pan's middle (mm)"] for r in rows)), ("a piece's top over the surface (mm)", max(r["top over the surface (mm)"] for r in rows))):  # fmt: skip
            most[key] = round(max(most[key], value), 2)
    done, floated = m.STATE["done_at"], m.STATE["floated"]
    rises = [float(f - d) for d, f in zip(done, floated) if not np.isnan(f)]
    step_s = 1.0 / hz  # a time is read once per control step
    result = {
        "ran (s)": run_s,
        "window (s): the pieces reach the surface from, to; a floating piece may stay": [m.FLOAT_FROM, m.FLOAT_TO, m.LATE_S],
        "done at (s)": [round(float(v), 2) for v in done],
        "reached the surface at (s)": [None if np.isnan(f) else round(float(f), 2) for f in floated],
        "reached the surface after (s)": [None if np.isnan(f) else round(float(f - d), 2) for d, f in zip(done, floated)],
        "all twelve came up": len(rises) == m.PIECES,
        "came up inside the window (to a control step)": bool(len(rises) == m.PIECES and np.all((floated >= m.FLOAT_FROM - step_s) & (floated <= m.FLOAT_TO + step_s))),
        "on the way up (s): least, most, the task file's RISE_TRAVEL_S": [round(min(rises), 2), round(max(rises), 2), m.RISE_TRAVEL_S] if rises else None,
        "RISE_TRAVEL_S holds (within a control step)": bool(rises and max(abs(r - m.RISE_TRAVEL_S) for r in rises) <= step_s),
        "at the end": afloat(m),
        "the most in the run": most,
        "wandered from where it came up (mm), most, by piece": [round(1000 * float(v), 1) for v in wandered],
        "left its start place on the way up (mm), by piece": [None if k not in came_up else round(1000 * float(np.linalg.norm(came_up[k] - m.STATE["place"][k])), 1) for k in range(m.PIECES)],
        "least distance inside the region's edge after coming up (mm), by piece": [None if not np.isfinite(v) else round(1000 * float(v), 1) for v in least_inside],
        "report": m.report(),
        "info": {k: float(v) for k, v in step.info.items()},
    }
    _save(out, "float_end", step.observation.images["top_cam"])
    print(f"FLOAT {json.dumps(result, default=str)}", flush=True)
    return result


def main() -> int:
    import demo_kit as kit

    args = [a for a in sys.argv[1:] if not a.startswith("window=")]
    windows = [a for a in sys.argv[1:] if a.startswith("window=")]
    out = Path(args[0])
    known = {"rest": rest, "cue": cue, "lamp": lamp, "lamp_moves": lamp_moves, "hide": hide, "scoop": scoop, "scoop_clear": scoop_clear, "dig": dig, "float": float_up}
    trials = args[1].split(",") if len(args) > 1 else list(known)
    if not trials or any(name not in known for name in trials):
        raise SystemExit(f"trials here: {sorted(known)}; got {trials}")
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    if windows:  # a what-if: other timing numbers than the task file's
        m = embodiment.task.module
        print(f"TIMING what-if {windows[-1]}: {m.set_timing(*(float(v) for v in windows[-1].split('=', 1)[1].split(',')))}", flush=True)
    for name in trials:
        result = known[name](embodiment, out)
        (out / f"{name}.json").write_text(json.dumps(result, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
