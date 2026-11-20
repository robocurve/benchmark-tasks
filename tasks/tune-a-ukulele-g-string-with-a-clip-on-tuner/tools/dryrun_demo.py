"""Dry run of the scripted demonstration without the simulator, scored by the real rules and scorer.

The arms follow their joint commands with a first-order lag (no contact physics). A button follows a right gripper
closed on it: it turns with the fingers' roll about its shaft. The G string's stretch is pressed down by the left
hand's closed fingertip while it is over the string, the arm giving ARM_COMPLIANCE
metres per newton of the string's spring, and it springs back within a step once the fingertips are above it. The
pitch rule, the pluck rule, the tuner and the scorer are the real ones (``tuning.py``, ``grading.py``), fed what the
simulator would feed them, and the demonstration gets the state it gets in the simulator.

This checks reach (inverse kinematics on the real arm model), joint limits, the control flow, the regrips (the wrist
roll of every grip) and the step count, and, as a kinematic proxy for collisions, how close the YAM's own collision
hulls (link 5, gripper and fingers, from its URDF, sampled over their surfaces) come to the ukulele (``HandHulls``):
for the left hand, the body (a joint-space move from the ready pose would put its fingertips on the body's upper
edge) and the C string over the soundhole; for the right hand, the neighbouring button on the headstock's upper edge,
the headstock, neck, posts and gear boxes, the strings between the nut and the posts and over the first frets; and
the tuner, for both. It says nothing about contact forces, friction or whether the pads hold the button: it does
not replace a run in the simulator.

Run:  python tools/dryrun_demo.py [--yam-arena <yam-isaac-lab-arena checkout>] [--verbose]
Needs NumPy and yam_arena (its kinematics need no Isaac Sim).
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARM_COMPLIANCE = .004         # m/N sideways at the hand (about 4 mm/N, an estimate for the YAM held out like this)
RETURN_TAU = .02              # s: a released string is back within 0.5 mm by the next step
SURFACE_SPACING = .002        # m: the hulls are sampled over their surfaces about this finely (distances good to
                              # 1 to 2 mm)
NECK_STRINGS = .060           # m: the strings are checked from the nut this far along the neck (for the right hand)
HEAD_REACH = .030             # m: clearances near the headstock are measured up to this; "30" means 30 or more


def _stl_triangles(path):
    """Triangles (n, 3, 3) of a binary STL file."""
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data[84:84 + record.itemsize * count], dtype=record)["vertices"].astype(float)


def _surface(triangles, spacing):
    """Points over the triangles' surfaces: their vertices, points along every edge about ``spacing`` apart, and a
    grid inside each triangle with about one point per ``spacing`` squared of its area (a thin sliver adds little
    more than its edges)."""
    points = [triangles.reshape(-1, 3)]
    for a, b in ((0, 1), (1, 2), (2, 0)):
        p, q = triangles[:, a], triangles[:, b]
        count = np.ceil(np.linalg.norm(q - p, axis=1) / spacing).astype(int)
        for n in np.unique(count[count > 1]):
            pick = count == n
            t = np.arange(1, n) / n
            points.append((p[pick, None, :] + t[None, :, None] * (q - p)[pick, None, :]).reshape(-1, 3))
    area = .5 * np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    count = np.ceil(1.52 * np.sqrt(area) / spacing).astype(int)
    for n in np.unique(count[count > 2]):
        i, j = np.mgrid[1:n, 1:n]
        keep = i + j < n
        u, v = i[keep] / n, j[keep] / n
        tri = triangles[count == n]
        a, ab, ac = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
        points.append((a[:, None, :] + u[None, :, None] * ab[:, None, :] + v[None, :, None] * ac[:, None, :]).reshape(-1, 3))
    return np.unique(np.round(np.concatenate(points), 6), axis=0)


class HandHulls:
    """The YAM's collision hulls near the hand (link 5, the gripper and both fingers), read from its URDF, sampled
    over their surfaces and posed by forward kinematics. A kinematic proxy for collisions with the ukulele: where the
    hulls are, not what contact would do."""

    LINKS = ("link5", "gripper", "tip_left", "tip_right")

    def __init__(self, spacing=SURFACE_SPACING):
        from yam_arena import YAM_URDF
        from yam_arena.urdf import rpy_matrix

        self.urdf = Path(str(YAM_URDF))
        self.hulls = []
        for link in ET.parse(self.urdf).getroot().iter("link"):
            if link.get("name") not in self.LINKS:
                continue
            for collision in link.findall("collision"):
                origin = collision.find("origin")
                xyz = np.array([float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()])
                rpy = np.array([float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()])
                triangles = _stl_triangles(self.urdf.parent / collision.find("geometry/mesh").get("filename"))
                points = _surface(triangles, spacing)
                self.hulls.append((link.get("name"), points @ rpy_matrix(rpy).T + xyz))

    def points(self, arm_joints, base, finger):
        """Every sample point in the tabletop frame; ``finger``: each finger joint's position (0 closed)."""
        from yam_arena.constants import ARM_JOINTS
        from yam_arena.urdf import forward_kinematics

        q = {name: float(v) for name, v in zip(ARM_JOINTS, arm_joints)}
        poses = forward_kinematics(self.urdf, {**q, "joint7": float(finger), "joint8": float(finger)})
        return np.concatenate([v @ poses[link][:3, :3].T + poses[link][:3, 3] for link, v in self.hulls]) + base


def _instrument(points):
    """Tabletop-frame points in the instrument frame (``fixed_scene.to_instrument``), as columns x, y, z."""
    from rdb_ukulele_tuning import fixed_scene as fixture

    d = points - np.asarray(fixture.ORIGIN)
    return d[:, 2], d[:, 1], -d[:, 0]


def _box(q):
    """Signed distance to a box from points given as their offsets past its half extents (|p - c| - half)."""
    return np.linalg.norm(np.maximum(q, 0.), axis=1) + np.minimum(np.max(q, axis=1), 0.)


def _segment(p, a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0., 1.)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def _slab(x, y, z, y0, y1, half_width, z0, z1):
    """Signed distance (approximate off the corners) to a slab between y0 and y1 whose half-width varies with y."""
    q = np.stack((np.abs(x) - half_width, np.maximum(y0 - y, y - y1), np.maximum(z0 - z, z - z1)), axis=1)
    return _box(q)


def body_clearances(points):
    """(distance to the ukulele's body, distance to the C string near the soundhole), metres, from tabletop-frame
    points: the body as the slab from its back to the soundboard inside its outline (negative inside it); the C
    string (the G string's neighbour, below it) as its line over the soundhole. Points off the body's length do not
    count for the first (the right hand works on the headstock)."""
    from rdb_ukulele_tuning import layout as L

    x, y, z = _instrument(points)
    outline = np.array(L.BODY_OUTLINE)
    w = np.interp(y, outline[:, 0], outline[:, 1], left=0., right=0.)
    on = w > 0.
    outside = np.sqrt(np.maximum(np.abs(x) - w, 0.) ** 2 + np.maximum(z - L.TOP_Z, 0.) ** 2
                      + np.maximum(L.BACK_Z - z, 0.) ** 2)
    inside = np.minimum.reduce([w - np.abs(x), L.TOP_Z - z, z - L.BACK_Z])
    body = np.where((np.abs(x) < w) & (z > L.BACK_Z) & (z < L.TOP_Z), -inside, outside)
    body = float(body[on].min()) if on.any() else math.inf
    near = np.abs(y - L.PLUCK_Y) < .04
    if near.any():
        c = L.string_point("c", L.PLUCK_Y)
        string = float((np.hypot(x[near] - c[0], z[near] - c[2]) - L.STRING_R).min())
    else:
        string = math.inf
    return body, string


def head_clearances(points, held, buttons):
    """Distances (metres, negative inside, at most HEAD_REACH) from tabletop-frame points to the parts near the
    buttons: ``button``: the other button on the headstock's upper edge (not ``held``, the one being turned; at its
    current angle ``buttons``); ``headstock``: the headstock, the neck's first NECK_STRINGS, the posts and the boxes
    standing in for the open gears; ``strings``: each string from the nut to its post and over the first NECK_STRINGS
    of the neck; ``tuner``: the tuner's head and clip."""
    from rdb_ukulele_tuning import layout as L

    x, y, z = _instrument(points)
    lo = np.array((-L.BUTTON_OUT, L.TIP_Y - L.CLIP_SPINE, L.AXIS_Z - L.BUTTON_WIDE / 2)) - HEAD_REACH
    hi = np.array((L.BUTTON_OUT, NECK_STRINGS, L.TOP_Z + .055)) + HEAD_REACH
    near = (x > lo[0]) & (x < hi[0]) & (y > lo[1]) & (y < hi[1]) & (z > lo[2]) & (z < hi[2])
    keys = ("button", "headstock", "strings", "tuner")
    if not near.any():
        return dict.fromkeys(keys, HEAD_REACH)
    x, y, z = x[near], y[near], z[near]
    p = np.stack((x, y, z), axis=1)
    out = {}
    # the neighbouring button: a box turned about the instrument's x with its joint
    other = next(name for name in L.STRINGS if L.SIDE[name] == L.SIDE[held] and name != held)
    c = np.asarray(L.button_centre(other))
    a = float(buttons[other])
    d = p - c
    local = np.stack((d[:, 0], math.cos(a) * d[:, 1] + math.sin(a) * d[:, 2],
                      -math.sin(a) * d[:, 1] + math.cos(a) * d[:, 2]), axis=1)
    out["button"] = float(_box(np.abs(local) - (L.BUTTON_LEN / 2, L.BUTTON_THICK / 2, L.BUTTON_WIDE / 2)).min())
    # headstock, neck, posts and gear boxes
    head = np.array(L.HEAD_OUTLINE)[::-1]                       # y rising, for interpolation
    gaps = [_slab(x, y, z, L.TIP_Y, 0., np.interp(y, head[:, 0], head[:, 1]), L.TOP_Z - L.HEAD_T, L.TOP_Z),
            _slab(x, y, z, 0., NECK_STRINGS, L.neck_width(np.clip(y, 0., NECK_STRINGS)) / 2,
                  L.neck_back_z(np.clip(y, 0., NECK_STRINGS)), L.FB_TOP)]
    for name in L.STRINGS:
        s = L.SIDE[name]
        lo, hi = sorted((s * L.HOUSING_IN, s * L.HOUSING_OUT))
        centre = np.array(((lo + hi) / 2, L.POST_Y[name], L.AXIS_Z))
        gaps.append(_box(np.abs(p - centre) - ((hi - lo) / 2, L.HOUSING[0] / 2, L.HOUSING[1] / 2)))
        radial = np.hypot(x - L.POST_X[name], y - L.POST_Y[name]) - L.POST_R
        axial = np.maximum(L.TOP_Z - z, z - (L.TOP_Z + L.POST_H))
        gaps.append(np.hypot(np.maximum(radial, 0.), np.maximum(axial, 0.)) + np.minimum(np.maximum(radial, axial), 0.))
    out["headstock"] = float(np.min([g.min() for g in gaps]))
    # strings: nut to post (drawn only in the scene, at 0.8 of the string's radius) and over the first frets
    strings = []
    for name in L.STRINGS:
        post = (L.POST_X[name] - L.SIDE[name] * L.POST_R, L.POST_Y[name], L.TOP_Z + L.POST_WRAP_Z)
        strings.append(_segment(p, L.string_point(name, 0.), post) - .8 * L.STRING_R)
        strings.append(_segment(p, L.string_point(name, 0.), L.string_point(name, NECK_STRINGS)) - L.STRING_R)
    out["strings"] = float(np.min([s.min() for s in strings]))
    # the tuner's head (a box in the screen's frame) and its clip
    across, up, normal = (np.asarray(v) for v in L.screen_frame())
    centre = np.asarray(L.TUNER_CENTRE)
    d = p - centre
    local = np.stack((d @ across, d @ up, d @ normal), axis=1)
    tuner = _box(np.abs(local) - np.asarray(L.TUNER_HEAD) / 2)
    clip = ((-L.CLIP_W / 2, L.CLIP_W / 2), (L.TIP_Y - L.CLIP_SPINE, L.CLIP_Y[1]),
            (L.TOP_Z - L.HEAD_T - L.CLIP_T, L.TUNER_CENTRE[2]))
    lo, hi = np.array([b[0] for b in clip]), np.array([b[1] for b in clip])
    out["tuner"] = float(min(tuner.min(), _box(np.abs(p - (lo + hi) / 2) - (hi - lo) / 2).min()))
    return {key: min(value, HEAD_REACH) for key, value in out.items()}


def dry_run(*, variant="full", arm_lag_s=.12, sag=(0., 0., 0.), max_steps=None, verbose=False):
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, READY

    from rdb_ukulele_tuning import fixed_scene as fixture
    from rdb_ukulele_tuning import grading, tuning
    from rdb_ukulele_tuning import layout as L
    from rdb_ukulele_tuning.scripted import FACE, TIP, TuningReference

    reference = TuningReference(variant=variant)
    actions = reference.sequence()
    dt = 1. / fixture.CONTROL_HZ
    blend = 1. - math.exp(-dt / arm_lag_s)
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    bases = {arm: np.asarray(fixture.BASES[arm], dtype=float) for arm in fixture.ARMS}
    half_open = abs(float(FINGER_OPEN))
    sag = np.asarray(sag, dtype=float)
    buttons = dict.fromkeys(L.STRINGS, 0.)
    held = {}                                    # the button held by the right hand: its roll offset
    grips = []                                   # every grip of a button: where joint 6 and the button were
    pitches = {name: tuning.Pitch(name) for name in L.STRINGS}
    detector, display, grade = tuning.PluckDetector(), tuning.TunerDisplay(), grading.TuningGrade()
    pluck, touched, shown, result = 0., False, tuning.blank(), None
    pressing, dip_before = False, -1.
    string = np.asarray(fixture.to_station(L.pluck_point()))
    report = {"worst_limit_margin_deg": 180., "events": [], "max_pluck_mm": 0.}
    hulls = HandHulls()
    keys = ("body", "c_string", "button", "headstock", "strings", "tuner")
    worst = {(arm, key): (math.inf, 0, "") for arm in fixture.ARMS for key in keys}     # (metres, step, phase)
    j6_limits = (float(ik.LOWER[5]), float(ik.UPPER[5]))
    tick = 0
    for tick in range(1, (max_steps or fixture.MAX_STEPS) + 1):
        cents = {name: pitches[name].cents for name in L.STRINGS}
        state = {"buttons": dict(buttons), "cents": cents, "play_deg": pitches[L.TUNED].play, "pluck": pluck,
                 "plucks": detector.count, "tuner": dict(shown)}
        measured = joints.copy()
        if held:                                 # a gripper closed on a button reads the button's thickness
            measured[13] = max(measured[13], L.BUTTON_THICK / (2 * half_open))
        reference.observe(measured, state)
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at step {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        joints = joints + blend * (command - joints)
        tools = [ik.tcp_pose(joints[7 * i:7 * i + 6], bases[arm]) for i, arm in enumerate(fixture.ARMS)]
        # right hand on a button standing up from the headstock: it turns with the fingers' roll about +z
        point, rot = tools[1]
        point = point + sag
        width = joints[13] * 2 * half_open
        roll = math.atan2(-rot[0, 0], rot[1, 0])
        if held:
            name, offset = next(iter(held.items()))
            if width > L.BUTTON_THICK + .003:
                held.clear()
                grips[-1].update(released_tick=tick, joint6_end_deg=round(math.degrees(joints[12]), 1),
                                 button_end_deg=round(math.degrees(buttons[name]), 1))
                report["events"].append((tick, f"let go of the {name.upper()} button at "
                                               f"{math.degrees(buttons[name]):.1f} degrees"))
            else:
                buttons[name] = tuning.unwrap(buttons[name], roll - offset)
        elif width <= L.BUTTON_THICK + .001:
            for name in ("g", "c"):
                top = np.asarray(fixture.to_station(L.button_tip(name)))
                pads = point + rot[:, 2] * TIP
                inside = top[2] - L.BUTTON_LEN - .006 < pads[2] < top[2] - .002     # the pads are 20+ mm long
                # pads 20 mm apart round a 6 mm button 18 mm across: they still close on it 6 mm off its shaft's axis
                on_axis = math.hypot(point[0] - top[0], point[1] - top[1]) < .006
                square = rot[2, 2] < -math.cos(math.radians(12.))
                twist = (roll - buttons[name] + math.pi / 2) % math.pi - math.pi / 2
                if inside and on_axis and square and abs(twist) < math.radians(20.):
                    held[name] = roll - buttons[name]
                    grips.append({"button": name, "tick": tick, "joint6_start_deg": round(math.degrees(joints[12]), 1),
                                  "button_start_deg": round(math.degrees(buttons[name]), 1)})
                    report["events"].append((tick, f"pads on the {name.upper()} button"))
                    break

        # left hand's closed, tipped fingertip on the G string's stretch: it presses the string down
        point, rot = tools[0]
        point = point + sag
        a, c = rot[:, 2], rot[:, 0]
        tip = point + a * TIP
        across = np.array((c[0], 0., c[2])) / max(np.hypot(c[0], c[2]), 1e-9)   # the end face's slope across the string
        s = (string[0] - tip[0]) / across[0] if abs(across[0]) > 1e-6 else 99.
        face_z = tip[2] + s * across[2]                    # the end face's height above the string's line
        along = abs(tip[1] - string[1]) < .0077 + L.PLUCK_LEN / 2
        over = abs(s) <= FACE
        dip = string[2] + L.STRING_R - face_z
        # pressing goes on while the fingertip stays over the string; it can only start from above it
        if along and over and dip > 0. and (pressing or dip_before <= .004):
            pressing = True
            pluck = -min(dip / (1. + L.PLUCK_STIFFNESS * ARM_COMPLIANCE), L.PLUCK_TRAVEL)
            touched = True
        else:
            pressing = False
            pluck *= math.exp(-dt / RETURN_TAU)
            touched = False
        dip_before = dip
        report["max_pluck_mm"] = max(report["max_pluck_mm"], 1000 * abs(pluck))
        # the hands' hulls against the ukulele; a hand pressing the string is where the arm gives to
        give = {"left": np.array((0., 0., dip - abs(pluck))) if pressing else np.zeros(3), "right": np.zeros(3)}
        phase = reference.phases[-1][1] if reference.phases else ""
        for i, arm in enumerate(fixture.ARMS):
            points = hulls.points(joints[7 * i:7 * i + 6], bases[arm] + sag + give[arm],
                                  joints[7 * i + 6] * float(FINGER_OPEN))
            gaps = dict(zip(("body", "c_string"), body_clearances(points)))
            gaps.update(head_clearances(points, reference.button, buttons))
            for key, gap in gaps.items():
                if gap < worst[(arm, key)][0]:
                    worst[(arm, key)] = (gap, tick, phase)

        cents = {name: pitches[name].update(buttons[name]) for name in L.STRINGS}
        plucked = detector.update(tick, pluck, touched)
        if plucked:
            report["events"].append((tick, f"pluck counted: G at {cents[L.TUNED]:.2f} cents"))
        shown = display.update(tick, plucked, cents[L.TUNED])
        result = grade.update(cents, [fixture.to_instrument(p) for p, _ in tools], plucked, pitches[L.TUNED].button)
        if verbose and tick % 20 == 0:
            print(tick, result["score"], round(cents[L.TUNED], 2), round(1000 * pluck, 2), shown["note"],
                  reference.phases[-1][1] if reference.phases else "", flush=True)
        if grade.finished:
            break
        if reference.done and tick > reference.phases[-1][0] + 3 * fixture.CONTROL_HZ:
            break
    for grip in grips:
        if "joint6_end_deg" in grip:
            grip["roll_used_deg"] = round(grip["joint6_end_deg"] - grip["joint6_start_deg"], 1)
            grip["button_turned_deg"] = round(grip["button_end_deg"] - grip["button_start_deg"], 1)
        j6 = [grip["joint6_start_deg"], grip.get("joint6_end_deg", grip["joint6_start_deg"])]
        grip["joint6_limit_margin_deg"] = round(min(min(j6) - math.degrees(j6_limits[0]),
                                                    math.degrees(j6_limits[1]) - max(j6)), 1)
    report.update(worst_body_clearance_mm=round(1000 * worst[("left", "body")][0], 1),
                  worst_c_string_clearance_mm=round(1000 * worst[("left", "c_string")][0], 1),
                  clearance_mm={f"{arm}_{key}": round(1000 * worst[(arm, key)][0], 1)
                                for arm, key in worst if math.isfinite(worst[(arm, key)][0])},
                  closest={f"{arm}_{key}": worst[(arm, key)][1:] for arm, key in worst},
                  joint6_limits_deg=[round(math.degrees(v), 1) for v in j6_limits], grips=grips)
    report.update(score=result["score"], steps=tick, grade=result, cents={k: round(v, 3) for k, v in cents.items()},
                  regrips=reference.regrips, cycles=reference.cycles, worst_ik_mm=round(1000 * reference.worst_ik, 3),
                  worst_joint6_shift_deg=round(math.degrees(reference.j6_shift), 2), phases=reference.phases,
                  log=reference.log)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena", help="a yam-isaac-lab-arena checkout, if yam_arena is not installed")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--case", default="all", choices=["all", "nominal", "sag", "wrong_button", "blind", "hold_on"])
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *((args.yam_arena,) if args.yam_arena else ())):
        sys.path.insert(0, str(path))
    cases = (("nominal", "nominal", {}),
             ("sag", "slower arm, the arms sag 2 to 4 mm", dict(arm_lag_s=.2, sag=(.002, -.002, -.004))),
             ("wrong_button", "control: the C string's button turned instead (must score 0)", dict(variant="wrong_button")),
             ("blind", "control: tuned by turning blind, never plucked after the last turn (must score 0.75)",
              dict(variant="blind")),
             ("hold_on", "control: tuned but the button never let go (must score 0.75)", dict(variant="hold_on", max_steps=1200)))
    for key, name, kw in cases:
        if args.case not in ("all", key):
            continue
        report = dry_run(verbose=args.verbose, **kw)
        log = report.pop("log")
        phases = report.pop("phases")
        events = report.pop("events")
        closest = report.pop("closest")
        grips = report.pop("grips")
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
        print("   grips:", grips)
        if args.verbose:
            print("   closest (step, phase):", closest)
            print("   phases:", phases)
            for event in events:
                print("   ", event)
            for event in log["events"]:
                print("   ", event)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
