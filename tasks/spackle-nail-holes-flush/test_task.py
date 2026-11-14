"""Offline checks of the spackle scorer and demo, no simulator. Knife states are built by hand from the same geometry
the demo uses (task.knife_rot, task.face_point), with the right gripper closed on the handle when the knife is held.
Covers every tier, every failure and adversarial cases: doing nothing, hovering, a dab, a jab straight in, a knife
pushed along without a hand on it, steep passes only (holes stay open), low passes only (coat left proud, 0.75), an
overfilled coat left on one hole, smearing the whole area, near-identical pack and strike angles (no pair closer than
14 degrees earns a flush patch), the corner gouge (capped at 0.5), the knife still held at the end, set down against
the panel (edge or blade face), left across the tub, set down but moving, the 1 s hold, a new trial, the thresholds in the text, and the demo plan.

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import pastekit as pk  # noqa: E402
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.25, M.TOP + 0.30)}
# a top-down grasp: tool z (the approach) straight down, the fingers closing along world y (across a handle along x)
GRIP_DOWN = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
GRIP_OPEN = 0.014 / 0.0475  # pads on the 28 mm handle's sides
EDGE = M.EDGE_LOCAL


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=pk.quat(np.asarray(rot)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def fresh():
    kit._EPISODE["count"] += 1
    M._L = M.Ledger()


FIXED = {"panel": st(M.PANEL_POS, M.R_PANEL), "tub": st((M.TUB_C[0], M.TUB_C[1], M.TOP)),
         "stop": st(M.STOP.pos), "brace": st(M.BRACE.pos)}


def objs(kpos, krot, held=True):
    """Objects for one step: the knife at kpos/krot; the right hand closed on its handle if held, else both away."""
    o = dict(FIXED)
    o["knife"] = st(kpos, krot)
    opening = {}
    for g, p in AWAY.items():
        o[g] = st(p, GRIP_DOWN)
        opening[g] = 1.0
    if held:
        o["right_gripper"] = st(np.asarray(kpos) + krot @ M.GRIP_LOCAL, krot @ GRIP_DOWN)
        opening["right_gripper"] = GRIP_OPEN
    return o, opening


def at_edge(edge, rot):
    """Knife position that puts its edge's middle at `edge` with rotation `rot`."""
    return np.asarray(edge, float) - rot @ EDGE


def feed(states):
    """Each state as the sim gives it: a fresh dict per step, the finger openings of that step."""
    s = 0.0
    for o, opening in states:
        kit._OPENING.update(opening)
        o = dict(o)
        M.success(o)
        s = M.score(o)
    return s


def path(a, b, step=0.002):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
    return [a + (b - a) * k / n for k in range(n + 1)]


def go(edges, rot, held=True):
    return [objs(at_edge(e, rot), rot, held) for e in edges]


def load_knife(held=True):
    """Dip the tip into the tub 15 degrees down and draw it 4 cm through the paste, then lift out."""
    r = M.knife_rot(M.DIP_PITCH)
    tub = np.array([M.TUB_C[0], M.TUB_C[1], M.TOP + M.PASTE_Z])
    d0, d1 = tub + [M.DIP_SPAN[0], 0, -M.DIP_DEPTH], tub + [M.DIP_SPAN[1], 0, -M.DIP_DEPTH]
    return go(path(d0 + [0, 0, 0.05], d0, 0.005) + path(d0, d1) + path(d1, d1 + [0, 0, 0.05], 0.005), r, held)


def face_pass(i, theta, span, d=-M.PRESS[1], held=True, tilt=0.0, dv=0.0, step=0.002):
    """A pass over hole i at `theta` from span[0] to span[1] m up the face from its center, the edge `d` out from
    the face; approach and leave square to the face, from 6 cm out."""
    _, s, y, _, _ = M.HOLE_SPECS[i]
    y += dv
    r = M.knife_rot(M.pitch_for(theta))
    if tilt:
        r = pk_rz(math.radians(tilt)) @ r
    s0, s1 = s + span[0], s + span[1]
    e = (path(M.face_point(s0, y, M.BACK), M.face_point(s0, y, d), 0.005) + path(M.face_point(s0, y, d),
         M.face_point(s1, y, d), step) + path(M.face_point(s1, y, d), M.face_point(s1, y, M.BACK), 0.005))
    return go(e, r, held)


def pk_rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pack(i, **kw):
    return face_pass(i, M.FILL_DEG, M.FILL_SPAN, **kw)


def strike(i, **kw):
    return face_pass(i, M.STRIKE_DEG, M.STRIKE_SPAN, **kw)


def rest_rot():
    return M.knife_rot(0.0, M.KNIFE_YAW)


def set_down(n=25, held=False, dz=0.0, wobble=False):
    """The knife lying back where it started, released (or still held), n steps."""
    out = []
    for k in range(n):
        p = np.asarray(M.KNIFE_START, float) + [0, 0, dz]
        if wobble:
            p = p + [0.003 * (k % 2), 0, 0]
        out.append(objs(p, rest_rot(), held))
    return out


def full_job():
    states = load_knife()
    for i in range(3):
        states += pack(i) + strike(i)
    return states


def main():
    # -- static checks of the layout ------------------------------------------------------------------------------
    check(abs(np.linalg.norm(M.NORMAL) - 1) < 1e-12 and abs(np.dot(M.NORMAL, M.UP)) < 1e-12, "face frame orthonormal")
    for _, s, y, dia, depth in M.HOLE_SPECS:
        p = M.face_point(s, y)
        check(abs((p - M.face_point(0, y)) @ M.NORMAL) < 1e-12, f"hole at {np.round(p, 3)} lies on the face")
    fr = M._L.paste._frame({"panel": (M.PANEL_POS, pk.quat(M.R_PANEL))}, M.FACE)
    check(np.allclose(fr[3], M.NORMAL) and np.allclose(fr[0], M.face_point(0, M.FACE_V0)),
          "pastekit's face frame matches the panel's face (origin at its bottom-right corner, normal toward the robot)")
    for theta in (M.FILL_DEG, M.STRIKE_DEG):
        r = M.knife_rot(M.pitch_for(theta))
        check(abs(pk.blade_angle_deg(r, M.NORMAL) - theta) < 1e-6, f"demo pitch gives a {theta:.0f} deg blade")
    check(M.KNIFE_MASS >= 0.040 and 0.025 <= M.HANDLE[1][1] <= 0.040, "knife >= 40 g, handle 25-40 mm across the grip")

    # -- doing nothing, hovering, dabbing --------------------------------------------------------------------------
    fresh()
    check(feed(set_down(40)) == 0.0, "doing nothing scores 0")
    fresh()
    s = feed(load_knife() + face_pass(0, M.FILL_DEG, M.FILL_SPAN, d=0.002) + set_down())
    check(s == 0.0 and M._L.paste.load > 1.9e-6, "a loaded knife hovering 2 mm off the face fills nothing: 0")
    fresh()
    _, s0, y0, _, _ = M.HOLE_SPECS[0]
    r = M.knife_rot(M.pitch_for(M.FILL_DEG))
    dab = load_knife() + go(path(M.face_point(s0, y0, M.BACK), M.face_point(s0, y0, -M.PRESS[1]), 0.005)
                            + [M.face_point(s0, y0, -M.PRESS[1])] * 30, r)
    check(feed(dab + set_down()) == 0.0, "a dab pressed on the hole and held still: 0")
    fresh()
    jab = load_knife() + go([M.face_point(s0, y0, M.BACK)], r)
    for _ in range(3):
        e = path(M.face_point(s0, y0, 0.03), M.face_point(s0, y0, -M.PRESS[1]), 0.008)
        jab += go(e + e[::-1], r)
    check(feed(jab + set_down()) == 0.0, "jabbing straight in and out three times: 0")
    fresh()
    check(feed(load_knife() + pack(0, held=False) + set_down()) == 0.0,
          "a knife pushed across the hole with no hand on it: 0")
    fresh()
    check(feed(pack(0) + strike(0) + set_down()) == 0.0, "an unloaded knife (never dipped): 0")
    fresh()
    feed(load_knife(held=False))
    check(M._L.paste.load == 0.0, "the knife doesn't load unless held")

    # -- the tiers -------------------------------------------------------------------------------------------------
    fresh()
    s = feed(load_knife() + pack(0) + set_down())
    check(s == 0.25 and M._L.filled() == [True, False, False], "one hole packed: 0.25")
    fresh()
    s = feed(load_knife() + pack(0) + pack(1) + set_down())
    check(s == 0.5, "two holes packed: 0.5")
    fresh()
    s = feed(load_knife() + pack(0) + pack(1) + load_knife() + pack(2) + set_down(40))  # reload: coats use 0.6 mL each
    check(s == 0.75 and M._L.proud_area() > M.PROUD_AREA,
          f"all three packed, coats left proud ({M._L.proud_area() * 1e6:.0f} mm^2), set down: 0.75")
    fresh()
    states = full_job()
    feed(states)
    check(M._L.paste.loaded_total <= M.LAW.load_cap + 1e-12, "the demo's single dip carries enough (one load)")
    check(all(M._L.filled()) and M._L.proud_area() <= M.PROUD_AREA,
          f"pack + strike off each hole: all filled, proud {M._L.proud_area() * 1e6:.0f} mm^2")
    check(max(M._L.depths()) <= M.FLUSH, "every hole's deepest point within 0.3 mm of the face")
    s = feed(set_down(M.HOLD_STEPS - 1))
    check(s == 0.75, "set down but not yet 1 s: 0.75")
    s = feed(set_down(2))
    check(s == 1.0, "filled flush, nothing proud, knife set down, released and still for 1 s: 1.0")
    fresh()
    check(feed(set_down(5)) == 0.0, "a new trial starts from open holes")

    # -- failures and adversarial cases ----------------------------------------------------------------------------
    fresh()
    s = feed(load_knife() + strike(0) + strike(1) + strike(2) + set_down())
    check(s == 0.0, "steep passes only: the blade bridges the holes and leaves them open: 0")
    fresh()
    s = feed(full_job() + set_down(40, held=True))
    check(s == 0.75, "the job done but the knife still held at the end: 0.75")
    fresh()
    s = feed(full_job() + set_down(40, wobble=True))
    check(s == 0.75, "set down but moving: 0.75")
    fresh()
    _, s2, y2, _, _ = M.HOLE_SPECS[2]
    lean = [objs(at_edge(M.face_point(s2 - 0.03, y2, 0.0), M.knife_rot(M.pitch_for(70))),
                 M.knife_rot(M.pitch_for(70)), False)] * 40
    s = feed(full_job() + lean)
    check(s == 0.75, "released with its edge resting on the panel (not set down): 0.75")
    fresh()
    states = load_knife()
    for i in range(3):
        states += pack(i) + strike(i)
    states += load_knife() + pack(1)  # an extra low pass over one hole, left on
    s = feed(states + set_down(40))
    check(s == 0.75, "a coat left standing over one hole after striking: 0.75")
    fresh()
    smear = load_knife()
    for i in range(3):
        for dv in (-0.03, 0.0, 0.03):
            smear += face_pass(i, 25.0, (0.03, -0.03), dv=dv)
    s = feed(smear + set_down(40))
    check(s == 0.0 and M._L.paste.load == 0.0,
          "smeared side to side at a low angle from one load: the knife runs dry on the wall, holes open: 0")
    fresh()
    smear = []
    for i in range(3):
        for dv in (-0.03, 0.0, 0.03):
            smear += load_knife() + face_pass(i, 25.0, (0.03, -0.03), dv=dv)
    s = feed(smear + set_down(40))
    check(s == 0.75 and M._L.proud_area() > 10 * M.PROUD_AREA,
          f"smeared over the whole area, reloading each pass: holes filled, {M._L.proud_area() * 1e6:.0f} mm^2 proud: 0.75")
    fresh()
    states = full_job() + load_knife() + face_pass(1, 45.0, (0.03, -0.03), tilt=25.0)
    s = feed(states + set_down(40))
    check(s == 0.5 and M._L.paste.gouged("face"),
          "all filled and struck off, but the corner dragged 6 cm (edge turned 25 deg): gouge proxy caps it at 0.5")
    fresh()
    s = feed(load_knife() + pack(0) + face_pass(0, 45.0, (0.03, -0.03), tilt=25.0) + set_down())
    check(s == 0.25 and M._L.paste.gouged("face"), "one hole filled, then a gouge: 0.25 (below the cap, unchanged)")
    fresh()
    s = feed(load_knife() + face_pass(0, M.FILL_DEG, M.FILL_SPAN, tilt=8.0) + strike(0) + set_down())
    check(s == 0.25 and not M._L.paste.gouged("face"), "an edge turned 8 deg is not a gouge")
    fresh()
    s = feed(load_knife() + face_pass(0, M.FILL_DEG, M.FILL_SPAN, dv=0.019) + set_down())
    check(s == 0.0, "a pass whose edge only half covers the hole leaves it open: 0")
    fresh()
    s = feed(full_job() + strike(0) + strike(1) + strike(2) + set_down(40))
    check(s == 1.0, "extra strike passes don't hurt: 1.0")

    # -- pack and strike angles: near-identical strokes can't pack and strike off -----------------------------------
    check(M.LAW.film(M.LAW.pack_max_deg) >= 2 * M.PROUD, "every packing angle leaves a coat >= 0.6 mm (2x the limit)")
    flat_at = [a for a in range(30, 91) if M.LAW.film(a) <= M.PROUD + 1e-12]
    check(flat_at[0] == 55 and flat_at[0] - M.LAW.pack_max_deg >= 14,
          f"the shallowest whole-degree strike-off within 0.3 mm is {flat_at[0]} deg, 14+ above packing (40)")

    def flush0(a, b):
        """Hole 0 after a pass at a degrees then one at b degrees: (filled, flush around it)."""
        fresh()
        feed(load_knife() + face_pass(0, a, M.STRIKE_SPAN) + face_pass(0, b, M.STRIKE_SPAN))
        return M._L.filled()[0], M._L.proud_area() <= M.PROUD_AREA

    for a in (30, 36, 40, 42, 46, 48, 50, 52, 54):
        for d in (0, 2, 6, 10):
            f, fl = flush0(a, a + d)
            check(not (f and fl), f"pass at {a} then {a + d} deg: hole 0 not both filled and flush ({f}, {fl})")
    f, fl = flush0(50, 52)
    check(not f, "50 then 52 deg: neither packs, hole open")
    f, fl = flush0(40, 56)
    check(f and fl, "40 deg (pack) then 56 deg (strike off): filled and flush")

    # -- set-down variants -------------------------------------------------------------------------------------------
    fresh()
    r_flat = M.knife_rot(M.pitch_for(0.0))  # blade face lying on the face, edge pointing up it
    p = at_edge(M.face_point(0.10, -0.15, 0.0012), r_flat)  # the blade's lower face 0.7 mm off: edge not touching
    s = feed(full_job() + [objs(p, r_flat, False)] * 40)
    check(s == 0.75 and not M._L.paste.touching("face") and not M.knife_clear_of_panel(p, pk.quat(r_flat)),
          "released lying against the panel on its blade face (edge not on it): within 2 mm of the panel, 0.75")
    fresh()
    across = np.array([M.TUB_C[0], M.TUB_C[1], M.TOP + M.TUB_H + M.HANDLE[1][2] / 2])
    s = feed(full_job() + [objs(across, rest_rot(), False)] * 40)
    check(s == 1.0, "laid across the tub's rim, released and still: set down, 1.0")

    # -- text matches code -----------------------------------------------------------------------------------------
    d = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(d["instruction"] == M.INSTRUCTION, "task.json instruction == task.INSTRUCTION")
    text = " ".join(str(d[k]) for k in ("setup", "scoring", "fidelity", "context")) + readme
    for frag in ("0.3 mm", "10 mm^2", "20 degrees", "3 mm", "6 mm", "8 mm", "10 mm", "0.6 mm", "40 degrees",
                 "60 degrees", "54 degrees", "14 degrees", "0.8 mm", "0.1 mm", "2 mL", "15 degrees", "1 s", "70 g",
                 "36 x 28 cm", "Capped at 0.5", "2 mm clear of the panel", "2 mm grid"):
        norm = lambda t: t.replace("^2", "\u00b2").lower()
        check(norm(frag) in norm(text), f"text states {frag!r}")
    limits = dict(title=120, instruction=1000, context=1000, setup=4000, scoring=4000, fidelity=500, scores=300)
    for k, n in limits.items():
        check(len(d.get(k, "")) <= n, f"{k} within {n} chars ({len(d.get(k, ''))})")
    check(not re.search(r"\b(colour|centre|grey|metre|litre)\b", json.dumps(d)), "American spelling")

    # -- the demo plan ---------------------------------------------------------------------------------------------
    plan = M.demo()["right"]
    steps = round(sum(w.duration for w in plan) / M.DT)
    check(steps <= 2100, f"demo plan {steps} steps (fits the README's 2,100-step demo run)")
    zs = [w.pos[2] - M.TOP for w in plan]
    check(max(zs) <= 0.22, f"highest tool point {max(zs):.3f} m above the table (arm lifts ~0.22 m)")
    xs = [w.pos[0] for w in plan]
    check(min(xs) >= 0.15 and max(xs) <= 0.40, f"tool points within x 0.15-0.40 m ({min(xs):.3f}-{max(xs):.3f})")
    check(all(abs(w.pos[1]) < 0.30 for w in plan), "tool points within |y| < 0.30 m")
    print("all pass")


def test_all():
    main()


if __name__ == "__main__":
    sys.exit(main())
