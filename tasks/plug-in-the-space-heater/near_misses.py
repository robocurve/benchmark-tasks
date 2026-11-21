"""Scoring counterexamples, run offline: every level, both sides of each limit, the naive errand and other shortcuts."""
import copy
import math

import numpy as np

import lots
import parts
import task

H, L, S = task.HEATER_PLUG, task.LAMP_PLUG, task.STRIP_PLUG
W1, W2 = task.WALL_OUTLETS
S1, S2, S3, S4, S5 = task.STRIP_OUTLETS
FLOOR_H = (0.229, 0.288, task.HEATER_Z)  # the heater's plug where lot A starts it
FLOOR_L = (0.250, 0.120, task.TOP + task.LAMP_BODY[1] / 2)  # the lamp's plug lying on its side on the floor


def quat(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    r = (parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((0, 1, 0), math.radians(pitch_deg))
         @ parts._rot((1, 0, 0), math.radians(roll_deg)))
    return parts._matrix_to_quat(r)


def in_outlet(o, gap=0.0002, dy=0.0, dz=0.0004):
    """The plug-face position for a plug in outlet ``o``, its face ``gap`` from the outlet's face."""
    return (o.xf - gap, o.yc + dy, o.zc + dz)


def states(heater=FLOOR_H, lamp=None, strip=None, hq=None, lq=None):
    lamp = in_outlet(W2) if lamp is None else lamp
    strip = in_outlet(W1) if strip is None else strip
    lq = quat(roll_deg=90.0) if lq is None and lamp == FLOOR_L else lq
    return {H.name: H.state(pos=heater, quat_xyzw=hq), L.name: L.state(pos=lamp, quat_xyzw=lq),
            S.name: S.state(pos=strip)}


def grade(objs, held=1.0, known=True, invalid=False, dropped=False, in_strip=False, stale=False):
    task.REC.update(known=known, invalid=invalid, dropped=dropped, in_strip=in_strip, held=held)
    task.REC["stamp"] = () if stale else task._stamp(objs)
    return task.score(objs)


# check.py runs these against the task's own history, which is unobserved before any simulator step: all score 0.
CASES = [
    ("heater in the wall, lamp in the strip, but no observed history", states(in_outlet(W2), in_outlet(S2)), 0.0,
     False),
    ("every plug at its start", {}, 0.0, False),
]


def run():
    old = copy.deepcopy(task.REC)
    lots.use("A")
    r = []
    try:
        full = states(in_outlet(W2), in_outlet(S2))
        r.append(("heater plug in the freed wall outlet, lamp plug in the strip, strip plug in the wall: 1.0",
                  grade(full) == 1.0))
        r.append(("same with the lamp plug in the strip's fifth outlet: 1.0",
                  grade(states(in_outlet(W2), in_outlet(S5))) == 1.0))
        r.append(("same, only 0.95 s let go and still: 0", grade(full, held=0.95) == 0.0))
        r.append(("start (nothing done): 0", grade(states()) == 0.0))
        # the naive errand: the heater's plug into the nearest free outlet, on the power strip
        naive = states(in_outlet(S4))
        r.append(("NAIVE: heater plug fully in the power strip: 0", grade(naive) == 0.0))
        r.append(("NAIVE then corrected: heater blade was in the strip earlier, everything right at the end: 0",
                  grade(full, in_strip=True) == 0.0))
        r.append(("heater blade tip 1.1 mm into a strip slot trips the whole-trial zero",
                  task.depth_in(states((S4.xf - task.BLADE_L + 0.0011, S4.yc, S4.zc))[H.name], S4) > task.STRIP_ZERO))
        r.append(("heater blade tip 0.9 mm into a strip slot does not",
                  not task.depth_in(states((S4.xf - task.BLADE_L + 0.0009, S4.yc, S4.zc))[H.name], S4)
                  > task.STRIP_ZERO))
        r.append(("heater blades against the strip's face between two outlets do not",
                  task.depth_in(states((S4.xf - task.BLADE_L + 0.0005, S4.yc + 0.018, S4.zc))[H.name], S4) == 0.0))
        # partial levels
        r.append(("heater plug in the wall, lamp plug left on the floor: 0.6",
                  grade(states(in_outlet(W2), FLOOR_L)) == 0.6))
        r.append(("lamp plug pulled out and left on the floor, heater plug not plugged in: 0.25",
                  grade(states(lamp=FLOOR_L)) == 0.25))
        r.append(("lamp plug moved to the strip, heater plug still on the floor: 0.25",
                  grade(states(lamp=in_outlet(S2))) == 0.25))
        r.append(("strip plug pulled out instead, heater plug in its outlet, lamp untouched: 0.6",
                  grade(states(in_outlet(W1), strip=(0.25, 0.03, task.TOP + 0.02))) == 0.6))
        r.append(("strip plug pulled out, nothing else: 0.25",
                  grade(states(strip=(0.25, 0.03, task.TOP + 0.02))) == 0.25))
        # fully in or not: the plug face within 2 mm of the outlet face
        r.append(("heater plug 1.9 mm proud of the wall outlet: still fully in, 1.0",
                  grade(states(in_outlet(W2, gap=0.0019), in_outlet(S2))) == 1.0))
        r.append(("heater plug 2.1 mm proud: partly in, 0",
                  grade(states(in_outlet(W2, gap=0.0021), in_outlet(S2))) == 0.0))
        r.append(("lamp plug 2.5 mm proud of the strip: partly in, 0",
                  grade(states(in_outlet(W2), in_outlet(S2, gap=0.0025))) == 0.0))
        r.append(("heater plug half in (8 mm of blade showing), lamp on the floor: 0",
                  grade(states(in_outlet(W2, gap=0.008), FLOOR_L)) == 0.0))
        r.append(("heater plug in the wall but drooping 6 degrees: not fully in, 0",
                  grade(states(in_outlet(W2), in_outlet(S2), hq=quat(pitch_deg=6.0))) == 0.0))
        r.append(("heater plug in the wall drooping 4 degrees: 1.0",
                  grade(states(in_outlet(W2), in_outlet(S2), hq=quat(pitch_deg=4.0))) == 1.0))
        r.append(("heater plug pushed flat against the wall plate beside the outlet, lamp in the strip: 0.25",
                  grade(states((W2.xf - task.BLADE_L, W2.yc + 0.045, W2.zc), in_outlet(S2))) == 0.25))
        r.append(("heater plug upside down, blade tips only in the mouths: 0",
                  grade(states(in_outlet(W2, gap=0.014), in_outlet(S2), hq=quat(roll_deg=180.0))) == 0.0))
        r.append(("lamp plug put back into the wall, heater on the floor: 0", grade(states()) == 0.0))
        # fail closed
        r.append(("a plug fell off the table at any time: 0", grade(full, dropped=True) == 0.0))
        r.append(("hand state unknown: 0", grade(full, known=False) == 0.0))
        r.append(("a bad step earlier: 0", grade(full, invalid=True) == 0.0))
        r.append(("stale state: 0", grade(full, stale=True) == 0.0))
        bad = dict(full)
        bad[H.name] = parts.PartState(**{**full[H.name].__dict__, "pos": np.array([math.nan, 0.0, 0.0])})
        r.append(("NaN plug position: 0", grade(bad) == 0.0))
        # stillness and the let-go rule (sampled points against the plugs' boxes, not just vertices)
        r.append(("still at 5 mm/s and 0.05 rad/s", task.still([(0.005, 0.05)] * 3)))
        r.append(("not still at 5.1 mm/s", not task.still([(0.0051, 0.0), (0.0, 0.0), (0.0, 0.0)])))
        r.append(("no previous pose: not still", task.motion(None, full[H.name], 0.05) == (math.inf, math.inf)))
        bx = [b for p in task.PLUGS for b in task.boxes(p, full[p.name])]
        c, rr, h = task.boxes(H, full[H.name])[0]  # the heater plug's body
        side = c + rr @ np.array([0.0, h[1], 0.0])
        r.append(("finger face 0.5 mm off the heater plug's side, no vertex near: touching",
                  task.hand_gap([np.array([side + (0, 0.0005, 0)])], bx) <= task.CLEAR))
        r.append(("finger 4.9 mm off the side: touching", task.hand_gap([np.array([side + (0, 0.0049, 0)])], bx)
                  <= task.CLEAR))
        r.append(("finger 5.1 mm off the side: clear", task.hand_gap([np.array([side + (0, 0.0051, 0)])], bx)
                  > task.CLEAR))
        r.append(("NaN hand points fail closed", task.hand_gap([np.full((3, 3), np.nan)], bx) == -math.inf))
        r.append(("no hand points fail closed", task.hand_gap([], bx) == -math.inf))
        # lot B: the lamp's and the strip's plugs swap wall outlets
        lots.use("B")
        fb = states(in_outlet(W1), in_outlet(S2), in_outlet(W2))
        r.append(("lot B: heater plug in the outlet the lamp left, lamp in the strip: 1.0", grade(fb) == 1.0))
        r.append(("lot B: start: 0", grade(states(task.HEATER_B, in_outlet(W1), in_outlet(W2))) == 0.0))
        r.append(("lot B: NAIVE, heater plug in the strip: 0",
                  grade(states(in_outlet(S3), in_outlet(W1), in_outlet(W2))) == 0.0))
        # A layout reset must keep the appliance endpoint fixed while the plug moves.
        for lot in task.LOTS:
            for plug in task.PLUGS:
                q = task.CORD_LAYOUTS[lot.name][plug.name]
                posed = plug.link_poses(lot.pos(plug.name), np.eye(3), q)[plug.joints[-1].name]
                anchor = plug.link_poses(plug.pos, np.eye(3), dict.fromkeys(q, 0.0))[plug.joints[-1].name]
                r.append((f"lot {lot.name}: {plug.name} reset retains cord endpoint",
                          np.linalg.norm(posed[0] - anchor[0]) < 1e-5 and np.linalg.norm(posed[1] - anchor[1]) < 1e-4))
        # The reset check: run in the embodiment's reset before the policy acts, never by the scorer.
        for lot in task.LOTS:
            start = {p.name: p.state(pos=lot.pos(p.name), joints=task.CORD_LAYOUTS[lot.name][p.name])
                     for p in task.PLUGS}
            r.append((f"reset check: lot {lot.name}'s published start accepted", task.valid_start(start, lot)))
        lot_b = task.LOTS[1]
        r.append(("reset check: lot B with lot A's plug positions refused", not task.valid_start(states(), lot_b)))
        failed = states((0.22198, 0.26177, task.HEATER_Z), (0.35979, 0.07542, 0.8269), (0.36002, 0.09943, 0.82689))
        r.append(("reset check: the failed native lot B start (lamp 9.5 mm off, partly in) refused",
                  not task.valid_start(failed, lot_b)))
        r.append(("reset check: lot B with the lamp's plug drawn 3 mm out refused",
                  not task.valid_start(states(task.HEATER_B, in_outlet(W1, gap=0.0032), in_outlet(W2)), lot_b)))
    finally:
        lots.use(None)
        task.REC.clear()
        task.REC.update(old)
    for name, ok in r:
        print("ok  " if ok else "FAIL", name)
    return sum(not ok for _, ok in r)


if __name__ == "__main__":
    raise SystemExit(run())
