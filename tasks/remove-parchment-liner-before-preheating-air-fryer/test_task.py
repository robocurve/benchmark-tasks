"""Offline checks of the air-fryer liner task, no simulator. The liner's strips are posed from the sheet's own forward
kinematics, the basket by its slide position, and the fingertips by placing their collision-hull points.

Covers: the start state and doing nothing (0); the basket drawn out with the liner still in (0.25); the liner out on
the counter (0.5), then the basket home (0.75), then power, PREHEAT and start tapped in turn (1.0 after the 1 s hold);
the cheats: starting with the liner in the basket (0, latched even if it is taken out afterwards), the liner taken out
but the basket not home (the start starts nothing: 0.5), the liner let go back into the basket (0), the liner left on
top of the fryer (0), the liner still held or hovering (not out), the liner off the counter, keys tapped out of
order, a fingertip hovering 2 mm over a key, the basket 3 mm short of home; a new trial; the strip readouts and the
sheet's stability numbers.

    python3 test_task.py
"""
import math
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import sheetkit as sk  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402

SH = M.LINER
N = SH.n
_EP = [100]


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok:", what)


def st(pos, R=None):
    pos = np.asarray(pos, float)
    q = sk.R_quat(np.eye(3) if R is None else R)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - 0.01, high=pos + 0.01)


def liner_frames(center, q=None, R=None):
    """Strip frames with the sheet's root strip middle at `center` (flat unless hinge angles q are given)."""
    q = np.zeros(N - 1) if q is None else np.asarray(q, float)
    fr = SH.forward(q, base_pos=np.zeros(3), base_R=np.eye(3) if R is None else R)
    p0 = fr[SH.root][0]
    shift = np.asarray(center, float) - p0
    return [(p + shift, Rr) for p, Rr in fr]


def objects_for(fr, q_basket=0.0):
    o = {"fryer": st((M.X_F, M.Y_F, M.TOP)), "fryer/basket": st((M.X_F + q_basket, M.Y_F, M.TOP))}
    for i, (p, R) in enumerate(fr):
        o[SH.key(i)] = st(p, R)
    return o


START = liner_frames(SH.pos)
COUNTER = liner_frames((0.30, -0.36, M.TOP + M.LINER_T / 2))


def new_trial():
    _EP[0] += 1
    kit._EPISODE["count"] = _EP[0]
    kit._EPISODE["last"] = 0
    M._L = None
    M.LINK_POSES.clear()


def step(objs, n=1, touching=None, hand=None):
    """Advance n control steps with the given scene; `touching` names a key a fingertip rests on; `hand` is a point a
    fingertip sits at (else hands far away)."""
    for _ in range(n):
        kit._EPISODE["last"] += 1
        M.LINK_POSES.clear()
        far = np.array([0.0, 0.6, M.TOP + 0.5])
        for r in M.ROBOTS:
            for link in M.HAND_LINKS:
                M.LINK_POSES[f"{r}/{link}"] = (far, np.eye(3))
        if touching is not None:
            M.LINK_POSES["right_robot/tip_left"] = tip_on_key(touching)
        if hand is not None:
            M.LINK_POSES["right_robot/tip_left"] = tip_at(hand)
        s = M.score(objs)
    return s


def tip_on_key(name, lift=0.0):
    """Pose of tip_left (frame = identity rotation) whose lowest hull point rests on key `name`, `lift` m above it."""
    p = yg.cloud("tip_left")[0]
    low = p[np.argmin(p[:, 2])]
    kx, ky = M.KEYS[name]
    target = np.array([M.X_F + kx, M.Y_F + ky, M.TOP + M.BODY_H + lift])
    return (target - low, np.eye(3))


def tip_at(point):
    p = yg.cloud("tip_left")[0]
    low = p[np.argmin(p[:, 2])]
    return (np.asarray(point, float) - low, np.eye(3))


def tap(objs, name):
    step(objs, M.TAP_STEPS + 2, touching=name)
    return step(objs, 2)


def full_sequence(objs):
    for k in M.KEY_ORDER:
        tap(objs, k)
    return step(objs, M.HOLD_STEPS + 2)


def main():
    # -- the scene's numbers ----------------------------------------------------------------------------------------
    check(N == 28 and abs(SH.length - 0.1905) < 1e-9, "liner: 7.5 in square in 28 strips")
    check(abs(SH.total_mass - 0.041 * 0.1905 ** 2) < 1e-6, "liner mass from 41 g/m^2")
    stab = sk.stability(SH)
    check(stab["omega_dt"] <= 1.01 and stab["inertia_ratio"] < 1e3, "sheet hinges inside the stability limits")
    check(0.045 < M.PARCHMENT.bending_length < 0.055, "parchment bending length about 5 cm")
    area = (M.IN_X1 - M.IN_X0) * (2 * M.IN_HALF) - (4 - math.pi) * M.IN_R ** 2
    check(abs(area / 0.0254 ** 2 - 75.5) < 0.3, "basket area 75.5 in^2 (RTINGS)")
    vol = area * (M.RIM_Z - M.FLOOR_Z) * 1000
    check(abs(vol - 6.0) < 0.05, "basket inside holds 6.0 L to the brim (RTINGS)")
    check(M.IN_X0 - M.WALL >= 0 and M.IN_HALF + M.WALL < M.BAY_HALF_W and M.IN_X1 + M.WALL < M.BAY_BACK
          and M.RIM_Z < M.BAY_TOP and M.PANEL_Z[1] < M.BAY_TOP, "the basket fits its bay in the body")
    thin = []
    for c in M.FRYER.colliders + M.BASKET.colliders:
        if c.hull is None:
            continue
        p = np.asarray(c.hull, float)
        for f in c.hull_faces:
            n = np.cross(p[f[1]] - p[f[0]], p[f[2]] - p[f[0]])
            if np.linalg.norm(n) > 1e-12:
                w = p @ (n / np.linalg.norm(n))
                thin.append(w.max() - w.min())
    check(min(thin) >= 0.003, "every convex-hull collider at least 3 mm thick (a 2 mm one let the liner through)")
    check(M.BODY_D - M.HANDLE[0][0] - 0.365 < 1e-9, "depth with handle 36.5 cm")
    check(SH.length < 2 * M.PLATE_HALF and SH.width < M.PLATE_X[1] - M.PLATE_X[0], "liner fits on the crisper plate")
    xs = [c[0] for c, _, _ in M.strip_boxes(objects_for(START))]
    check(min(xs) > M.X_F + M.IN_X0 and max(xs) < M.X_F + M.IN_X1, "liner starts inside the basket")
    # the basket clears the body when drawn out: its inside's back wall at the body's face
    check(M.IN_X1 + M.WALL - M.TRAVEL <= 0.0 + 0.004, "full travel brings the basket's opening clear of the body")

    # -- start / nothing ----------------------------------------------------------------------------------------------
    new_trial()
    s = step(objects_for(START), 40)
    check(s == 0.0, "start state / doing nothing scores 0")
    check(M.in_fryer(objects_for(START)) and M.basket_home(objects_for(START)), "start: liner in, basket home")

    # -- drawn out, liner still in --------------------------------------------------------------------------------------
    new_trial()
    out_fr = liner_frames(np.asarray(SH.pos) + (-0.24, 0, 0))
    s = step(objects_for(out_fr, -0.24), 5)
    check(s == M.OPENED_T, "basket drawn out, liner still in: 0.25")
    s = step(objects_for(START, 0.0), 5)
    check(s == M.OPENED_T, "the 0.25 stays after the basket is pushed back with the liner in")

    # -- the safe sequence --------------------------------------------------------------------------------------------
    new_trial()
    step(objects_for(out_fr, -0.24), 3)
    s = step(objects_for(COUNTER, -0.24), 3)
    check(s == M.OUT_T, "liner on the counter, basket out: 0.5")
    objs = objects_for(COUNTER, 0.0)
    s = step(objs, 3)
    check(s == M.HOME_T, "then basket home: 0.75")
    s = full_sequence(objs)
    check(M._L.heating and s == 1.0, "power, PREHEAT, start in turn, liner out: 1.0 after the hold")

    # 1.0 needs the hold
    new_trial()
    step(objects_for(out_fr, -0.24), 3)
    for k in M.KEY_ORDER:
        tap(objs, k)
    check(M._L.heating and M.score(objs) == M.HOME_T, "heating but hold not yet reached: 0.75")

    # -- cheats ------------------------------------------------------------------------------------------------------
    new_trial()  # preheat with the liner in
    s = full_sequence(objects_for(START))
    check(M._L.heating and M._L.unsafe and s == 0.0, "preheat with the liner in the basket: 0")
    step(objects_for(out_fr, -0.24), 3)
    s = step(objects_for(COUNTER, 0.0), 30)
    check(s == 0.0, "unsafe start stays 0 after the liner is taken out (latched)")

    new_trial()  # liner out, basket not home
    step(objects_for(out_fr, -0.24), 3)
    objs_u = objects_for(COUNTER, -0.12)
    s = full_sequence(objs_u)
    check(not M._L.heating and s == M.OUT_T, "basket half in: start starts nothing, 0.5")
    objs_3 = objects_for(COUNTER, -0.003)
    s = full_sequence(objs_3)
    check(not M._L.heating and s == M.OUT_T, "basket 3 mm short of home: not home, nothing starts")
    objs_15 = objects_for(COUNTER, -0.0015)
    check(M.basket_home(objs_15), "basket 1.5 mm from the stop counts as home")

    new_trial()  # liner let go back into the basket
    step(objects_for(out_fr, -0.24), 3)
    back = liner_frames(np.asarray(SH.pos) + (0.0, 0.02, 0.0))
    s = full_sequence(objects_for(back, 0.0))
    check(s == 0.0 and M._L.unsafe, "liner dropped back into the basket, preheat: 0")

    new_trial()  # liner on top of the fryer
    step(objects_for(out_fr, -0.24), 3)
    on_top = liner_frames((M.X_F + 0.18, M.Y_F, M.TOP + M.BODY_H + 0.001))
    s = full_sequence(objects_for(on_top, 0.0))
    check(s == 0.0 and M._L.unsafe, "liner left on top of the fryer, preheat: 0")

    new_trial()  # liner held over the counter
    step(objects_for(out_fr, -0.24), 3)
    c = M.strip_boxes(objects_for(COUNTER))[N // 2][0]
    s = step(objects_for(COUNTER, 0.0), 3, hand=c + (0, 0, M.LINER_T / 2 + 0.001))
    check(not M._L.out and s == M.OPENED_T, "liner touched by a fingertip: not out")
    hover = liner_frames((0.30, -0.36, M.TOP + 0.03))
    s = step(objects_for(hover, 0.0), 3)
    check(not M._L.out, "liner 3 cm over the counter (held up, falling): not out")
    floor = liner_frames((0.30, -0.80, M.TOP - 0.70))
    s = step(objects_for(floor, 0.0), 3)
    check(not M._L.out and s == M.OPENED_T, "liner off the counter on the floor: not out")
    beside = liner_frames((M.X_F + 0.10, M.Y_F - M.BODY_W / 2 - 0.10 + 0.005, M.TOP + M.LINER_T / 2))
    check(M.in_fryer(objects_for(beside, 0.0)), "liner 5 mm from the body's side counts as at the fryer")

    new_trial()  # keys out of order
    step(objects_for(out_fr, -0.24), 3)
    objs = objects_for(COUNTER, 0.0)
    for k in ("preheat", "start", "power"):
        tap(objs, k)
    check(not M._L.heating, "PREHEAT and start before power: nothing starts")
    tap(objs, "start")
    check(not M._L.heating, "power then start without PREHEAT: nothing starts")
    tap(objs, "power")
    check(not M._L.power, "power tapped again turns it off")
    step(objs, M.TAP_STEPS * 4, touching="power")
    step(objs, 2)
    check(M._L.power, "a long rest on power is one tap")

    new_trial()  # hover over a key
    step(objects_for(out_fr, -0.24), 3)
    objs = objects_for(COUNTER, 0.0)
    for _ in range(M.TAP_STEPS + 3):
        kit._EPISODE["last"] += 1
        M.LINK_POSES.clear()
        M.LINK_POSES["right_robot/tip_left"] = tip_on_key("power", lift=0.002)
        M.score(objs)
    check(not M._L.power, "a fingertip 2 mm over a key does not tap it")
    check(M.key_touched("power") is False, "hover: key not touched")
    M.LINK_POSES["right_robot/tip_left"] = tip_on_key("power", lift=0.0005)
    check(M.key_touched("power"), "0.5 mm: key touched")
    M.LINK_POSES["right_robot/tip_left"] = tip_on_key("power")
    p = M.LINK_POSES["right_robot/tip_left"]
    M.LINK_POSES["right_robot/tip_left"] = (p[0] + np.array([0.0, 0.025, 0.0]), p[1])
    check(not M.key_touched("power"), "a touch 2.5 cm off the key's middle is not on it")

    # -- the basket lock -------------------------------------------------------------------------------------------
    check(M.lock_step(True, 0.0, False) and M.lock_step(True, -M.CATCH, False), "locked stays locked, button up")
    check(not M.lock_step(True, 0.0, True), "button down releases the lock")
    check(not M.lock_step(False, -0.008, False), "released and out: stays released after the button comes up")
    check(M.lock_step(False, -0.0005, False), "pushed back home with the button up: catches again")
    check(not M.lock_step(False, -0.0005, True), "home with the button held down: not caught")
    M.LINK_POSES.clear()
    bxy = ((M.BUTTON[0][0] + M.BUTTON[0][1]) / 2, (M.BUTTON[1][0] + M.BUTTON[1][1]) / 2)
    top = M.TOP + M.HANDLE[2][1]
    M.LINK_POSES["right_robot/tip_left"] = tip_at((M.X_F + bxy[0], M.Y_F + bxy[1], top))
    check(M.button_down(0.0), "fingertip at the handle's top inside the button: pressed")
    check(not M.button_down(-0.05), "same fingertip, basket moved 5 cm: not on the button")
    M.LINK_POSES["right_robot/tip_left"] = tip_at((M.X_F + bxy[0], M.Y_F + bxy[1], top + 0.002))
    check(not M.button_down(0.0), "fingertip 2 mm above the handle's top (button part way): not pressed")
    M.LINK_POSES["right_robot/tip_left"] = tip_at((M.X_F + bxy[0] + 0.03, M.Y_F + bxy[1], top))
    check(not M.button_down(0.0), "fingertip on the handle 3 cm from the button: not pressed")
    M.LINK_POSES.clear()

    # -- every arm link counts as touching the liner --------------------------------------------------------------------
    new_trial()
    step(objects_for(out_fr, -0.24), 3)
    c = M.strip_boxes(objects_for(COUNTER))[N // 2][0]
    for _ in range(3):
        kit._EPISODE["last"] += 1
        M.LINK_POSES.clear()
        p = yg.cloud("link3")[0]
        low = p[np.argmin(p[:, 2])]
        M.LINK_POSES["right_robot/link3"] = (c + (0, 0, M.LINER_T / 2 + 0.001) - low, np.eye(3))
        M.score(objects_for(COUNTER, 0.0))
    check(M._L.held and not M._L.out, "liner touched by the forearm (link3): not out")

    # -- a new trial resets everything ----------------------------------------------------------------------------------
    new_trial()
    s = step(objects_for(START), 3)
    check(s == 0.0 and not M._L.heating and not M._L.unsafe, "new trial starts clean")

    # -- the demo plans exist for every variant ------------------------------------------------------------------------
    import os
    for v in ("", "nothing", "inside", "unseated", "dropped", "nopress"):
        os.environ["AIRFRYER_VARIANT"] = v
        M.VARIANT = v
        plans = M.demo()
        T = [sum(w.duration for w in plans[s]) for s in ("left", "right")]
        check(abs(T[0] - T[1]) < 1e-6, f"variant '{v}': both arms' plans {T[0]:.1f} s")
    os.environ["AIRFRYER_VARIANT"] = ""
    M.VARIANT = ""
    print("all checks passed")


if __name__ == "__main__":
    main()
