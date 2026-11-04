"""On the station table: a printed paper template (white sheet, 22 x 34 cm) lies flat on the table in front of the
robot. Printed on it is a 20 mm wide blue band whose centreline is a wavy S: it starts at the sheet's left edge
(robot's left) heading towards the robot's right, makes a quarter turn to the left (6 cm radius) so it heads away
from the robot, then a half turn to the right and a half turn to the left (both 6 cm radius), and ends with a 3 cm
straight run away from the robot at an orange 30 mm end circle. The band is 50 cm long along its centreline.
A small grey device box sits just off the sheet's left edge; a black 6 mm cable, 50 cm long (exactly the band's
length), leaves the box's right face at the start of the band and lies straight across the table towards the
robot's right. Nothing holds the cable in shape (no walls, clips or tape): it must be laid along the band
precisely, from the box end, and stay there by friction alone, with its free end on the orange circle.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Lay the black cable along the blue S-shaped band on the paper template, starting at the box, so that it lies inside the band along its whole length with its free end on the orange circle. Then let go of the cable."
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Lay the black cable along the blue S-shaped band on the paper template, starting at the box, so "
               "that it lies inside the band along its whole length with its free end on the orange circle. "
               "Then let go of the cable.")

# -- template path ------------------------------------------------------------------------------------------
RC = 0.06  # radius of every turn of the band's centreline
P0 = np.array([0.20, 0.175])  # band start = cable exit from the box; heading -y
C1 = P0 + np.array([RC, 0.0])  # quarter turn left (CCW): heading -y -> +x
C2 = C1 + np.array([0.0, -2 * RC])  # half turn right (CW): +x -> -x
C3 = C2 + np.array([0.0, -2 * RC])  # half turn left (CCW): -x -> +x
L_A, L_B, L_C, L_D = math.pi / 2 * RC, math.pi * RC, math.pi * RC, 0.03
PATH_LEN = L_A + L_B + L_C + L_D  # 0.5012 m
END_PT = C3 + np.array([L_D, -RC])  # centre of the end circle
HALF_W = 0.010  # half the band width: the +-1 cm tolerance
END_R = 0.015  # end circle radius


def path_point(u: float) -> tuple[np.ndarray, float]:
    """Band centreline (x, y) and heading (rad) at distance ``u`` from the box."""
    u = min(max(u, 0.0), PATH_LEN)
    if u <= L_A:
        ph = math.pi + u / RC
        return C1 + RC * np.array([math.cos(ph), math.sin(ph)]), ph + math.pi / 2
    u -= L_A
    if u <= L_B:
        ph = math.pi / 2 - u / RC
        return C2 + RC * np.array([math.cos(ph), math.sin(ph)]), ph - math.pi / 2
    u -= L_B
    if u <= L_C:
        ph = math.pi / 2 + u / RC
        return C3 + RC * np.array([math.cos(ph), math.sin(ph)]), ph + math.pi / 2
    u -= L_C
    return C3 + np.array([u, -RC]), 0.0


# -- objects --------------------------------------------------------------------------------------------------
R = 0.003  # 6 mm cable
CABLE_LEN = 0.50
BOX = Box("device_box", pos=(0.20, 0.205, TOP + 0.02), size=(0.05, 0.05, 0.04), color=(0.45, 0.47, 0.50))
CORD_START = (0.20, 0.175, TOP + R + 0.0005)
CORD = cable("cord", start=CORD_START, yaw=-math.pi / 2, length=CABLE_LEN, radius=R, anchor_start=True,
             color=(0.05, 0.05, 0.05), bend_stiffness=0.0005, bend_damping=0.0005, friction=1.0)

SHEET_X, SHEET_Y = (0.15, 0.37), (-0.165, 0.175)
SHEET_T = 0.002
SHEET = Box("template_sheet", pos=((SHEET_X[0] + SHEET_X[1]) / 2, (SHEET_Y[0] + SHEET_Y[1]) / 2,
                                   TOP + 0.0002 - SHEET_T / 2),
            size=(SHEET_X[1] - SHEET_X[0], SHEET_Y[1] - SHEET_Y[0], SHEET_T), color=(0.95, 0.95, 0.93))
BAND_TOP = TOP + 0.0004  # printed ink: drawn 0.2 mm above the sheet (rendering only; flat for the cable)


def _band() -> list:
    out = []
    n = 50
    for i in range(n):
        (a, _), (b, _) = path_point(i * PATH_LEN / n), path_point((i + 1) * PATH_LEN / n)
        mid, d = (a + b) / 2, b - a
        out.append(Box(f"band{i:02d}", pos=(float(mid[0]), float(mid[1]), BAND_TOP - SHEET_T / 2),
                       size=(float(np.linalg.norm(d)) + 0.0025, 2 * HALF_W, SHEET_T),
                       yaw=math.atan2(d[1], d[0]), color=(0.10, 0.40, 0.90)))
    return out


BAND = _band()
END_CIRCLE = Cylinder("end_circle", pos=(float(END_PT[0]), float(END_PT[1]), TOP + 0.0006 - SHEET_T / 2),
                      radius=END_R, height=SHEET_T, color=(0.95, 0.45, 0.05))
OBJECTS = [BOX, SHEET, *BAND, END_CIRCLE, CORD]

# -- scoring --------------------------------------------------------------------------------------------------
N_SECTIONS = 8
SEC_LEN = PATH_LEN / N_SECTIONS  # 6.27 cm
SAMPLES = np.linspace(0.0, PATH_LEN, 8 * N_SECTIONS + 1)[1:] - SEC_LEN / 16  # 8 per section, centred
REST_Z = BAND_TOP + R + 0.004  # cable centre at most this high: lying on the template, not held up
RELEASE_DIST = 0.03  # both gripper TCPs at least 3 cm from every point of the cable
_CL = np.array([path_point(u)[0] for u in np.linspace(0.0, PATH_LEN, 501)])
_CL_U = np.linspace(0.0, PATH_LEN, 501)


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _cable_arclength(objects) -> tuple[np.ndarray, np.ndarray]:
    pts = _dense(objects["cord"].polyline(), 0.002)
    return pts, np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def covered_points(objects) -> np.ndarray:
    """Per check point (SAMPLES): True if some part of the cable lying on the template is within 1 cm of it."""
    pts = _dense(objects["cord"].polyline())
    low = pts[pts[:, 2] <= REST_Z]
    ok = np.zeros(len(SAMPLES), dtype=bool)
    if len(low) == 0:
        return ok
    for i, u in enumerate(SAMPLES):
        p, _ = path_point(u)
        ok[i] = bool((np.linalg.norm(low[:, :2] - p, axis=1) <= HALF_W).any())
    return ok


def in_band(objects) -> np.ndarray:
    """Per section: True if every part of the cable from (section start - 1 cm) to the section's end, measured
    along the cable from the box, lies inside the band (within 1 cm of the centreline)."""
    pts, s = _cable_arclength(objects)
    d = np.linalg.norm(pts[:, None, :2] - _CL[None], axis=2).min(axis=1)
    out = np.zeros(N_SECTIONS, dtype=bool)
    for k in range(N_SECTIONS):
        m = (s >= k * SEC_LEN - 0.01) & (s <= (k + 1) * SEC_LEN)
        out[k] = bool((d[m] <= HALF_W).all())
    return out


def end_on_circle(objects) -> bool:
    e = objects["cord"].end_tip
    return bool(np.linalg.norm(e[:2] - END_PT) <= END_R and e[2] <= REST_Z)


def section_ok(objects) -> np.ndarray:
    """Per section (8 x 6.27 cm along the band from the box): all 8 check points covered by cable lying on the
    template, and that stretch of cable inside the band. The last section also needs the end on the circle."""
    ok = covered_points(objects).reshape(N_SECTIONS, -1).all(axis=1) & in_band(objects)
    ok[-1] &= end_on_circle(objects)
    return ok


def laid_sections(objects) -> int:
    """Number of consecutive good sections counted from the box end."""
    ok = section_ok(objects)
    return int(np.argmin(np.r_[ok, False]))


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_shape_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._shape_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    pts = _dense(objects["cord"].polyline())
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless the cable is released and its free end lies on the end circle. Else 1/8 per section laid,
    counted consecutively from the box."""
    if not (released(objects) and end_on_circle(objects)):
        return 0.0
    return laid_sections(objects) / N_SECTIONS


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting()


def distance(objects) -> float:
    """Band length (m) not yet laid (from the first bad section on)."""
    return float((N_SECTIONS - laid_sections(objects)) * SEC_LEN)


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def _cord_at(objects, a: float) -> tuple[np.ndarray, float]:
    """Live cable point at arclength ``a`` from the box, and the cable's heading there."""
    poly = objects["cord"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _near_yaw(yaw: float, ref: float) -> float:
    """``yaw`` + k*pi closest to ``ref`` (a straight-down grip is symmetric under a half turn)."""
    return yaw + math.pi * round((ref - yaw) / math.pi)


YAW_MAX = 1.35  # |grip yaw| kept below this: IK is exact there (fingers then close <= 13 deg off square)


def _grip_yaw(heading: float) -> float:
    y = (heading + math.pi / 2) % math.pi - math.pi / 2
    return float(np.clip(y, -YAW_MAX, YAW_MAX))


def _laid_arclength(objects, u: float) -> float | None:
    """Arclength of the resting cable point nearest the band centreline at ``u`` (None if none within 1 cm)."""
    pts, s = _cable_arclength(objects)
    p, _ = path_point(u)
    d = np.linalg.norm(pts[:, :2] - p, axis=1)
    ok = (d <= HALF_W) & (pts[:, 2] <= REST_Z)
    if not ok.any():
        return None
    return float(s[np.where(ok)[0][d[ok].argmin()]])


YAW_RANGE = {"left": (-2.1, 1.25), "right": (-1.25, 2.1)}  # grip yaws each arm reaches exactly over the band


def _grip_pair(h: float, th: float, arm: str = "left") -> tuple[float, float]:
    """Grasp yaw for a cable running along ``h`` and place yaw that turns it onto heading ``th``: of the
    straight-down grasps (half a turn apart), the one that keeps both yaws inside the arm's reachable range."""
    lo, hi = YAW_RANGE[arm]
    rot = (th - h + math.pi) % (2 * math.pi) - math.pi  # turn that carries the cable's direction onto the band's
    y0 = (h + math.pi / 2) % math.pi - math.pi / 2
    best = None
    for gy in (y0 - math.pi, y0, y0 + math.pi):
        ty = gy + rot
        cost = sum(max(lo - v, 0) + max(v - hi, 0) for v in (gy, ty))
        if best is None or cost < best[0] - 1e-9:
            best = (cost, gy, ty)
    gy = float(np.clip(best[1], lo, hi))
    return gy, float(np.clip(gy + rot, lo, hi))


PAD_TIP = 0.013  # finger pad tips below the TCP
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart round a 6 mm cable
DEMO_STEP = 0.03  # band distance per placement
SPLIT_U = 0.215  # left arm lays the band up to here (y > ~0.03), right arm the rest


def demo_script(ctx):
    """Closed loop. The right arm first slides the free end off to the right of the template so the loose cable
    lies flat and slack. The left arm lays the band up to 21.5 cm from the box: for each band point u (every 3 cm)
    it grasps the cable at arclength u (the cable is exactly as long as the band), slides it just above the table
    to the band point and lets go, with the fingers closing square across the cable; a touch-up re-places the worst
    point of any bad section. The left arm then pins the last laid point while the right arm moves the free end to
    beyond the last turn, moves clear, and the right arm lays the rest (end on the circle) plus a touch-up.
    SHAPE_CHEAT=nothing|drag|half|hold runs a shortcut instead (each must score 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    over = 0.010  # TCP above the cable centre when grasping it
    hi = TOP + 0.06
    carry = TOP + 0.03
    lay_z = BAND_TOP + R + over + 0.001
    cheat = os.environ.get("SHAPE_CHEAT", "")
    dbg = bool(os.environ.get("DEMO_DEBUG"))

    def log(tag):
        if dbg:
            st = ctx.states()
            print(f"[demo t={ctx.t:5.1f}] {tag}: ok={section_ok(st).astype(int).tolist()} "
                  f"laid={laid_sections(st)} rel={released(st)} score={score(st):.3f}", flush=True)

    if cheat == "nothing":
        yield from L.wait(3.0)
        log("end")
        return

    def take(arm, a, z_after=hi):
        st = ctx.states()
        p, h = _cord_at(st, a)
        gy = _grip_yaw(h)
        yield from arm.move((p[0], p[1], hi), yaw=gy, finger=GRASP_OPEN, duration=0.9)
        yield from arm.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.6)
        yield from arm.grip(FINGER_CLOSED, 0.4)
        yield from arm.move((p[0], p[1], z_after), yaw=gy, duration=0.5)
        return p, h, gy

    if cheat == "drag":
        # drag the free end straight across the template (low, along the table) to the end circle and let go:
        # the cable ends up as a straight chord from the box to the circle
        p, h, gy = yield from take(Rt, CABLE_LEN - 0.02, TOP + 0.012)
        for q in ((0.30, -0.25), (0.33, -0.15), (END_PT[0] + 0.01, END_PT[1] - 0.01)):
            yield from Rt.move((q[0], q[1], TOP + 0.012), yaw=gy, duration=1.2)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((Rt.tcp()[0], Rt.tcp()[1], hi), yaw=gy, duration=0.6)
        yield from Rt.move((0.25, -0.30, hi), yaw=0.0, duration=0.8)
        yield from L.wait(2.0)
        log("end")
        return

    park_r = (0.28, -0.30, hi)

    def move_end(xy):
        """Right arm: take the cable 3 cm from its free end, slide the end along the table to ``xy``, let go."""
        p, h, gy = yield from take(Rt, CABLE_LEN - 0.03, TOP + R + over + 0.004)
        yield from Rt.move((xy[0], xy[1], TOP + R + over + 0.004), yaw=gy, duration=1.2)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((xy[0], xy[1], hi), yaw=gy, duration=0.4)
        yield from Rt.move(park_r, yaw=0.0, duration=0.6)

    # 1) right arm slides the free end off to the right of the template (the loose cable lies flat and slack,
    #    off the first half of the band)
    hold = np.array([0.42, -0.05])
    yield from move_end(hold)

    def dump(tag, u):
        if os.environ.get("DEMO_DUMP"):
            poly = ctx.states()["cord"].polyline()
            print(f"DUMP {tag} {u:.3f} " + " ".join(f"{x*1000:.1f},{y*1000:.1f},{(z-TOP)*1000:.1f}"
                                                   for x, y, z in poly), flush=True)

    def place(arm, a, u, tgt=None):
        """Grasp the cable at arclength a, slide it just above the table to band point u (or to ``tgt``), put it
        down there and let go."""
        st = ctx.states()
        p, h = _cord_at(st, a)
        bp, th = path_point(u)
        tgt = bp if tgt is None else tgt
        gy, ty = _grip_pair(h, th, arm.name)
        yield from arm.move((p[0], p[1], hi), yaw=gy, finger=GRASP_OPEN, duration=0.8)
        yield from arm.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.5)
        dump("pre", u)
        yield from arm.grip(FINGER_CLOSED, 0.4)
        dump("grip", u)
        low = p[2] + over + 0.002  # slide it along the table: lifting it peels the laid cable back up
        yield from arm.move((p[0], p[1], low), yaw=gy, duration=0.3)
        dump("lift", u)
        yield from arm.move((tgt[0], tgt[1], max(lay_z, low)), yaw=ty, duration=0.7)
        yield from arm.move((tgt[0], tgt[1], lay_z), yaw=ty, duration=0.4)
        dump("held", u)
        yield from arm.grip(GRASP_OPEN, 0.4)
        dump("open", u)
        yield from arm.move((tgt[0], tgt[1], carry), yaw=ty, duration=0.3)
        dump("rel", u)
        if dbg:
            st = ctx.states()
            pts, s = _cable_arclength(st)
            d = np.linalg.norm(pts[:, :2] - bp, axis=1)
            j = int(d.argmin())
            print(f"DBG {arm.name} u={u:.3f} a={a:.3f} -> cable a={s[j]:.3f} lat={d[j]*1000:.1f}mm "
                  f"z={(pts[j, 2]-TOP)*1000:.1f}mm ok={section_ok(st).astype(int).tolist()} "
                  f"cov={covered_points(st).reshape(N_SECTIONS, -1).sum(axis=1).tolist()} "
                  f"band={in_band(st).astype(int).tolist()}", flush=True)

    def lay(arm, u_prev, a_prev, u):
        """Grasp the cable at the arclength that belongs at band point u and place it there."""
        # the cable is exactly as long as the band, so cable arclength u belongs at band distance u: any extra
        # length fed in would pile up as wiggles and leave the end short of the circle
        a = u
        yield from place(arm, a, u)
        return a

    def fix(arm, u_lo, u_hi, a_hi, tries):
        """Closed-loop touch-up of the sections lying within band distances u_lo..u_hi: while one of them is bad,
        re-place the cable point farthest out of the band there (aiming a little past the centreline), or the
        first uncovered check point."""
        secs = [k for k in range(N_SECTIONS) if k * SEC_LEN >= u_lo - 0.02 and (k + 1) * SEC_LEN <= u_hi + 0.02]
        for _ in range(tries):
            st = ctx.states()
            ok = covered_points(st).reshape(N_SECTIONS, -1).all(axis=1) & in_band(st)
            bad = [k for k in secs if not ok[k]]
            if not bad:
                return
            k0 = bad[0]
            s_lo, s_hi = max(k0 * SEC_LEN - 0.01, 0.02), min((k0 + 1) * SEC_LEN, a_hi)
            pts, s = _cable_arclength(st)
            dd = np.linalg.norm(pts[:, None, :2] - _CL[None], axis=2)
            kk = dd.argmin(axis=1)
            dev = dd[np.arange(len(pts)), kk]
            cand = np.where((s >= s_lo) & (s <= s_hi))[0]
            if len(cand) and dev[cand].max() > 0.008:
                j = cand[dev[cand].argmax()]
                u = float(_CL_U[kk[j]])
                bp, _ = path_point(u)
                yield from place(arm, float(s[j]), u, bp + 0.3 * (bp - pts[j, :2]))
                continue
            cov = covered_points(st).reshape(N_SECTIONS, -1)[k0]
            u = float(SAMPLES.reshape(N_SECTIONS, -1)[k0][int(np.argmin(cov))])
            a0 = _laid_arclength(st, max(u - 0.02, 0.0))
            a = (a0 if a0 is not None else u - 0.02) + 0.02
            yield from place(arm, a, u)

    # 2) left arm lays the first part
    u_prev, a_prev = 0.0, 0.0
    while u_prev < SPLIT_U - 1e-6:
        u = min(u_prev + DEMO_STEP, SPLIT_U) if u_prev > 0 else 0.04
        a_prev = yield from lay(L, u_prev, a_prev, u)
        u_prev = u
        # keep the loose cable slack: if its end lies farther from the last laid point than 85% of the free
        # length, the right arm moves the end closer
        last = path_point(u_prev)[0]
        end = ctx.states()["cord"].end_tip[:2]
        free = CABLE_LEN - a_prev
        d = float(np.linalg.norm(end - last))
        if d > 0.85 * free:
            hold = last + (end - last) * (0.7 * free / d)
            yield from move_end(hold)
    yield from fix(L, 0.0, SPLIT_U, a_prev, 4)
    if cheat == "half":  # stop after the first half: the end is left lying off the template
        yield from L.move((0.22, 0.27, hi), yaw=0.0, finger=FINGER_OPEN, duration=0.8)
        yield from L.wait(2.0)
        log("end")
        return
    # 3) left pins the last laid point while the right arm puts the free end down beyond the band's last turn,
    #    so the loose cable lies roughly along the rest of the band; then the left arm moves clear
    pin, pth = path_point(SPLIT_U - 0.02)
    py = _grip_yaw(pth + math.pi / 2)
    yield from L.move((pin[0], pin[1], carry), yaw=py, finger=FINGER_CLOSED, duration=0.5)
    yield from L.move((pin[0], pin[1], BAND_TOP + 0.002 + PAD_TIP), yaw=py, duration=0.4)
    yield from move_end(np.array([0.27, -0.15]))
    yield from L.move((pin[0], pin[1], hi), yaw=py, duration=0.4)
    yield from L.move((0.22, 0.27, hi), yaw=0.0, finger=FINGER_OPEN, duration=0.6)
    log("left laid")
    while u_prev < PATH_LEN - 0.03 - 1e-6:
        u = min(u_prev + DEMO_STEP, PATH_LEN - 0.03)
        a_prev = yield from lay(Rt, u_prev, a_prev, u)
        u_prev = u
    yield from fix(Rt, SPLIT_U - 0.01, PATH_LEN, CABLE_LEN, 5)
    if cheat == "hold":  # the cable is laid; the right arm takes it 2 cm from its end and keeps holding it there
        p, h = _cord_at(ctx.states(), CABLE_LEN - 0.02)  # stay low: never let go of the cable for a second
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], carry), yaw=gy, finger=GRASP_OPEN, duration=0.6)
        yield from Rt.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.4)
        yield from Rt.grip(FINGER_CLOSED, 0.4)
        log("holding")
        yield from Rt.wait(3.0)
        log("end")
        return
    yield from Rt.move((0.30, -0.28, hi), yaw=0.0, finger=FINGER_OPEN, duration=0.8)
    log("laid")
    # touch-up: re-lay where a check point is still bad (closed loop)
    yield from L.wait(1.0)
    log("end")
