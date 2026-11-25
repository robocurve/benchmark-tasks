"""Reconnect a pop-up stopper under a wall-hung basin: rod through the stopper's hole into the drain's rear port, pivot
nut threaded on until snug, rod pressed down so the stopper lifts open. Scene in ``scene.py``, figures in ``parts.py``.
The thread catches only where the nut is set square on the port's first thread; a hidden carrier then takes it 1.6 mm
a turn toward the drain, from where it stands, to the stop. Nothing writes the nut's or the rod's pose.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.5  # the arms are back at ready before the end
INSTRUCTION = (
    "Push the pivot rod through the stopper's hole into the drain port, thread the pivot nut on until snug, press the "
    "rod down so the stopper lifts open, then let go and move both hands clear."
)
RUN_LIMIT_STEPS = 1800  # 90 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}
_HULLS: dict = {}
_LINKS: dict = {}


def _in_sim() -> bool:
    return mk.MECH.environment is not None


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(seed=seed, zero=None, hold=0, hold_from=None, hands=None, carried_hidden=False, unthread_wait=False,
                 turn_rest=dict.fromkeys(scene.TURN_JOINTS, 0.0), catch=None, peak_cw_deg=0.0, peak_gap=0.0, events=[])
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if not _in_sim():
        return
    for name in scene.HIDDEN_NAMES:
        mk.show(name, 0, False)
    mk.set_limits("thread_s", lower=-P.SLIDE_PLAY, upper=P.SLIDE_PLAY)
    mk.set_joint("thread_s", 0.0)
    for j in scene.TURN_JOINTS:
        mk.set_spring(j, rest=0.0, most=P.THREAD_DRAG_NM)
        mk.set_joint(j, 0.0)


def _note(msg: str) -> None:
    STATE.setdefault("events", []).append((round(mk.now(), 2) if mk.MECH.poses else 0.0, msg))


# --- readings ------------------------------------------------------------------------------------------------------
def stopper_gap() -> float:
    """m the stopper's seal stands over the flange (0 closed)."""
    return float(mk.pose("stopper")[0][2] - (TOP + P.FLANGE_Z))


def stopper_radial() -> float:
    return float(math.hypot(*(mk.pose("stopper")[0][:2] - (P.X_DRAIN, P.Y_DRAIN))))


def stopper_tilt_deg() -> float:
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(mk.pose("stopper")[1][2, 2]))))))


def ball_seat_offset() -> float:
    """m from the ball's centre to its seat in the port."""
    seat = np.array([P.X_DRAIN + P.SEAT_OFFSET_X, P.Y_DRAIN, TOP + P.PORT_Z])
    return float(np.linalg.norm(mk.pose("pivot_rod")[0] - seat))


def rod_pitch_deg() -> float:
    """How far the rod's rear end is pressed down (deg; positive lifts the tip)."""
    return float(math.degrees(math.asin(max(-1.0, min(1.0, -float(mk.pose("pivot_rod")[1][2, 0]))))))


def hole_crossing() -> tuple[bool, float]:
    """Whether the rod's axis crosses the stopper's shaft inside its hole, and how far (m) the tip's end is through."""
    sp, sr = mk.pose("stopper")
    rp, rr = mk.pose("pivot_rod")
    ball = (rp - sp) @ sr
    tip = (rp + rr @ np.array([-P.TIP_REACH, 0.0, 0.0]) - sp) @ sr
    if ball[0] * tip[0] >= 0.0:
        return False, 0.0
    q = ball + (ball[0] / (ball[0] - tip[0])) * (tip - ball)
    inside = abs(float(q[1])) <= P.EYE_W / 2.0 and abs(float(q[2]) + P.EYE_DEPTH) <= P.EYE_H / 2.0 + 0.001
    return inside, abs(float(tip[0]))


def rod_in_port() -> bool:
    """The rod is in the port through the stopper's hole: ball within 3 mm of its seat, tip 3 mm or more past the
    stopper's middle."""
    inside, through = hole_crossing()
    return ball_seat_offset() <= P.SEAT_MAX and inside and through >= P.TIP_PAST


def nut_front_u() -> float:
    return float(mk.pose("pivot_nut")[0][0] - P.X_DRAIN)


def nut_xy_off() -> float:
    """m from the nut's front face centre to the port's axis."""
    p = mk.pose("pivot_nut")[0]
    return float(math.hypot(p[1] - P.Y_DRAIN, p[2] - (TOP + P.PORT_Z)))


def nut_tilt_deg() -> float:
    """Angle (deg) between the nut's axis and the port's axis."""
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(mk.pose("pivot_nut")[1][0, 0]))))))


def land_gap() -> float:
    """m from the thread start (the land's rear face) back to the nut's front face (0 when the nut is set on it)."""
    return nut_front_u() - P.U_START


def thread_engaged() -> bool:
    return not mk.is_cut("w_thread")


def thread_cw_deg() -> float:
    return float(math.degrees(sum(mk.joint(j) for j in scene.TURN_JOINTS)))


def nut_snug() -> bool:
    return thread_engaged() and thread_cw_deg() >= P.STOP_DEG - P.SNUG_DEG


def thread_start_ok() -> bool:
    """The nut set on the thread start (at most 0.4 mm onto the washer), coaxial within 0.5 mm, square within 1.5 deg."""
    return bool(-P.CATCH_GAP_BAND <= land_gap() <= 0.0001 and nut_xy_off() <= P.CATCH_XY_MAX and nut_tilt_deg() <= P.CATCH_TILT_DEG)


def zero_now() -> str | None:
    if mk.pose("pivot_rod")[0][2] - TOP < P.FALL_Z:
        return "the pivot rod fell onto the bench"
    if mk.pose("pivot_nut")[0][2] - TOP < P.FALL_Z:
        return "the pivot nut fell onto the bench"
    return None


REST_BODIES = ("stopper", "pivot_rod", "pivot_nut")


def _track_speeds() -> None:
    """Speeds (m/s) from each body's move since the last control step (the welded nut's simulator velocity reads 55 mm/s
    at rest)."""
    now, t = {n: mk.pose(n)[0].copy() for n in REST_BODIES}, mk.now()
    prev, t_prev = STATE.get("rest_prev"), STATE.get("rest_t")
    if prev is not None and t > t_prev:
        STATE["speeds"] = {n: float(np.linalg.norm(now[n] - prev[n])) / (t - t_prev) for n in REST_BODIES}
    STATE["rest_prev"], STATE["rest_t"] = now, t


def at_rest() -> bool:
    """The stopper, rod and nut each moved under 25 mm/s since the last control step."""
    speeds = STATE.get("speeds")
    if speeds is None:
        return False
    return all(v <= P.REST_SPEED_MAX for v in speeds.values())


# --- hands ---------------------------------------------------------------------------------------------------------
def read_hands() -> None:
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        pts = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pw = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pw[3:7]).T + pw[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    pos, rot = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - pos) @ rot
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    """Nearest distance (m) from a hand to the stopper, rod, nut or drain."""
    hands = STATE.get("hands")
    if hands:
        pts = np.vstack([hands[s] for s in SIDES if s in hands])
    elif mk.MECH.grippers:
        pts = np.vstack([mk.gripper(s)[0].reshape(1, 3) for s in SIDES if s in mk.MECH.grippers])
    else:
        return 0.0
    return min(_box_distance(pts, n) for n in ("stopper", "pivot_rod", "pivot_nut", "drain"))


# --- the thread ----------------------------------------------------------------------------------------------------
def _carrier_to(cw_deg: float) -> float:
    z_rel = -P.THREAD_PITCH * cw_deg / 360.0
    mk.set_limits("thread_s", lower=z_rel - P.SLIDE_PLAY, upper=z_rel + P.SLIDE_PLAY)
    return z_rel


def _step_thread() -> None:
    if _in_sim() and not STATE.get("carried_hidden", False):
        for name in scene.HIDDEN_NAMES:
            mk.push(name, force=(0.0, 0.0, P.HIDDEN_MASS * P.GRAVITY))
        STATE["carried_hidden"] = True

    if not thread_engaged():
        if STATE.get("unthread_wait", False):
            if land_gap() > P.REARM_GAP:
                STATE["unthread_wait"] = False
                _note("thread start re-armed")
            return
        if thread_start_ok():
            # Catch where the nut stands: the carrier (never the nut) goes to the nut's thread travel, the weld joins them
            cw = min(max(P.cw_for_gap_deg(nut_front_u() - P.U_SNUG), 0.0), P.STOP_DEG - 1.0)
            if _in_sim():
                z_rel = _carrier_to(cw)
                mk.set_joint("thread_s", z_rel)
                per = math.radians(cw) / len(scene.TURN_JOINTS)
                for j in scene.TURN_JOINTS:
                    mk.set_joint(j, per)
                    mk.set_spring(j, rest=per, most=P.THREAD_DRAG_NM)
                    STATE["turn_rest"][j] = per
            mk.make("w_thread")
            STATE["catch"] = {"t": round(mk.now(), 2) if mk.MECH.poses else 0.0, "cw_deg": round(cw, 2),
                              "land_gap_mm": round(1000.0 * land_gap(), 3), "xy_off_mm": round(1000.0 * nut_xy_off(), 3),
                              "tilt_deg": round(nut_tilt_deg(), 3)}
            _note(f"thread caught where the nut stands: cw={cw:.1f} deg")
        return

    cw = thread_cw_deg()
    STATE["peak_cw_deg"] = max(STATE.get("peak_cw_deg", 0.0), cw)
    if cw <= P.UNTHREAD_DEG:
        mk.cut("w_thread")
        STATE["unthread_wait"] = True
        if _in_sim():
            mk.set_limits("thread_s", lower=-P.SLIDE_PLAY, upper=P.SLIDE_PLAY)
            mk.set_joint("thread_s", 0.0)
            for j in scene.TURN_JOINTS:
                mk.set_joint(j, 0.0)
                mk.set_spring(j, rest=0.0, most=P.THREAD_DRAG_NM)
                STATE["turn_rest"][j] = 0.0
        _note(f"nut turned off its thread at cw={cw:.1f} deg")
        return
    if _in_sim():
        _carrier_to(cw)
        lag = P.THREAD_DRAG_NM / P.THREAD_SPRING_K
        cap = math.radians(30.0)
        for j in scene.TURN_JOINTS:
            ang, r = mk.joint(j), STATE["turn_rest"][j]
            if ang > r + lag:
                r = ang - lag
            elif ang < r - lag:
                r = ang + lag
            r = max(ang - cap, min(ang + cap, r))
            STATE["turn_rest"][j] = r
            mk.set_spring(j, rest=r, most=P.THREAD_DRAG_NM)


# --- scoring -------------------------------------------------------------------------------------------------------
def stopper_open() -> bool:
    return (P.GAP_OPEN_MIN <= stopper_gap() <= P.GAP_OPEN_MAX and stopper_radial() <= P.STOP_RADIAL_MAX
            and stopper_tilt_deg() <= P.STOP_TILT_MAX_DEG)


def full_now() -> bool:
    return rod_in_port() and nut_snug() and stopper_open() and hands_clear() >= P.HANDS_CLEAR and at_rest()


@mk.on_step
def step() -> None:
    if _in_sim():
        read_hands()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    _step_thread()
    _track_speeds()
    STATE["peak_gap"] = max(STATE.get("peak_gap", 0.0), stopper_gap())
    if full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = mk.pose("stopper")[0].copy()
        STATE["hold"] = STATE.get("hold", 0) + 1
        if float(np.linalg.norm(mk.pose("stopper")[0] - STATE["hold_from"])) > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if rod_in_port() and nut_snug():
        return 0.5
    if rod_in_port():
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for the whole trial if the pivot rod's ball or the pivot nut drops below 30 mm over the bench. The nut counts
    as threaded only if its thread caught at a square start: its front face set on the port's first thread, coaxial
    within 0.5 mm and square within 1.5 deg (a nut started crooked does not catch). At the end: 1.0 when the rod is
    in the port through the stopper's hole (ball within 3 mm of its seat, tip 3 mm or more past the stopper's
    middle), the nut is threaded on snug (within 20 deg of where its thread stops), and the stopper is lifted 8 to 22
    mm over the flange (within 4 mm of the drain's axis, within 8 deg of upright) for 1.0 s with at most 2 mm drift,
    both hands at least 30 mm clear of the stopper, rod, nut and drain, and the stopper, rod and nut still (under 25
    mm/s). 0.5 when the rod is in the port through the stopper's hole and the nut is threaded on snug, short of 1.0.
    0.25 when the rod is in the port through the stopper's hole, the nut not snug. 0 else."""
    _ = objects
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"zero": STATE.get("zero"), "hold": STATE.get("hold", 0), "catch": STATE.get("catch"), "events": STATE.get("events", []),
           "peak_cw_deg": round(STATE.get("peak_cw_deg", 0.0), 1), "peak_gap_mm": round(1000.0 * STATE.get("peak_gap", 0.0), 2)}
    if mk.MECH.poses:
        inside, through = hole_crossing()
        pairs = {"rod-drain": ("pivot_rod", "drain"), "rod-stopper": ("pivot_rod", "stopper"), "nut-rod": ("pivot_nut", "pivot_rod"),
                 "nut-drain": ("pivot_nut", "drain"), "stopper-drain": ("stopper", "drain")}
        mm = {"stopper_gap": stopper_gap(), "stopper_radial": stopper_radial(), "ball_seat": ball_seat_offset(), "tip_through": through,
              "nut_land_gap": land_gap(), "nut_xy_off": nut_xy_off(), "hands_clear": hands_clear()}
        out.update({f"{k}_mm": round(1000.0 * v, 2) for k, v in mm.items()})
        out.update(rod_in_hole=inside, rod_pitch_deg=round(rod_pitch_deg(), 2), stopper_tilt_deg=round(stopper_tilt_deg(), 2),
                   thread_engaged=thread_engaged(), thread_cw_deg=round(thread_cw_deg(), 1), nut_tilt_deg=round(nut_tilt_deg(), 2),
                   forces_N={k: round(mk.contact_force(a, b), 2) for k, (a, b) in pairs.items()}, score=score(None))
    return out
