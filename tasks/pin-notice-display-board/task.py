"""Pin a notice to a tilted display board, its top edge at the line.

A cork display board stands on a desk easel at 30 deg from the table, with a 12 mm ledge along its bottom edge. An A6
laminated notice rests on the ledge. A pushpin stands needle-down in a pin cup. Two black ticks at the sides of the
board mark a line 2.5 cm above the notice's resting top edge. The notice has to be slid up the board to the line,
held there, and pinned through with the pushpin; then both arms let go and it hangs from the pin.

Cork model
- The needle is a collider on a prismatic slider in the pin's head with F_INS of Coulomb friction. The tip stops on
  the board face; pushing the head harder than F_INS drives the needle back into the head, and how far it has gone
  back is how far the pin is in. The collar stops on the notice.
- A pin left in the cork is held by it: once the collar seats (needle back by at least SEAT_BACK), or the fingers
  open with the tip 2 mm or more in, the pin's pose is written back every physics substep (a kinematic hold). At
  that moment it is moved out along its axis just enough for the collar to clear the notice. Pulling a seated pin
  out again is not modelled.
- Needle through the notice: a notice with a seated pin through it (3 mm or more from its edges) is tethered at that
  point to the pin's axis by a spring-damper (K_PIV, C_PIV, at most F_PIV_MAX) every substep: it can slide along the
  needle, not away from it. The head's clamp on it is a torsional spring-damper about the pin (K_ROT, C_ROT) that
  holds the turn it was pinned at up to T_ROT_MAX (the clamp's friction torque), beyond which it slips.

Scoring (score(), judged at the end; success() ends the episode once 1.0 has held for SUCCESS_HOLD_S)
  0.25  the pin stands in the cork (2 mm or more in) and has been let go, nothing else
  0.5   the pin is through the notice with its head seated on it, and let go, but the notice isn't at the line
  0.75  the notice is pinned at the line but a gripper is still within reach of it
  1.0   pinned at the line (top edge within 10 mm of the line, long edges within 5 deg of the board's up
        direction), both TCPs clear (3 cm or more off the board face and 4 cm or more from the pin), and still (its
        top edge has moved under 2 mm and it has turned under 1 deg over the last STILL_S seconds)
  0     otherwise (also if the notice ends off the board)

Env knobs (defaults are the task): F_INS (N, 6), PIN_TILT (deg, 30, demo only), CARD_PRESS (mm, 15, demo only),
OPEN_S (s, 2.5, demo only), PIN_DEBUG=1 prints a trace every 4 control steps (every step over the release).
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import partkit  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from partkit import Slider, _np, _t, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
TOP = TABLE_TOP[SCENE]
DEBUG = os.environ.get("PIN_DEBUG", "") not in ("", "0")
TH = math.radians(30.0)                                # the board's tilt from the table
PRESS = float(os.environ.get("CARD_PRESS", "15")) / 1000
F_INS = float(os.environ.get("F_INS", "6"))            # push needed to drive the pin into the cork (N)
BOARD_MU = 0.3
CARD_MU = 1.0

# --- tilted grip for the demo: yaw values > 50 encode a tilt of (yaw - 100) deg about world y ---------------------
_orig_grip = _demo.grip_rotation


def _grip(yaw: float = 0.0):
    if yaw > 50:                                       # >= 1000: also turned 90 deg (fingers close up/down the board)
        turn = -math.pi / 2 if yaw >= 1000 else 0.0
        return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.radians((yaw % 1000) - 100)) @ _orig_grip(turn)
    return _orig_grip(yaw)


_demo.grip_rotation = _grip
TILT = 100 + float(os.environ.get("PIN_TILT", "30"))
TILT90 = TILT + 1000

# --- geometry -----------------------------------------------------------------------------------------------------
D = np.array([math.cos(TH), 0.0, math.sin(TH)])        # up the board face
N = np.array([-math.sin(TH), 0.0, math.cos(TH)])       # out of the face
Y = np.array([0.0, 1.0, 0.0])
E = np.array([0.12, 0.0, 0.03])                        # low edge of the face, relative to (0, 0, TOP)
Q = (math.cos(TH / 2), 0.0, -math.sin(TH / 2), 0.0)    # wxyz, rotates box x->D, z->N
QM = partkit._quat_matrix(Q)
L, W, T, YC = 0.30, 0.32, 0.012, 0.06                  # face: 30 up, 32 across (y -0.10 to 0.22), 12 mm thick
CORK, LIP, GREY, RED, WHITE, BLACK = (0.72, 0.55, 0.36), (0.45, 0.3, 0.18), (0.4, 0.4, 0.42), (0.85, 0.1, 0.1), \
    (0.95, 0.95, 0.92), (0.05, 0.05, 0.05)
FACE0 = np.array([0, 0, TOP]) + E
U_LINE = 0.175                                         # the line, up the face from the ledge
LINE_TOL = 0.010                                       # the notice's top edge within this of the line
TWIST_TOL = 5.0                                        # deg, its long edges to the board's up direction
STILL_S, STILL_MM, STILL_DEG = 0.5, 2.0, 1.0          # still: top edge moved under 2 mm, turned under 1 deg, in 0.5 s


def on_face(u, y, out=0.0):
    return FACE0 + D * u + N * out + np.array([0, y, 0])


def rel(p):
    return tuple(float(v) for v in (np.asarray(p) - np.array([0, 0, TOP])))


_top_back = on_face(L - 0.02, YC, -T)                  # where the rear strut meets the board
BOARD = part("board", pos=(0, 0, TOP), mass=None, color=CORK, friction=BOARD_MU, boxes=[
    (rel(on_face(L / 2, YC, -T / 2)), (L, W, T), CORK, Q),          # box_0: the cork face
    (rel(on_face(-0.005, YC, 0.006)), (0.01, W, 0.012), LIP, Q),      # the ledge, 12 mm proud
    ((0.12, YC, 0.015), (0.03, W, 0.03), GREY),                       # the stand's foot
    ((float(_top_back[0]), YC, float(_top_back[2] - TOP) / 2), (0.03, 0.03, float(_top_back[2] - TOP)), GREY),
    (rel(on_face(U_LINE, YC - W / 2 + 0.012, 0.00015)), (0.002, 0.02, 0.0003), BLACK, Q),   # line ticks
    (rel(on_face(U_LINE, YC + W / 2 - 0.012, 0.00015)), (0.002, 0.02, 0.0003), BLACK, Q),
])

CARD_T, CARD_LEN, CARD_WID = 0.001, 0.148, 0.105       # laminated A6 notice
CARD_Y = 0.07
CARD_U0 = 0.0015 + CARD_LEN / 2
CARD_C0 = on_face(CARD_U0, CARD_Y, CARD_T / 2 + 0.0001)
CARD = part("card", pos=tuple(float(v) for v in CARD_C0), mass=0.006, color=WHITE, friction=CARD_MU,
            boxes=[((0, 0, 0), (CARD_LEN, CARD_WID, CARD_T), WHITE, Q)])

# pushpin: head 9 x 11 mm (box + cylinder) over a collar 11 x 2 mm; 8 mm steel needle below the collar
NEEDLE_LEN = 0.008
NEEDLE_TIP = -0.0075 - NEEDLE_LEN                       # tip below the head's centre (pin frame z), needle fully out
PIN_XY = (0.25, -0.17)
HOLE, HOLD_H = 0.0015, 0.02
PIN_Z0 = TOP + HOLD_H + 0.0075
TRAVEL_N = NEEDLE_LEN - 0.0005                         # the needle can go back 7.5 mm (into the cork)
NEEDLE = Slider("needle", rest=(0, 0, -0.0075 - NEEDLE_LEN / 2 + TRAVEL_N),
                boxes=(((0, 0, -0.0075 - NEEDLE_LEN / 2 + TRAVEL_N), (0.0016, 0.0016, NEEDLE_LEN), GREY),),
                travel=TRAVEL_N, rate=10.0, damping=0.5, mass=0.002, axis="Z", start=-TRAVEL_N)
PIN = part("pin", pos=(*PIN_XY, PIN_Z0), mass=0.005, color=RED, friction=0.8,
           boxes=[((0, 0, 0), (0.006, 0.006, 0.009), RED)],
           cylinders=[((0, 0, 0), 0.0045, 0.011, RED), ((0, 0, -0.0065), 0.0055, 0.002, RED)],
           sliders=(NEEDLE,))
partkit.JOINT_FRICTION["needle"] = {"friction": F_INS, "dynamic_friction": 0.9 * F_INS}
SEAT_BACK = NEEDLE_LEN - CARD_T - 0.0002               # needle back this far: the collar is on the notice
PINNED_BACK = NEEDLE_LEN - CARD_T - 0.0015             # head within 1.5 mm of the notice: pinned through it
CLEAR = 0.0001                                          # collar-to-notice clearance of a held pin
GRIP_OPEN = -0.0055                                     # finger joint beyond this: the fingers have let go of the pin
K_PIV, C_PIV, F_PIV_MAX = 60.0, 0.25, 2.0               # needle-through-notice tether
K_ROT, C_ROT, T_ROT_MAX = 0.5, 0.002, 0.003             # the head's clamp on the notice: holds its turn about the pin
CUP = part("cup", pos=(*PIN_XY, TOP), mass=None, color=GREY, boxes=[            # pin cup: a 3 mm hole, 2 cm deep
    ((0, HOLE + 0.01, 0.01), (0.04, 0.02, 0.02), GREY), ((0, -HOLE - 0.01, 0.01), (0.04, 0.02, 0.02), GREY),
    ((HOLE + 0.01, 0, 0.01), (0.02, 2 * HOLE, 0.02), GREY), ((-HOLE - 0.01, 0, 0.01), (0.02, 2 * HOLE, 0.02), GREY)])

OBJECTS = [BOARD, CARD, PIN, CUP]
PINS = ("pin",)
TCP_OFF_FACE, TCP_FROM_PIN = 0.03, 0.04                 # "let go": every TCP this far off the face and from the pin

# --- spawn hook: board friction min-combined (the cork decides its contacts' friction) ------------------------------
_orig_spawn = partkit._spawn_part


def _spawn(prim_path, cfg, *a, **k):
    prim = _orig_spawn(prim_path, cfg, *a, **k)
    if cfg.part_name == "board":
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import PhysxSchema
        mat = get_current_stage().GetPrimAtPath(f"{prim_path}/Looks/physics")
        PhysxSchema.PhysxMaterialAPI.Apply(mat).CreateFrictionCombineModeAttr().Set("min")
    return prim


partkit._spawn_part = _spawn

# --- state ---------------------------------------------------------------------------------------------------------
_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(step=0, pins={}, card_pins=[], score=0.0, why="start", ep_len=-1)


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return partkit._quat_matrix((w, x, y, z))


def _pose(sc, name):
    p = _np(sc[name].data.root_link_pose_w)[0].copy()
    p[:3] -= _np(sc.env_origins[0])
    return p


def _vel(sc, name):
    d = sc[name].data
    for attr in ("root_com_vel_w", "root_vel_w", "root_link_vel_w"):
        if hasattr(d, attr):
            return _np(getattr(d, attr))[0]
    return np.zeros(6)


def _write(sc, name, pos, quat_xyzw, lin=(0, 0, 0), ang=(0, 0, 0)):
    import torch
    asset = sc[name]
    origin = _t(sc.env_origins[0])
    pose = torch.tensor([[*map(float, pos), *map(float, quat_xyzw)]], dtype=torch.float32, device=origin.device)
    pose[:, :3] += origin.to(torch.float32)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([[*map(float, lin), *map(float, ang)]], dtype=torch.float32, device=origin.device)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


def _fingers(sc):
    """The right arm's finger joint positions (m; 0 closed, FINGER_OPEN open), or None."""
    key = _S.get("right_key")
    if key is None:
        for k in sc.articulations.keys():
            jn = list(sc.articulations[k].joint_names)
            if "joint7" in jn and "right" in k:
                key = k
                _S["right_key"], _S["finger_ids"] = k, (jn.index("joint7"), jn.index("joint8"))
                break
        else:
            return None
    q = _np(sc.articulations[key].data.joint_pos)[0]
    i, j = _S["finger_ids"]
    return float(q[i]), float(q[j])


def _install_substep(sc):
    """Run _substep before every physics substep: the env calls scene.write_data_to_sim() once per substep."""
    if getattr(sc, "_pin_substep", False):
        return
    orig = sc.write_data_to_sim

    def write_data_to_sim():
        try:
            _substep(sc)
        except Exception as e:  # noqa: BLE001
            if not _S.get("sub_err"):
                import traceback
                _S["sub_err"] = 1
                print(f"[pin] substep error {e!r}\n{traceback.format_exc()}", flush=True)
        return orig()

    sc.write_data_to_sim = write_data_to_sim
    sc._pin_substep = True


def _substep(sc):
    _S["sub"] = _S.get("sub", 0) + 1
    for name, fx in _S.get("fix", {}).items():
        _write(sc, name, fx["pos"], fx["quat"])            # the cork holds the pin
    piv = _S.get("pivot")
    if piv:
        _tether(sc, piv)


def _tether(sc, piv):
    """Needle through the notice: a spring-damper pulls the notice's hole point onto the pin's axis (never along it)."""
    import torch
    card = sc["card"]
    pose = _np(card.data.root_link_pose_w)[0]
    vel = _vel(sc, "card")
    if not (np.all(np.isfinite(pose)) and np.all(np.isfinite(vel))):
        return
    origin = _np(sc.env_origins[0])
    p = pose[:3] - origin
    hole = p + _rot(pose[3:7]) @ QM @ piv["loc"]
    a = piv["axis"]
    e = hole - piv["point"]
    e -= a * float(e @ a)
    vh = vel[:3] + np.cross(vel[3:6], hole - p)
    vh -= a * float(vh @ a)
    f = -K_PIV * e - C_PIV * vh
    n = float(np.linalg.norm(f))
    if n > F_PIV_MAX:
        f *= F_PIV_MAX / n
    piv["err"], piv["f"] = float(np.linalg.norm(e)), min(n, F_PIV_MAX)
    # the head clamps the notice to the cork: a torsional spring-damper about the pin holds the turn it was pinned
    # at, up to the clamp's friction torque (T_ROT_MAX), beyond which it slips
    xa = _rot(pose[3:7]) @ QM @ np.array([1.0, 0.0, 0.0])
    xa -= a * float(xa @ a)
    x0 = piv["x0"]
    turn = math.atan2(float(np.cross(x0, xa) @ a), float(x0 @ xa))
    tq = -K_ROT * turn - C_ROT * float(vel[3:6] @ a)
    tq = max(-T_ROT_MAX, min(T_ROT_MAX, tq))
    piv["turn"] = math.degrees(turn)
    comp = card.permanent_wrench_composer
    comp.reset()
    dev = _t(sc.env_origins[0]).device
    ft = torch.tensor([[[float(v) for v in f]]], dtype=torch.float32, device=dev)
    tt = torch.tensor([[[float(v) for v in a * tq]]], dtype=torch.float32, device=dev)
    pt = torch.tensor([[[float(v) for v in hole + origin]]], dtype=torch.float32, device=dev)
    comp.add_forces_and_torques_index(forces=ft, torques=tt, positions=pt, body_ids=[0], is_global=True)


def _card_local(card_pose, world):
    """Point in the notice box's frame (x along its length, y across, z out of it)."""
    return QM.T @ (_rot(card_pose[3:7]).T @ (np.asarray(world) - card_pose[:3]))


def collar_shift(a, collar, nrm, ref_pt):
    """How far to move a pin out along its axis ``a`` so its collar's low edge clears the plane (``nrm``, ``ref_pt``)."""
    cos = float(a @ nrm)
    d = float((collar - ref_pt) @ nrm)
    dip = 0.0055 * math.sqrt(max(0.0, 1.0 - cos * cos))   # the collar's low edge, if the pin is tilted
    return max(0.0, (CLEAR + dip - d) / max(cos, 0.5)), d


def _fix_pin(sc, name, p, r, tip, card, depth, off):
    """The cork takes hold of the pin: fix its pose, moved out along its axis so the collar just clears the notice."""
    a = r[:, 2]                                           # pin axis, head side (out of the board)
    collar = p[:3] - 0.0075 * a                           # collar bottom, centre
    loc = _card_local(card, tip)
    if abs(loc[0]) <= CARD_LEN / 2 and abs(loc[1]) <= CARD_WID / 2:
        nrm = _rot(card[3:7]) @ QM @ np.array([0.0, 0.0, 1.0])
        ref_pt, ref = card[:3] + nrm * (CARD_T / 2), "notice"
    else:
        nrm, ref_pt, ref = N, FACE0, "board"
    shift, d = collar_shift(a, collar, nrm, ref_pt)
    pos = p[:3] + a * shift
    _S.setdefault("fix", {})[name] = {"pos": pos, "quat": p[3:7].copy(), "tip": tip + a * shift, "axis": a.copy()}
    print(f"[pin] step {_S['step']}: {name} held by the cork, {depth * 1000:.1f} mm in, {off:.1f} deg off square, "
          f"collar {d * 1000:+.2f} mm off the {ref}, moved out {shift * 1000:.2f} mm", flush=True)
    for pin_name, _, hole_loc in _S["card_pins"]:
        if pin_name == name:
            xa = _rot(card[3:7]) @ QM @ np.array([1.0, 0.0, 0.0])
            xa -= a * float(xa @ a)
            _S["pivot"] = {"pin": name, "point": tip + a * shift, "axis": a.copy(), "loc": hole_loc,
                           "x0": xa / np.linalg.norm(xa)}
            print(f"[pin] step {_S['step']}: notice hangs on {name} at notice-local {hole_loc.round(4)}", flush=True)
    _write(sc, name, pos, p[3:7])


def _tcps(sc):
    out = {}
    for key in sc.articulations.keys():
        art = sc.articulations[key]
        bn = list(art.body_names)
        if "gripper" in bn:
            p = _np(art.data.body_link_pose_w)[0, bn.index("gripper")]
            p[:3] -= _np(sc.env_origins[0])
            out[key] = p[:3] + _rot(p[3:7]) @ np.array(TCP_OFFSET)
    return out


def rule(pin_in, pinned, at_line, clear, still):
    """The score from the five findings (see the module docstring)."""
    if pinned and at_line:
        return 1.0 if clear and still else 0.75
    if pinned:
        return 0.5
    return 0.25 if pin_in else 0.0


def card_findings(card_pose, history=()):
    """(top edge's height up the face, twist deg, on the board, still) of the notice; ``history`` holds the
    (top edge point, twist) of the last STILL_S seconds."""
    r = _rot(card_pose[3:7]) @ QM
    top = card_pose[:3] + r @ np.array([CARD_LEN / 2, 0, 0])
    u_top = float((top - FACE0) @ D)
    xa = r @ np.array([1.0, 0, 0])
    twist = math.degrees(math.atan2(float(xa @ Y), float(xa @ D)))
    out = float((card_pose[:3] - FACE0) @ N)
    on_board = -0.005 < out < 0.02 and 0 <= float((card_pose[:3] - FACE0) @ D) <= L and abs(card_pose[1] - YC) <= W / 2
    still = bool(history) and all(np.linalg.norm(top - t) < STILL_MM / 1000 and abs(twist - a) < STILL_DEG
                                  for t, a in history)
    return u_top, twist, on_board, still, top


def _update(environment):
    base = environment.unwrapped
    sc = base.scene
    _install_substep(sc)
    ep = getattr(base, "episode_length_buf", None)
    if ep is not None:
        ep = int(_np(ep)[0])
        if ep < _S.get("ep_len", -1):                      # a reset happened: start over
            _reset()
        _S["ep_len"] = ep
    tcps = _tcps(sc)
    fingers = _fingers(sc)
    card = _pose(sc, "card")
    log = []
    pin_in = pinned = False
    for name in PINS:
        p = _pose(sc, name)
        r = _rot(p[3:7])
        art = sc[name]
        q = float(_np(art.data.joint_pos)[0][list(art.joint_names).index("needle_slide")])
        back = q + TRAVEL_N                              # 0 = needle fully out, TRAVEL_N = all the way in
        tip = p[:3] + r @ np.array([0, 0, NEEDLE_TIP + back])
        h = float((tip - FACE0) @ N)
        uu, yy = float((tip - FACE0) @ D), float(tip[1])
        off = math.degrees(math.acos(max(-1.0, min(1.0, float(r[:, 2] @ N)))))
        held = any(np.linalg.norm(t - p[:3]) < 0.025 for t in tcps.values())
        gripped = held and fingers is not None and min(fingers) > GRIP_OPEN
        in_cork = 0 <= uu <= L and abs(yy - YC) <= W / 2 and h < CARD_T + 0.001 and off < 25
        depth = back if in_cork else 0.0
        st = _S["pins"].setdefault(name, {})
        if name not in [c[0] for c in _S["card_pins"]] and depth >= PINNED_BACK:
            loc = _card_local(card, tip)
            if abs(loc[0]) <= CARD_LEN / 2 - 0.003 and abs(loc[1]) <= CARD_WID / 2 - 0.003:
                _S["card_pins"].append((name, tip.copy(), loc.copy()))
                print(f"[pin] step {_S['step']}: {name} is through the notice at notice-local {loc.round(4)}",
                      flush=True)
        fixed = name in _S.get("fix", {})
        if not fixed and in_cork and (back >= SEAT_BACK or (depth >= 0.002 and not gripped)):
            st["lock"] = depth
            _fix_pin(sc, name, p, r, tip, card, depth, off)
            fixed = True
        if fixed:
            depth = st["lock"]
        let_go = all(np.linalg.norm(t - p[:3]) >= TCP_FROM_PIN for t in tcps.values())
        pin_in |= fixed and let_go
        pinned |= fixed and let_go and _S.get("pivot", {}).get("pin") == name
        if not np.all(np.isfinite(p)) and not _S.get("nan"):
            _S["nan"] = 1
            print(f"[pin] first NaN pose for {name} at step {_S['step']}", flush=True)
        log.append(f"{name}: back={back * 1000:4.1f}mm depth={depth * 1000:4.1f}mm h={h * 1000:6.1f}mm "
                   f"off={off:4.1f} held={int(held)} grip={int(gripped)} fix={int(fixed)}")
    hist = _S.setdefault("hist", [])
    u_top, twist, on_board, still, top = card_findings(card, hist)
    hist.append((top, twist))
    del hist[:-max(1, round(STILL_S * 20))]
    at_line = on_board and abs(u_top - U_LINE) <= LINE_TOL and abs(twist) <= TWIST_TOL
    clear = all(float((t - FACE0) @ N) >= TCP_OFF_FACE for t in tcps.values())
    score = rule(pin_in, pinned and on_board, at_line, clear, still)
    why = (f"pin_in={int(pin_in)} pinned={int(pinned)} on_board={int(on_board)} top={u_top * 1000:.1f}mm "
           f"(line {U_LINE * 1000:.0f}) twist={twist:.1f}deg at_line={int(at_line)} clear={int(clear)} still={int(still)}")
    if score != _S["score"]:
        print(f"[pin] step {_S['step']}: score {score} ({why})", flush=True)
    _S["score"], _S["why"] = score, why
    return log, card, tcps, why


@on_step
def _log(environment):
    if "pins" not in _S:
        _reset()
    _S["step"] += 1
    n = _S["step"]
    sub = _S.pop("sub", 0)
    try:
        log, card, tcps, why = _update(environment)
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[pin] update error {e!r}\n{traceback.format_exc()}", flush=True)
        raise
    if not DEBUG:
        return
    if n == 2:
        print(f"[pin] {sub} physics substeps per control step", flush=True)
    dense = 396 <= n <= 500
    if n % 4 and not dense:
        return
    if dense and (f := _fingers(environment.unwrapped.scene)) is not None:
        log.append(f"fingers=({f[0] * 1000:.1f},{f[1] * 1000:.1f})mm")
    piv = _S.get("pivot")
    if piv:
        log.append(f"tether_err={piv.get('err', 0) * 1000:.2f}mm f={piv.get('f', 0):.2f}N "
                   f"turn={piv.get('turn', 0):.2f}deg")
    dv = card[:3] - CARD_C0
    tc = " ".join(f"{k[:5]}=({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})" for k, v in tcps.items())
    print(f"[pin] step {n} {' '.join(log)} card_up={float(dv @ D) * 1000:6.1f}mm card_dy={dv[1] * 1000:5.1f}mm "
          f"{why} score={_S['score']} {tc}", flush=True)


def score(objects):
    return float(_S.get("score", 0.0))


def success(objects):
    return _S.get("score", 0.0) >= 1.0


# --- scripted demo --------------------------------------------------------------------------------------------------
def _wp(t, p, f, label, yaw=0.0):
    return Waypoint(t, tuple(float(v) for v in p), f, label, yaw)


DRAG = 0.040                                            # commanded slide up the board
SLIP_UP = 0.024                                         # how far the notice actually goes (it slips under the pad)
PIN_U = CARD_U0 + CARD_LEN / 2 + SLIP_UP - 0.021        # 21 mm below the notice's top edge once it is up
PIN_YW = CARD_Y - CARD_WID / 2 + 0.015                  # 15 mm in from its right-arm side
U0 = 0.035                                              # the pad presses 3.5 cm above the notice's bottom edge


def _right_plan(s):
    """Take the pushpin from the cup, tilt it square to the board, press it in at ``s`` (a point on the face or the
    notice), ease off, open, retreat."""
    px, py = PIN_XY
    tcp_pin = PIN_Z0 - 0.004 + PAD_TIP_PAST_TCP
    tip_below_tcp = tcp_pin - (PIN_Z0 + NEEDLE_TIP)
    plan = [_wp(2.0, (px, py, tcp_pin + 0.06), FINGER_OPEN, "above pin"),
            _wp(1.5, (px, py, tcp_pin), FINGER_OPEN, "down to pin"),
            _wp(1.0, (px, py, tcp_pin), FINGER_CLOSED, "close on pin"),
            _wp(1.5, (px, py, TOP + 0.16), FINGER_CLOSED, "lift pin high"),
            _wp(2.5, s + N * (tip_below_tcp + 0.06), FINGER_CLOSED, "tilt, over the board", TILT),
            _wp(1.5, s + N * (tip_below_tcp + 0.03), FINGER_CLOSED, "above the spot", TILT),
            _wp(1.5, s + N * (tip_below_tcp + 0.003), FINGER_CLOSED, "tip near the spot", TILT)]
    for k in range(1, 9):                               # 5..40 mm past touching: the arm pushes until the pin seats
        plan.append(_wp(0.8, s + N * (tip_below_tcp - 0.005 * k), FINGER_CLOSED, f"press cmd {5 * k} mm", TILT))
    seat = tip_below_tcp - NEEDLE_LEN + CARD_T           # TCP standoff with the collar on the notice
    plan += [_wp(1.0, s + N * (tip_below_tcp - 0.04), FINGER_CLOSED, "hold press", TILT),
             _wp(1.0, s + N * (seat - 0.003), FINGER_CLOSED, "ease off", TILT),
             _wp(float(os.environ.get("OPEN_S", "2.5")), s + N * (seat - 0.003), FINGER_OPEN, "open", TILT),
             _wp(1.5, s + N * (tip_below_tcp + 0.05), FINGER_OPEN, "retreat", TILT),
             _wp(8.0, s + N * (tip_below_tcp + 0.05), FINGER_OPEN, "wait", TILT)]
    return plan


def _left_plan(drag, release=True):
    """Press the notice to the board with the closed fingers, slide it up by ``drag``, hold it while the pin goes in,
    then lift off (or keep holding)."""
    c = lambda u, out: on_face(u, CARD_Y, CARD_T + out)  # noqa: E731
    plan = [_wp(2.0, c(U0, PAD_TIP_PAST_TCP + 0.06), FINGER_CLOSED, "above notice", TILT90),
            _wp(1.5, c(U0, PAD_TIP_PAST_TCP + 0.01), FINGER_CLOSED, "near notice", TILT90),
            _wp(1.0, c(U0, PAD_TIP_PAST_TCP - PRESS), FINGER_CLOSED, "press notice", TILT90),
            _wp(2.5, c(U0 + drag, PAD_TIP_PAST_TCP - PRESS), FINGER_CLOSED, "slide up", TILT90),
            _wp(16.0, c(U0 + drag, PAD_TIP_PAST_TCP - PRESS), FINGER_CLOSED, "hold notice", TILT90)]
    if release:
        plan += [_wp(1.5, c(U0 + drag, PAD_TIP_PAST_TCP + 0.004), FINGER_CLOSED, "let go", TILT90),
                 _wp(1.5, c(U0 + drag, PAD_TIP_PAST_TCP + 0.05), FINGER_CLOSED, "back off", TILT90),
                 _wp(6.0, c(U0 + drag, PAD_TIP_PAST_TCP + 0.05), FINGER_CLOSED, "watch", TILT90)]
    else:
        plan.append(_wp(7.0, c(U0 + drag, PAD_TIP_PAST_TCP - PRESS), FINGER_CLOSED, "keep holding", TILT90))
    return plan


def demo():
    """Left arm presses the notice to the board and slides it up to the line, then holds it; right arm takes the
    pushpin from the cup, tilts it square to the board and presses it through the notice's top corner into the cork,
    eases off and opens. Both arms retreat. DEMO_VARIANT selects a shortcut: nothing, pin_only (pin into the bare
    cork beside the notice), no_slide (pin the notice where it rests), keep_holding (never let go of the notice)."""
    variant = os.environ.get("DEMO_VARIANT", "full")
    s_note = on_face(PIN_U, PIN_YW, CARD_T)              # on the notice, once it is up
    if variant == "nothing":
        return {"right": [], "left": []}
    if variant == "pin_only":
        return {"right": _right_plan(on_face(PIN_U, PIN_YW - 0.08, 0.0)),
                "left": [_wp(2.0, on_face(U0, CARD_Y, PAD_TIP_PAST_TCP + 0.06), FINGER_CLOSED, "wait", TILT90)]}
    if variant == "no_slide":
        return {"right": _right_plan(s_note - D * SLIP_UP), "left": _left_plan(0.0)}
    if variant == "keep_holding":
        return {"right": _right_plan(s_note), "left": _left_plan(DRAG, release=False)}
    return {"right": _right_plan(s_note), "left": _left_plan(DRAG)}
