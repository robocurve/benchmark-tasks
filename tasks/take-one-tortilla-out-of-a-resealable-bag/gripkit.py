"""gripkit: what the YAM's rubber finger pads hold by friction, on clothkit sheets.

PhysX's surface-deformable contacts are position based: friction from a rigid pad on a sheet scales with the mass of
the few sheet vertices the pad touches (a gram or less), so a pad cannot hold a 50 g tortilla, nor a film pulled
against a zip, the way rubber does. Two rules stand in for that friction. Each is checked after every control step,
for each arm, from the arm's joint positions:

- Pinch. Both pads close on a sheet: the gap between them is no more than the thickness of the sheets that lie
  between them plus 1.5 mm. The sheet vertices between the pads (within the pads' 15 mm width and 30 mm length) are
  then fixed to the finger where they are, until the pads open 3 mm past that gap.
- Press. The fingers are closed together and their tips press into a sheet: sheet vertices under the tips' 8 x 15 mm
  end face, no more than half the sheet's thickness plus 0.4 mm from the face, are fixed to the finger where they
  are, once they have pressed for two control steps, until the tips lift 3 mm or the fingers open. Only the sheet nearest the tips (the top one) is held, within the pad's footprint (10 mm around the contact).

Nothing is held unless the pads physically reach the sheet: the arm has to put the pads there and close them, or
press the tips down on it, as on the real rig.
"""

from __future__ import annotations

import numpy as np

import bodykit
import clothkit
import fastik

BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
PAD_HALF_W = 0.0077  # pad half width (TCP y)
PAD_Z = (-0.017, 0.0128)  # the pads' extent along the approach (TCP z), tip at +12.8 mm
TIP_HALF_X = 0.0045  # closed fingers: the tip face spans this either side of the TCP (TCP x)
PINCH_SLACK = 0.0015
PINCH_LET_GO = 0.003
CROSS_BAND = 0.006  # m: a sheet crossing the pads' mid-plane (a mesh edge from one side to the other within the pad area) while they close is pinched once they are this much more than its thickness apart
PRESS_TOL = 0.0004
PRESS_LIFT = 0.003
SLICK: set = set()  # sheets (smooth film) that a press through onto another sheet does not hold
CLOSED_GAP = 0.0010  # fingers closed together (nothing between them)

_TIP_PATH: dict[str, str] = {}
_STATE: dict = {}
_LOG: list = []


def _tip_path(arm: str) -> str:
    if arm not in _TIP_PATH:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        side = "LeftArm" if arm == "left" else "RightArm"
        for prim in stage.Traverse():
            p = prim.GetPath().pathString
            if p.endswith("/tip_left") and f"/{side}/" in p:
                _TIP_PATH[arm] = p
                break
    return _TIP_PATH[arm]


def arm_state(env, arm: str):
    """(TCP position, TCP rotation, finger gap, tip_left position, tip_left quat xyzw), env frame."""
    robot = env.unwrapped.scene[arm + "_robot"]
    jp = bodykit.as_numpy(robot.data.joint_pos)[0]
    fids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
    aids, _ = robot.find_joints([f"joint{i}" for i in range(1, 7)], preserve_order=True)
    gap = float(-2.0 * jp[fids].mean())
    tcp, rot = fastik.fk(jp[aids], BASES[arm])
    bids, _ = robot.find_bodies(["tip_left"], preserve_order=True)
    origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
    bp = bodykit.as_numpy(robot.data.body_pos_w)[0][bids[0]] - origin
    bq = bodykit.as_numpy(robot.data.body_quat_w)[0][bids[0]]
    return tcp, rot, gap, bp, bq


def reset() -> None:
    _STATE.clear()
    _LOG.clear()


def log() -> list:
    return list(_LOG)


def held(arm: str | None = None) -> dict:
    """{(arm, kind, sheet): number of vertices} held now."""
    out = {}
    for key, (sheet, verts) in clothkit.holds().items():
        a, kind, s = key.split(":")
        if arm is None or a == arm:
            out[(a, kind, s)] = len(verts)
    return out


def update(env, step: int, sheets: list[str]) -> None:
    for arm in ("left", "right"):
        tcp, rot, gap, bp, bq = arm_state(env, arm)
        local = {s: (clothkit.points(s) - tcp) @ rot for s in sheets}
        _pinch(arm, gap, local, sheets, bp, bq, step)
        _press(arm, gap, local, sheets, bp, bq, step, tip=tcp + rot[:, 2] * PAD_Z[1], approach=rot[:, 2])


_EDGES: dict[str, np.ndarray] = {}


def _edges(sheet: str) -> np.ndarray:
    if sheet not in _EDGES:
        f = np.asarray(clothkit.get(sheet).faces).reshape(-1, 3)
        e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        _EDGES[sheet] = np.unique(e, axis=0)
    return _EDGES[sheet]


def _pinch(arm, gap, local, sheets, bp, bq, step):
    st = _STATE.setdefault((arm, "pinch"), {"gap": None, "prev": 1.0})
    prev, st["prev"] = st["prev"], gap
    if st["gap"] is not None:  # holding: let go once the pads open
        if gap > st["gap"] + PINCH_LET_GO:
            for s in sheets:
                clothkit.release(f"{arm}:pinch:{s}")
            _LOG.append((step, arm, "pinch released", gap))
            st["gap"] = None
        return
    if gap > 0.020 or prev < 0.002:  # a pinch is made as the pads close on a sheet, not by closed fingers
        return
    between, crossing = {}, {}
    for s in sheets:
        q = local[s]
        foot = (np.abs(q[:, 1]) <= PAD_HALF_W) & (q[:, 2] >= PAD_Z[0]) & (q[:, 2] <= PAD_Z[1] - 0.0015)
        m = foot & (np.abs(q[:, 0]) <= gap / 2 + 0.0004)
        if m.any():
            between[s] = np.where(m)[0]
        # a sheet running across the gap at a slant passes between the pads with its vertices (a mesh-spacing
        # apart) on either side of them: a mesh edge crossing the pads' mid-plane within the pad area
        e = _edges(s)
        xa, xb = q[e[:, 0], 0], q[e[:, 1], 0]
        k = (np.sign(xa) != np.sign(xb)) & (np.abs(xa) <= CROSS_BAND) & (np.abs(xb) <= CROSS_BAND)
        if k.any():
            t = (xa[k] / (xa[k] - xb[k]))[:, None]
            c = q[e[k, 0]] + t * (q[e[k, 1]] - q[e[k, 0]])
            inside = (np.abs(c[:, 1]) <= PAD_HALF_W) & (c[:, 2] >= PAD_Z[0]) & (c[:, 2] <= PAD_Z[1])
            if inside.any():
                crossing[s] = np.unique(e[k][inside])
    if between:
        thick = sum(clothkit.get(s).thickness for s in between)
        if gap > thick + PINCH_SLACK:
            return
    elif crossing and gap < prev - 0.0001:  # closing on a slanted sheet: it is trapped once the pads are this near
        between = crossing
        if gap > sum(clothkit.get(s).thickness for s in between) + CROSS_BAND:
            return
    else:
        return
    for s, v in between.items():
        clothkit.hold(f"{arm}:pinch:{s}", s, v, _tip_path(arm), bp, bq)
    st["gap"] = gap
    _LOG.append((step, arm, "pinch " + ",".join(f"{s}:{len(v)}" for s, v in between.items()), gap))


PRESS_PATCH = 0.010  # a press holds the pressed sheet within this radius of the contact (the pad's footprint)


def _press(arm, gap, local, sheets, bp, bq, step, tip=None, approach=None):
    """Fingers closed, tips pressing a sheet: hold the one sheet nearest the tip face (the top one) under the tips,
    once the tips have pressed it for two control steps, until they draw back 3 mm along the fingers or open."""
    pinching = (arm, "pinch") in _STATE and _STATE[(arm, "pinch")]["gap"] is not None
    st = _STATE.setdefault((arm, "press"), {"on": None, "seen": 0})
    if st["on"] is not None:
        lift = float(tip[2] - st["tip"][2])  # sheets here lie flat: lifting off is moving up
        if gap > CLOSED_GAP + 0.002 or lift > PRESS_LIFT:
            clothkit.release(f"{arm}:press:{st['on']}")
            _LOG.append((step, arm, f"press released {st['on']}", gap))
            st.update(on=None, seen=0)
        return
    if gap > CLOSED_GAP or pinching:
        st["seen"] = 0
        return
    best = None
    for s in sheets:
        q = local[s]
        th = clothkit.get(s).thickness
        m = ((np.abs(q[:, 0]) <= TIP_HALF_X) & (np.abs(q[:, 1]) <= PAD_HALF_W)
             & (q[:, 2] >= PAD_Z[1] - 0.003) & (q[:, 2] <= PAD_Z[1] + th / 2 + PRESS_TOL))
        if m.any():
            d = float(np.min(q[m, 2]))
            if best is None or d < best[0]:
                best = (d, s, m)
    if best is None:
        st["seen"] = 0
        return
    _, s0, m0 = best
    if s0 in SLICK:  # clear film lying on a tortilla slides over it: pressing through it holds neither
        q0 = local[s0][m0]
        for s in sheets:
            if s in SLICK:
                continue
            q = local[s]
            under = ((np.abs(q[:, 0]) <= TIP_HALF_X + 0.003) & (np.abs(q[:, 1]) <= PAD_HALF_W + 0.003)
                     & (q[:, 2] >= q0[:, 2].min()) & (q[:, 2] <= q0[:, 2].max() + 0.005))
            if under.any():
                st["seen"] = 0
                return
    st["seen"] += 1
    if st["seen"] < 2:
        return
    _, s, m = best
    q = local[s]
    c = q[m].mean(axis=0)
    patch = np.where((np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) <= PRESS_PATCH)
                     & (np.abs(q[:, 2] - c[2]) <= 0.004))[0]
    clothkit.hold(f"{arm}:press:{s}", s, patch, _tip_path(arm), bp, bq)
    st.update(on=s, tip=tip.copy(), approach=approach.copy())
    _LOG.append((step, arm, f"press {s}:{len(patch)}", gap))
