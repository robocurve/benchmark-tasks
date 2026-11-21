"""Take one tortilla out of a resealable bag and lay it on the plate. See README.md and DESIGN.md."""

import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bodykit  # noqa: E402
import clothkit  # noqa: E402
import gripkit  # noqa: E402
import layout as L  # noqa: E402
import scoring  # noqa: E402
import zipkit  # noqa: E402
from bodykit import Body, BoxShape, CylShape, plate  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
bodykit.allow_matrix_grips()

# -- rigid bodies ----------------------------------------------------------------------------------------------------
BAMBOO = (0.80, 0.64, 0.42)  # end-grain maple
BAMBOO_DARK = (0.72, 0.55, 0.34)
CERAMIC = (0.93, 0.93, 0.91)
bodykit.FINISH[CERAMIC] = (0.18, 0.0)


def _board() -> Body:
    sx, sy, sz = L.BOARD_SIZE
    shapes = [BoxShape((0, 0, 0), (sx, sy, sz), BAMBOO)]
    # the strips of a bamboo board (visual only), and a darker edge band
    n = 9
    for i in range(n):
        if i % 2:
            x = -sx / 2 + (i + 0.5) * sx / n
            shapes.append(BoxShape((x, 0, sz / 2 + 0.0001), (sx / n, sy - 0.002, 0.0002), BAMBOO_DARK, collide=False))
    return Body.build("board", pos=tuple(L.BOARD_AT), mass=2.6, shapes=shapes, friction=(0.55, 0.5))


def _plate(segments: int = 40) -> Body:
    shapes = [CylShape((0, 0, L.PLATE_WELL_Z / 2), L.PLATE_WELL_R, L.PLATE_WELL_Z, CERAMIC)]
    t = 0.004
    for i in range(segments):
        a = 2 * math.pi * (i + 0.5) / segments
        c, s = math.cos(a), math.sin(a)
        width = 2 * math.tan(math.pi / segments) * L.PLATE_R + 0.002
        lo = np.array([c * (L.PLATE_WELL_R - 0.004), s * (L.PLATE_WELL_R - 0.004), L.PLATE_WELL_Z - t / 2])
        hi = np.array([c * L.PLATE_R, s * L.PLATE_R, L.PLATE_H - t / 2])
        n = np.array([-c * (L.PLATE_H - L.PLATE_WELL_Z), -s * (L.PLATE_H - L.PLATE_WELL_Z),
                      L.PLATE_R - L.PLATE_WELL_R])
        shapes.append(plate(lo, hi, width, t, n / np.linalg.norm(n), CERAMIC))
    return Body.build("plate", pos=tuple(L.PLATE_AT), mass=L.PLATE_MASS, shapes=shapes, friction=(0.5, 0.45),
                      solver_iterations=16)


BOARD = _board()
PLATE = _plate()
OBJECTS = [BOARD, PLATE]

# -- the bag's films and the tortillas (PhysX surface deformables) ---------------------------------------------------
TEX = HERE / "textures"
START = HERE / "start.npz"
_start = np.load(START) if START.is_file() and not os.environ.get("TORT_FRESH") else None


def _st(name):
    return None if _start is None or name not in _start else _start[name]


FILM = dict(mass=L.FILM_MASS, thickness=L.FILM_T, stretch=1.0e4, shear=1.0e3, bend=4.0e4,
            friction=(0.35, 0.3), iterations=32, cast_shadows=False, base_look={"glass": (0.97, 0.98, 1.0), "frost": 0.04})
ZIP_LOOK = {"color": (0.55, 0.72, 0.88), "roughness": 0.35}
SEAL_LOOK = {"glass": (0.97, 0.98, 1.0), "frost": 0.35}


def _film_looks(which):
    parts = L.face_parts(which)
    looks = [(parts["seal"], SEAL_LOOK), (parts["zip"], ZIP_LOOK)]
    if "label" in parts:
        looks.append((parts["label"], {"texture": str(TEX / "film_top.png"), "roughness": 0.3}))
    # a face belongs to one subset only: the later parts win
    seen = set()
    out = []
    for faces, look in reversed(looks):
        faces = [int(x) for x in faces if int(x) not in seen]
        seen.update(faces)
        out.append((faces, look))
    return list(reversed(out))
ft, ff, fuv, finfo = L.film("top")
fb, fbf, fbuv, binfo = L.film("bottom")
FILM_LOOK = "alpha"
if FILM_LOOK == "alpha":
    FILM.pop("base_look")
_fl = (lambda w: dict(texture=str(TEX / f"film_{w}.png"), alpha_opacity=True, roughness=0.15)) if FILM_LOOK == "alpha" \
    else (lambda w: dict(looks=_film_looks(w)))
clothkit.sheet("film_top", host="board", points=ft, faces=ff, uv=fuv, **_fl("top"),
               rest_bend="restShapeDefault", start=_st("film_top"), **FILM)
clothkit.sheet("film_bottom", host="board", points=fb, faces=fbf, uv=fbuv, **_fl("bottom"),
               start=_st("film_bottom"), **FILM)
TORTILLAS = [f"tortilla{k}" for k in range(L.N_TORTILLAS)]
_tfaces = []
for k, name in enumerate(TORTILLAS):
    p, f, uv = L.tortilla(k)
    _tfaces.append(f)
    clothkit.sheet(name, host="board", points=p, faces=f, uv=uv, texture=str(TEX / f"tortilla{k}.png"),
                   mass=L.T_MASS, thickness=L.T_TH, stretch=1.0e4, shear=1.0e3,
                   bend=1.0e5, friction=(0.3, 0.28), roughness=0.75,
                   iterations=32, start=_st(name))
SHEETS = ["film_top", "film_bottom", *TORTILLAS]
gripkit.SLICK.update({"film_top", "film_bottom"})

_seams = L.seam_pairs()
clothkit.link("seams", "film_top", tuple(_seams[:, 0]), "film_bottom", tuple(_seams[:, 1]))
ZIP_FORCE = 0.15  # N to pull one 6.4 mm cell of the zip apart
ZIP = L.zip_pairs()
ZIPPER = zipkit.zip_("zip", host="board", top_sheet="film_top", bottom_sheet="film_bottom",
                     cells=[(vt, vb, (vt - 1, vt + 1), (vb - 1, vb + 1)) for vt, vb, _ in ZIP],
                     break_force=ZIP_FORCE, break_torque=ZIP_FORCE * 0.004)
ZIP_BOTTOM = np.array([vb for _, vb, _ in ZIP])

# -- per-episode state -----------------------------------------------------------------------------------------------
_ep = {"step": 0, "trace": None, "metrics": {}}


@bodykit.on_reset
def reset():
    _ep["step"] = 0
    _ep["metrics"] = {}
    if _ep["trace"] is not None:
        _ep["trace"].close()
    path = os.environ.get("TORT_TRACE")
    _ep["trace"] = open(path, "w") if path else None  # noqa: SIM115 -- closed at the next reset
    gripkit.reset()


def _open_cells():
    return [ZIP[i][2] for i in zipkit.open_cells("zip")]


@bodykit.on_step
def observe(env):
    _ep["step"] += 1
    k = _ep["step"]
    gripkit.update(env, k, SHEETS)
    states = {arm: gripkit.arm_state(env, arm) for arm in ("left", "right")}
    gaps = {arm: st[2] for arm, st in states.items()}
    plate_state = bodykit._body_state(env, PLATE)
    speeds = [float(np.linalg.norm(clothkit.velocities(n), axis=1).max()) for n in TORTILLAS]
    m = scoring.evaluate([clothkit.points(n) for n in TORTILLAS], _tfaces, speeds, clothkit.points("film_bottom"),
                         ZIP_BOTTOM, _open_cells(), plate_state.pos, hands_open=min(gaps.values()) > 0.020,
                         hands_empty=not gripkit.held(), tips=[st[0] + st[1][:, 2] * 0.0128 for st in states.values()])
    _ep["metrics"] = m
    save = os.environ.get("TORT_SAVE")
    if save and k == int(os.environ.get("TORT_SAVE_AT", 40)):
        np.savez(save, **{n: clothkit.points(n) for n in SHEETS}, **{f"{a}_tcp": st[0] for a, st in states.items()},
                 **{f"{a}_rot": st[1] for a, st in states.items()}, **{f"{a}_gap": st[2] for a, st in states.items()})
        print(f"task: saved the sheets at step {k} to {save}", flush=True)
    tr = _ep["trace"]
    if tr is not None and (k % 5 == 0 or gripkit.log()):
        row = {"step": k, "phase": os.environ.get("TORT_PHASE", ""), "cmd_tip": json.loads(os.environ.get("TORT_CMD", "{}")), "metrics": m, "gaps": {a: round(g * 1000, 1) for a, g in gaps.items()},
               "held": {":".join(key): n for key, n in gripkit.held().items()}, "events": gripkit.log(),
               "tortilla_centres": [clothkit.points(n).mean(axis=0).round(4).tolist() for n in TORTILLAS],
               "plate": plate_state.pos.round(4).tolist()}
        top, bot = clothkit.points("film_top"), clothkit.points("film_bottom")
        b0 = clothkit.get("film_bottom").start if clothkit.get("film_bottom").start is not None else clothkit.get("film_bottom").points
        bnow = clothkit.points("film_bottom")
        row["bag_shift_mm"] = (1000 * (bnow[:, :2].mean(axis=0) - b0[:, :2].mean(axis=0))).round(1).tolist()
        d0, d1 = b0[-1, :2] - b0[0, :2], bnow[-1, :2] - bnow[0, :2]
        row["bag_yaw_deg"] = round(float(np.degrees(np.arctan2(d1[1], d1[0]) - np.arctan2(d0[1], d0[0]))), 1)
        row["zip_sep_mm"] = [round(float(v) * 1000, 1) for v in zipkit.separations("zip")[::4]]
        row["zip_bottom_z_mm"] = round((bot[ZIP_BOTTOM, 2].max() - L.BOARD_TOP) * 1000, 1)
        row["zip_top_z_mm"] = round((top[[vt for vt, _, _ in ZIP], 2].max() - L.BOARD_TOP) * 1000, 1)
        for arm in ("left", "right"):
            tcp, rot = gripkit.arm_state(env, arm)[:2]
            row["tcp_" + arm] = tcp.round(4).tolist()
            row["tip_" + arm] = (tcp + rot[:, 2] * 0.0128).round(4).tolist()
        tr.write(json.dumps(row) + "\n")
        tr.flush()
        gripkit._LOG.clear()


def score(objects):
    return _ep["metrics"].get("score", 0.0)


def success(objects):
    return score(objects) == 1.0


def demo():
    """Hold still (the scripted demo is run_demo.py's policy, which plays plan.plan() in joint space)."""
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    import fastik

    out = {}
    for arm in ("left", "right"):
        pos, rot = fastik.fk(L.READY, L.BASES[arm])
        out[arm] = [Waypoint(float(os.environ.get("HOLD_S", 60.0)), tuple(pos), FINGER_OPEN, "hold", rot)]
    return out
