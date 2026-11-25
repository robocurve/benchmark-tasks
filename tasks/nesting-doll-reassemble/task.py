"""On a table: a wooden nesting doll set taken apart. The two largest dolls lie open, each as a bottom half standing
open end up and a top half standing on its rim; the third doll stands closed (it holds the two smallest). The robot must
put the set back together: the third doll inside the second, the second closed with its own top and standing inside
the first, and the first closed with its own top over it, each top turned so its painted face lines up with the apron
painted on its bottom half, and let go.

The physical catch: each top only seats on its own bottom, and only when pressed. A top goes over the bottom half's
spigot with 1.5 mm to spare, so it has to come down square and centred; the other doll's top is too big (it slides
down over the whole bottom) or too small (it sits up on the spigot's rim). The last 4 mm are a snug friction fit: a top
let go on its spigot stops 4 mm proud of the shoulder, and only an axial press of more than the fit's friction (4 N for
the first doll, 3 N for the second) slides it home. The order matters too: the second doll goes in before the first
is closed, and it is not carried by its top.

The friction fit, modelled: inside each top's skirt a hidden ring rides on a prismatic joint along the doll's axis.
When the top comes down, the ring lands on the spigot's rim; the top can then slide down over it only against the
joint's drive, a damping drive capped at the fit force, which acts as Coulomb friction (it holds the top's own weight
to under 0.2 mm of creep in a two-minute trial). When a top is lifted clear of its own spigot, the ring goes back to
its rest stop, as wood springs back, so every try at closing a doll needs the press again.

Sizes: a five-piece turned linden set, the largest doll 150 mm tall and 71 mm across (a common size: 5.9 in by 2.8 in),
the second 110 mm by 52 mm and the third 72 mm by 34 mm. Each doll splits at 40 % of its height; the walls are 2.5 mm,
the spigot 2 mm thick and 8 mm tall, the floor 4 mm. Linden at 0.45 g/cm3 gives the masses from the volumes (the third
doll holds two more and is weighed as solid wood). The outline is a classic matryoshka: a flat foot, a full belly, a
narrower waist at the split, shoulders, a neck and a round head.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import meshkit as mk  # noqa: E402
import partkit  # noqa: E402
from partkit import Slider, finger_points  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Put the nesting dolls back together, each closed with its own top and its face lined up, "
               "the smaller ones inside the largest.")

LINDEN = 450.0                            # kg/m3
RED, CREAM, GOLD, BLUE = (0.72, 0.12, 0.10), (0.95, 0.86, 0.72), (0.86, 0.66, 0.20), (0.18, 0.30, 0.62)

# outline of a doll as (r / R, z / H), from the foot to the top of the head
OUTLINE = [(0.78, 0.0), (0.90, 0.03), (0.98, 0.12), (1.00, 0.25), (0.98, 0.36), (0.965, 0.40), (0.93, 0.48),
           (0.82, 0.58), (0.70, 0.66), (0.66, 0.70), (0.69, 0.76), (0.70, 0.82), (0.64, 0.90), (0.48, 0.96),
           (0.25, 0.99), (0.0, 1.0)]
SPLIT = 0.40                              # the halves part at 40 % of the height (OUTLINE's 6th point)
WALL, SPIGOT_T, SPIGOT_H, FLOOR, FIT = 0.0025, 0.0020, 0.008, 0.004, 0.0015
FIT_TRAVEL = 0.004                        # the last 4 mm of a top's travel are the snug friction fit
FIT_FORCE = {1: 4.0, 2: 3.0}              # N: the axial press that slides a top over that last 4 mm
FIT_DAMPING = 1.0e5                       # N s/m below the cap: the top's own weight creeps it under 0.2 mm in 2 min
RING_T, RING_MASS = 0.0012, 0.002         # the hidden ring: 1.2 mm thick, 2 g
LEAD_IN = 0.001                           # the skirt's inner edge is eased 1 mm at 45 degrees, as on a turned set
SEGMENTS = 48
SIZES = {1: (0.150, 0.0355), 2: (0.110, 0.0260), 3: (0.072, 0.0170)}   # height, belly radius


def outer(n: int, z: float) -> float:
    """The doll's outer radius at height ``z`` (doll frame, foot at 0)."""
    H, R = SIZES[n]
    pts = [(r * R, f * H) for r, f in OUTLINE]
    for (r0, z0), (r1, z1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            return r0 + (z - z0) / (z1 - z0) * (r1 - r0) if z1 > z0 else max(r0, r1)
    return 0.0


def dims(n: int) -> dict:
    H, R = SIZES[n]
    zs = SPLIT * H
    rs = outer(n, zs)
    top_in = rs - WALL                    # the top's skirt, inside
    sp_out = top_in - FIT                 # the bottom's spigot, outside and inside
    sp_in = sp_out - SPIGOT_T
    return {"H": H, "R": R, "zs": zs, "rs": rs, "top_in": top_in, "sp_out": sp_out, "sp_in": sp_in}


def bottom_profile(n: int):
    """Closed (r, z) polygon of a bottom half (its own frame: foot at z = 0)."""
    H, R = SIZES[n]
    d = dims(n)
    pts = [(r * R, f * H) for r, f in OUTLINE if f * H <= d["zs"] + 1e-9]
    prof = [(0.0, 0.0)] + pts + [(d["sp_out"], d["zs"]), (d["sp_out"], d["zs"] + SPIGOT_H),
                                 (d["sp_in"], d["zs"] + SPIGOT_H)]
    # the cavity: the spigot's bore down to the shoulder, then the outline less the wall (never wider than the bore)
    inner = [(min(d["sp_in"], r - WALL), z) for r, z in reversed(pts) if z > FLOOR + 1e-9]
    inner.append((min(d["sp_in"], outer(n, FLOOR) - WALL), FLOOR))
    prof += inner + [(0.0, FLOOR)]
    return prof


def top_profile(n: int):
    """Closed (r, z) polygon of a top half (its own frame: the rim of its skirt at z = 0)."""
    H, R = SIZES[n]
    d = dims(n)
    pts = [(r * R, f * H - d["zs"]) for r, f in OUTLINE if f * H >= d["zs"] - 1e-9]
    inner = []
    for r, z in reversed(pts):
        if r - WALL > 0.002 and z < pts[-1][1] - WALL:
            inner.append((r - WALL, z))
    r_rim, _ = inner.pop()                  # the skirt's inner edge at the rim, eased LEAD_IN at 45 degrees
    inner += [(r_rim, LEAD_IN), (r_rim + LEAD_IN, 0.0)]
    return pts + [(0.0, pts[-1][1] - WALL)] + inner


def solid_profile(n: int):
    H, R = SIZES[n]
    return [(0.0, 0.0)] + [(r * R, f * H) for r, f in OUTLINE]


def _paint_band(n: int, z0: float, z1: float, color, half_deg: float, inset: float = 0.0006):
    """A painted panel on the front (+x) of a half: a thin shell 0.6 mm proud of the outline, ``half_deg`` either side
    of +x, from z0 to z1 (doll frame). Visual only."""
    zs = np.linspace(z0, z1, 6)
    prof = [(outer(n, z) + inset, z) for z in zs] + [(outer(n, z) - 0.0005, z) for z in zs[::-1]]
    return mk.Shape(mk.revolve(prof, segments=12, angle=2 * half_deg, start=-half_deg), color=color, collider="none",
                    density=0.0)


def _mass(mesh) -> float:
    return float(mesh.volume()) * LINDEN


BOUNDS: dict = {}                         # every piece's shape bounds in its own frame (for the release check)


def bottom_part(n: int, x: float, y: float, yaw: float):
    d = dims(n)
    body = mk.revolve(bottom_profile(n), segments=SEGMENTS)
    apron = _paint_band(n, 0.10 * d["H"], d["zs"] - 0.004, CREAM, 35.0)
    sh = [mk.Shape(body, color=RED, sdf_resolution=160), apron]
    BOUNDS[f"doll{n}_bottom"] = tuple(tuple(v.tolist()) for v in body.bounds())
    return mk.mesh_part(f"doll{n}_bottom", pos=(float(x), float(y), float(TOP + 0.002)), yaw=float(yaw),
                        mass=round(_mass(body), 4), shapes=sh, friction=0.5)


def top_part(n: int, x: float, y: float, yaw: float):
    """A top half as a partkit part: the lathed shell (and its paint) on the body, and the hidden friction-fit ring
    on a slider along the doll's axis. The ring rests 4 mm below the spigot's height in the top's frame, so a top let
    go on its spigot stands 4 mm proud; pressing it home slides the top 4 mm down over the ring, against the cap."""
    d = dims(n)
    body = mk.revolve(top_profile(n), segments=SEGMENTS)
    zf0, zf1 = 0.70 * d["H"] - d["zs"], 0.90 * d["H"] - d["zs"]
    face = mk.Shape(mk.revolve([(0.0006 + max(outer(n, z + d["zs"]), 0.003), z) for z in np.linspace(zf0, zf1, 6)]
                               + [(outer(n, z + d["zs"]) - 0.0005, z) for z in np.linspace(zf1, zf0, 6)],
                               segments=12, angle=70.0, start=-35.0), color=CREAM, collider="none", density=0.0)
    shawl = mk.Shape(mk.revolve([(outer(n, z + d["zs"]) + 0.0006, z) for z in np.linspace(0.004, zf0 - 0.004, 5)]
                                + [(outer(n, z + d["zs"]) - 0.0005, z) for z in np.linspace(zf0 - 0.004, 0.004, 5)],
                                segments=12, angle=50.0, start=-25.0), color=GOLD, collider="none", density=0.0)
    z_ring = SPIGOT_H - FIT_TRAVEL
    ring = mk.revolve_pieces([[(d["sp_in"] + 0.0005, z_ring), (d["sp_out"] + 0.001, z_ring),
                               (d["sp_out"] + 0.001, z_ring + RING_T), (d["sp_in"] + 0.0005, z_ring + RING_T)]],
                             segments=16)
    slider = Slider(f"fit{n}", rest=(0.0, 0.0, z_ring), travel=FIT_TRAVEL + 0.001, sign=1, rate=0.0,
                    damping=FIT_DAMPING, effort=FIT_FORCE[n], mass=RING_MASS)
    # partkit needs one box on the body: a 2 mm collider-only cube inside the head, 4 mm under the ceiling, clear of
    # every doll that can be inside (28 mm or more)
    anchor = ((0.0, 0.0, d["H"] - d["zs"] - WALL - 0.005), (0.002, 0.002, 0.002), None)
    spec = partkit.part(f"doll{n}_top", pos=(float(x), float(y), float(TOP + 0.002)), yaw=float(yaw),
                        mass=round(_mass(body) - RING_MASS, 4), boxes=[anchor], sliders=[slider], friction=0.5,
                        color=RED)
    mk.attach(spec, [mk.Shape(body, color=RED, sdf_resolution=160), face, shawl])
    mk.attach(spec, mk.hulls(ring), to=f"fit{n}")
    lo, hi = body.bounds()
    BOUNDS[spec.name] = (tuple(lo.tolist()), tuple(hi.tolist()))
    return spec


def closed_part(n: int, x: float, y: float, yaw: float):
    H, _ = SIZES[n]
    body = mk.revolve(solid_profile(n), segments=SEGMENTS)
    BOUNDS[f"doll{n}"] = tuple(tuple(v.tolist()) for v in body.bounds())
    sh = [mk.Shape(body, color=BLUE), _paint_band(n, 0.70 * H, 0.90 * H, CREAM, 35.0),
          _paint_band(n, 0.10 * H, 0.36 * H, CREAM, 35.0)]
    return mk.mesh_part(f"doll{n}", pos=(float(x), float(y), float(TOP + 0.002)), yaw=float(yaw),
                        mass=round(_mass(body), 4), shapes=sh, friction=0.5)


# -- layout: faces turned every which way ------------------------------------------------------------------------------
LAYOUT = {"doll1_bottom": (0.22, 0.04, math.radians(180.0)),
          "doll1_top": (0.30, 0.22, math.radians(90.0)),
          "doll2_bottom": (0.22, -0.18, math.radians(150.0)),
          "doll2_top": (0.40, -0.20, math.radians(-120.0)),
          "doll3": (0.18, -0.33, math.radians(30.0))}
OBJECTS = [bottom_part(1, *LAYOUT["doll1_bottom"]), top_part(1, *LAYOUT["doll1_top"]),
           bottom_part(2, *LAYOUT["doll2_bottom"]), top_part(2, *LAYOUT["doll2_top"]),
           closed_part(3, *LAYOUT["doll3"])]
SPECS = {o.name: o for o in OBJECTS}

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_XY_MM, IN_Z_MM, IN_TILT = 3.0, 3.0, 8.0        # a doll inside a bottom half: down on its floor, on its axis
SEAT_XY_MM, SEAT_Z_MM, SEAT_TILT = 2.5, 1.0, 5.0  # a top pressed home on its bottom: rim on the shoulder, square
FACE_DEG = 15.0                                   # face over apron
UPRIGHT_DEG = 5.0
RELEASE = 0.02
STILL_STEPS, STILL_MM = 21, 1.0
_STATE: dict = {}


@mk.on_reset
def _reset_state(environment=None) -> None:
    _STATE.clear()
    _STATE.update(step=0, free=True, violation="", hist=[], updated_at=None, last=(0.0, False))


_reset_state()


def tilt_deg(st) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(st.rotation()[2, 2])))))


def yaw_of(st) -> float:
    R = st.rotation()
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def _ang(a: float) -> float:
    return abs((a + math.pi) % (2 * math.pi) - math.pi)


def inside(objects, inner: str, outer_bottom: str) -> bool:
    """``inner`` (a closed doll or a bottom half) stands on ``outer_bottom``'s floor, on its axis."""
    a, b = objects[inner], objects[outer_bottom]
    loc = b.to_local(a.pos)
    return (math.hypot(float(loc[0]), float(loc[1])) * 1000 <= IN_XY_MM and abs(float(loc[2]) - FLOOR) * 1000
            <= IN_Z_MM and tilt_deg(a) <= IN_TILT and tilt_deg(b) <= IN_TILT)


def seated(objects, n: int) -> bool:
    """Doll n's top is pressed home on its own bottom: its rim on the shoulder (1 mm; a top only let go on its spigot
    stands 4 mm proud), centred (2.5 mm), square (5 degrees) and its face over the apron (15 degrees)."""
    t, b = objects[f"doll{n}_top"], objects[f"doll{n}_bottom"]
    loc = b.to_local(t.pos)
    return (math.hypot(float(loc[0]), float(loc[1])) * 1000 <= SEAT_XY_MM
            and abs(float(loc[2]) - dims(n)["zs"]) * 1000 <= SEAT_Z_MM
            and tilt_deg(t) <= SEAT_TILT and tilt_deg(b) <= SEAT_TILT
            and math.degrees(_ang(yaw_of(t) - yaw_of(b))) <= FACE_DEG)


def levels(objects) -> dict:
    three_in_two = inside(objects, "doll3", "doll2_bottom")
    two_closed = three_in_two and seated(objects, 2)
    two_in_one = two_closed and inside(objects, "doll2_bottom", "doll1_bottom")
    one_closed = (two_in_one and seated(objects, 1) and tilt_deg(objects["doll1_bottom"]) <= UPRIGHT_DEG)
    return {"three_in_two": three_in_two, "two_closed": two_closed, "two_in_one": two_in_one,
            "one_closed": one_closed}


def _near(pt, st, name) -> bool:
    lo, hi = BOUNDS[name]
    lo, hi = np.asarray(lo, float) - RELEASE, np.asarray(hi, float) + RELEASE
    loc = st.to_local(pt)
    if not (lo[2] <= loc[2] <= hi[2]):
        return False
    return math.hypot(float(loc[0]), float(loc[1])) <= max(abs(hi[0]), abs(lo[0])) + 0.0


def _state(environment, name: str):
    from meshkit import _mesh_state   # noqa: PLC0415

    spec = SPECS[name]
    return partkit._part_state(environment, spec) if isinstance(spec, partkit.Part) else _mesh_state(environment, spec)


def engaged(top, bottom, n: int) -> bool:
    """Doll n's top is over its own bottom's spigot: centred within the 1.5 mm fit and its rim below the spigot top
    plus 3 mm."""
    loc = bottom.to_local(top.pos)
    return (math.hypot(float(loc[0]), float(loc[1])) <= FIT + 0.001
            and float(loc[2]) <= dims(n)["zs"] + SPIGOT_H + 0.003)


def _release_fit(environment, states) -> None:
    """A top lifted clear of its own spigot: its fit ring back to the rest stop (wood springs back)."""
    import torch   # noqa: PLC0415

    for n in FIT_FORCE:
        top = states[f"doll{n}_top"]
        if top.sliders.get(f"fit{n}", 0.0) > 0.0003 and not engaged(top, states[f"doll{n}_bottom"], n):
            asset = environment.unwrapped.scene[f"doll{n}_top"]
            j = list(asset.joint_names).index(f"fit{n}_slide")
            zero = torch.zeros((1, 1), device=partkit._t(asset.data.joint_pos).device)
            asset.write_joint_position_to_sim_index(position=zero, joint_ids=[j], env_ids=[0])


@mk.on_step
def _step(environment) -> None:
    _STATE["step"] += 1
    pts = finger_points(environment)
    states = {n: _state(environment, n) for n in SPECS}
    _STATE["free"] = not any(_near(p, states[n], n) for n in SPECS for p in pts)
    _STATE["fit_mm"] = {n: round(1000 * states[f"doll{n}_top"].sliders.get(f"fit{n}", 0.0), 2) for n in FIT_FORCE}
    _release_fit(environment, states)


def _update(objects) -> tuple[float, bool]:
    for n in SPECS:
        if float(objects[n].pos[2]) < TOP - 0.05 and not _STATE["violation"]:
            _STATE["violation"] = f"{n} fell off the table"
    hist = _STATE["hist"]
    hist.append((np.array([np.asarray(objects[n].pos, float) for n in SPECS]), _STATE["free"]))
    del hist[:-STILL_STEPS]
    if _STATE["violation"] or not _STATE["free"]:
        return 0.0, False
    lv = levels(objects)
    if lv["one_closed"]:
        still = (len(hist) == STILL_STEPS and all(f for _, f in hist)
                 and max(float(np.linalg.norm(h - hist[-1][0], axis=1).max()) for h, _ in hist) <= STILL_MM / 1000)
        return (1.0, True) if still else (0.75, False)
    if lv["two_in_one"]:
        return 0.75, False
    if lv["two_closed"]:
        return 0.5, False
    if lv["three_in_two"]:
        return 0.25, False
    return 0.0, False


def _once(objects) -> tuple[float, bool]:
    """``_update`` once per control step (success() and score() are both called each step)."""
    if _STATE.get("updated_at") != _STATE["step"]:
        _STATE["updated_at"], _STATE["last"] = _STATE["step"], _update(objects)
    return _STATE["last"]


def success(objects) -> bool:
    return _once(objects)[1]


def score(objects) -> float:
    return _once(objects)[0]


def distance(objects) -> float:
    a, b = objects["doll1_top"], objects["doll1_bottom"]
    return float(np.linalg.norm(np.asarray(a.pos) - np.asarray(b.pos) - np.array([0.0, 0.0, dims(1)["zs"]])))


def demo():
    """The scripted demonstration is in demo.py."""
    return []
