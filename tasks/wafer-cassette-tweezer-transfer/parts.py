"""USD parts for the 2-inch cassette-to-cassette wafer transfer, at real size (metres, kilograms). Pure pxr, no simulator:
`python parts.py` writes assets/*.usda (pxr must not be imported before the simulator starts).

Sources (also in README.md, "Parts"):
  cassette  Chung King "2" PP Cassette", PP-C050 (ckplas.com): 142 (L) x 67.4 (W) x 61.5 (H) mm, 25 wafers, slot pitch
            4.76 mm, 82 g, PP. Pocket data from the Entegris 2" carrier PA72-20M (Specialized Handling Products catalog,
            p. 27): pocket spacing 4.76 mm, pocket flat 1.02 mm, D1 14.0 mm (first slot from the end).
  wafer     2 inch (50.8 mm), 279 +/- 25 um, CZ-Si (100), MicroChemicals WSM20279250B1324SNN1; silicon 2.329 g/cm3.
  tweezers  PELCO 5048-SV (Ted Pella), for 2" wafers: 136.5 mm long, points 6 mm x 0.7 mm.
ASSUMPTIONS (no source gives them; README "Assumptions" and the fidelity note): the rib shape (root = pitch - pocket flat,
tapering to 1.8 mm at the tip, scaled from our 72-series carrier model), the side-comb depth, the bottom-rail position and
height (so the seated wafer's top edge sits 1.9 mm under the carrier's top), the tweezers' leaf section, open gap,
spring and mass, and the stand (our own printable part, drawn here).
Frames: carrier slots stack along +y (wafer faces normal to y); the carrier origin is its footprint centre on the table.
"""

from __future__ import annotations

import math
from pathlib import Path

# -- sourced ----------------------------------------------------------------------------------------------------------
CAS_L, CAS_W, CAS_H = 0.142, 0.0674, 0.0615  # Chung King PP-C050 (L along the slot stack)
CAS_MASS = 0.082  # Chung King PP-C050
PITCH, POCKET_FLAT, D1, N_SLOTS = 0.00476, 0.00102, 0.0140, 25  # Entegris PA72-20M (pitch also Chung King)
WAFER_D = 0.0508
WAFER_R = WAFER_D / 2
WAFER_T_REAL = 0.000279  # MicroChemicals WSM20279250B1324SNN1
SI_DENSITY = 2329.0  # kg/m3
TW_LEN, TIP_W, TIP_T = 0.1365, 0.006, 0.0007  # PELCO 5048-SV

# -- drawn thickness (fidelity note if not the real one) ------------------------------------------------------------
import os  # noqa: E402

WAFER_T = float(os.environ.get("WAFER_T_DRAWN", 0.0005))  # drawn 0.5 mm: the thinnest that held in the simulator (fidelity note)
WAFER_MASS = SI_DENSITY * math.pi * WAFER_R**2 * WAFER_T_REAL  # real mass whatever the drawn thickness (1.32 g)

# -- assumptions ------------------------------------------------------------------------------------------------------
RIB_ROOT = PITCH - POCKET_FLAT  # 3.74 mm
RIB_TIP = 0.0018
SIDE_ROOT_X = WAFER_R + 0.0015  # comb groove root, 1.5 mm outside the wafer edge
TOOTH_DEPTH = 0.0034  # comb teeth reach 1.9 mm inside the wafer radius
COMB_Z = (0.016, 0.058)  # comb teeth height range above the table
RAIL_X = 0.015  # bottom rails either side of the centre line; the wafer bears on their inner corners (|x| = 12 mm)
RAIL_CONTACT_X = RAIL_X - 0.003
SEAT_TOP_BELOW = 0.0019  # seated top edge below the carrier top
TW_BODY_T, TW_BODY_W = 0.0015, 0.006
TW_HINGE_GAP = 0.003
TW_OPEN_GAP = 0.008
TW_MASS_HALF = 0.007
TW_SPRING = 2.3  # N m/rad: about 1 N at the tips to close them from open
TW_SPRING_DAMPING = 0.002
CONTACT_OFFSET = float(os.environ.get("PART_CONTACT_OFFSET", 0.0005))
WAFER_OFFSET = float(os.environ.get("WAFER_CONTACT_OFFSET", 0.0004))
STAND = (0.030, 0.054, 0.012)  # our printable tweezer stand (x, y, z): 16 (x) x 18 (y) mm slot, 12 mm deep


def seated_center_z() -> float:
    return CAS_H - SEAT_TOP_BELOW - WAFER_R


def rail_top() -> float:
    """Rail groove root height giving the seated centre above (the wafer bears on the rails' inner corners)."""
    return seated_center_z() - math.sqrt(WAFER_R**2 - RAIL_CONTACT_X**2)


def slot_y(k: int) -> float:
    """Slot k (1..25) centre along the stack, carrier frame."""
    return -CAS_L / 2 + D1 + (k - 1) * PITCH


def tw_open_angle() -> float:
    return math.asin((TW_OPEN_GAP - TW_HINGE_GAP) / TW_LEN)


def tw_closed_angle() -> float:
    return -math.asin(TW_HINGE_GAP / TW_LEN)


def tip_gap_from_hinge(b: float) -> float:
    return TW_HINGE_GAP + TW_LEN * math.sin(tw_open_angle() - b)


def half_point(half: str, x_off: float, z: float):
    """A point of a tweezer leaf in its body frame (x_off outward from the hinge mid-plane, z along the length, tips at
    z = -TW_LEN); leaf B's points are turned open like its shapes."""
    sx = -1.0 if half == "half_a" else 1.0
    p = (sx * x_off, 0.0, z)
    if half == "half_a":
        return p
    th = -tw_open_angle()
    c, s_ = math.cos(th), math.sin(th)
    x, y, zz = p
    return (x * c + zz * s_, y, -x * s_ + zz * c)


# -- USD helpers --------------------------------------------------------------------------------------------------------


def _stage(path: Path, root: str):
    from pxr import Usd, UsdGeom, UsdPhysics

    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    prim = UsdGeom.Xform.Define(stage, f"/{root}").GetPrim()
    stage.SetDefaultPrim(prim)
    return stage, prim


def _material(stage, path, mu):
    from pxr import UsdPhysics, UsdShade

    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(mu)
    api.CreateDynamicFrictionAttr(mu)
    api.CreateRestitutionAttr(0.0)
    return m


def _collider(prim, mat, offset=CONTACT_OFFSET):
    from pxr import Sdf, UsdPhysics, UsdShade

    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def _box(stage, path, center, size, color, mat, rot_wxyz=None):
    from pxr import Gf, UsdGeom

    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    if rot_wxyz is not None:
        c.AddOrientOp().Set(Gf.Quatf(*rot_wxyz))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    _collider(c.GetPrim(), mat)
    return c.GetPrim()


MAX_DEPEN = float(os.environ.get("MAX_DEPEN_VEL", 0.02))  # m/s: cap on the speed contacts push overlapping bodies apart


def _body(prim, mass=None, kinematic=False, iters=(16, 4), max_depen=None):
    from pxr import Sdf, UsdPhysics

    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        rb.CreateKinematicEnabledAttr(True)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(iters[0])
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(iters[1])
    if max_depen is not None:
        # thin parts squeezed between two contacts chatter at the default depenetration speed (measured: a pinched
        # wafer jittered +-0.5 mm across its faces and was thrown out of the tweezers); a low cap removes the chatter
        prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(max_depen)


PP_COLOR = (0.82, 0.82, 0.78)
GREY = (0.6, 0.62, 0.66)


# -- parts --------------------------------------------------------------------------------------------------------------


def write_cassette(path: Path) -> Path:
    """Upright open-top carrier, slots along +y. One kinematic body (it stands still on the bench): two end walls, two
    side walls with comb teeth between the pockets, and two bottom rails with teeth between the pockets."""
    stage, root = _stage(path, "cassette")
    _body(root, mass=CAS_MASS, kinematic=True)
    pp = _material(stage, "/cassette/pp", 0.35)
    t_end = 0.004
    for nm, s in (("a", -1), ("b", 1)):
        _box(stage, f"/cassette/end_{nm}", (0, s * (CAS_L / 2 - t_end / 2), CAS_H / 2), (CAS_W, t_end, CAS_H), PP_COLOR, pp)
    inner = CAS_L - 2 * t_end
    ribs = [slot_y(1) - PITCH / 2 + k * PITCH for k in range(N_SLOTS + 1)]  # rib centres between/around pockets
    zc, zh = (COMB_Z[0] + COMB_Z[1]) / 2, COMB_Z[1] - COMB_Z[0]
    rt = rail_top()
    for side, s in (("p", 1), ("n", -1)):
        wall_x = s * (SIDE_ROOT_X + 0.003)
        _box(stage, f"/cassette/side_{side}", (wall_x, 0, (CAS_H + 0.010) / 2), (0.006, inner, CAS_H - 0.010), PP_COLOR, pp)
        for i, y in enumerate(ribs):
            for part, t, x0 in (("r", RIB_ROOT, SIDE_ROOT_X - TOOTH_DEPTH / 4), ("t", RIB_TIP, SIDE_ROOT_X - 3 * TOOTH_DEPTH / 4)):
                _box(stage, f"/cassette/comb_{side}_{i:02d}{part}", (s * x0, y, zc), (TOOTH_DEPTH / 2, t, zh), PP_COLOR, pp)
        rx = s * RAIL_X
        _box(stage, f"/cassette/rail_{side}", (rx, 0, rt / 2), (0.006, inner, rt), PP_COLOR, pp)
        for i, y in enumerate(ribs):
            _box(stage, f"/cassette/rt_{side}_{i:02d}r", (rx, y, rt + 0.001), (0.006, RIB_ROOT, 0.002), PP_COLOR, pp)
            _box(stage, f"/cassette/rt_{side}_{i:02d}t", (rx, y, rt + 0.003), (0.006, RIB_TIP, 0.002), PP_COLOR, pp)
    stage.GetRootLayer().Save()
    return path


def write_wafer(path: Path, color=(0.42, 0.44, 0.55)) -> Path:
    """A 2-inch wafer, faces normal to its local y, drawn WAFER_T thick at its real mass."""
    from pxr import Gf, UsdGeom

    stage, root = _stage(path, "wafer")
    _body(root, mass=WAFER_MASS, iters=(64, 16), max_depen=MAX_DEPEN)
    si = _material(stage, "/wafer/si", 0.3)
    c = UsdGeom.Cylinder.Define(stage, "/wafer/disc")
    c.CreateRadiusAttr(WAFER_R)
    c.CreateHeightAttr(WAFER_T)
    c.CreateAxisAttr("Y")
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    _collider(c.GetPrim(), si, offset=WAFER_OFFSET)
    stage.GetRootLayer().Save()
    return path


def write_stand(path: Path) -> Path:
    """Our printable tweezer stand: a block with a through slot the tweezer tips stand in (squeeze axis along y)."""
    stage, root = _stage(path, "stand")
    _body(root, mass=0.03, kinematic=True)
    pla = _material(stage, "/stand/pla", 0.4)
    hx, hy, hz = STAND
    sx, sy = 0.016, 0.018
    c = (0.3, 0.3, 0.32)
    _box(stage, "/stand/ya", (0, (hy + sy) / 4, hz / 2), (hx, (hy - sy) / 2, hz), c, pla)
    _box(stage, "/stand/yb", (0, -(hy + sy) / 4, hz / 2), (hx, (hy - sy) / 2, hz), c, pla)
    _box(stage, "/stand/xa", ((hx + sx) / 4, 0, hz / 2), ((hx - sx) / 2, sy, hz), c, pla)
    _box(stage, "/stand/xb", (-(hx + sx) / 4, 0, hz / 2), ((hx - sx) / 2, sy, hz), c, pla)
    stage.GetRootLayer().Save()
    return path


def write_tweezers(path: Path) -> Path:
    """Two leaves joined at the back end by a sprung hinge (an angular force drive toward 0 = sprung open). Length along
    local -z from the back end (origin) to the tips at z = -TW_LEN; leaves separated along local x (squeeze axis),
    width along y. Leaf B is drawn already open, so the joint's zero is the sprung-open rest."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    stage, root = _stage(path, "tweezers")
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.AddAppliedSchema("PhysxArticulationAPI")
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    root.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    root.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(8)
    ss = _material(stage, "/tweezers/tips", 0.5)
    k_tip = float(os.environ.get("TIP_COMPLIANT_K", 0))  # optional compliant tip contact (N/m), see the fidelity note
    if k_tip > 0:
        from pxr import Sdf as _Sdf

        mp = ss.GetPrim()
        mp.AddAppliedSchema("PhysxMaterialAPI")
        mp.CreateAttribute("physxMaterial:compliantContactStiffness", _Sdf.ValueTypeNames.Float).Set(k_tip)
        mp.CreateAttribute("physxMaterial:compliantContactDamping", _Sdf.ValueTypeNames.Float).Set(
            float(os.environ.get("TIP_COMPLIANT_D", 5.0)))
    tip_len = 0.012
    body_len = TW_LEN - tip_len
    th = -tw_open_angle()
    c, s_ = math.cos(th), math.sin(th)
    q_open = (math.cos(th / 2), 0.0, math.sin(th / 2), 0.0)

    def turn(p):
        x, y, z = p
        return (x * c + z * s_, y, -x * s_ + z * c)

    halves = {}
    for name, sx in (("half_a", -1), ("half_b", 1)):
        x0 = sx * (TW_HINGE_GAP / 2 + TW_BODY_T / 2)
        xt = sx * (TW_HINGE_GAP / 2 + TIP_T / 2)
        prim = UsdGeom.Xform.Define(stage, f"/tweezers/{name}").GetPrim()
        _body(prim, mass=TW_MASS_HALF, iters=(32, 8), max_depen=MAX_DEPEN)
        pb, pt = (x0, 0, -body_len / 2), (xt, 0, -body_len - tip_len / 2)
        rot = None
        if name == "half_b":
            pb, pt, rot = turn(pb), turn(pt), q_open
        _box(stage, f"/tweezers/{name}/body", pb, (TW_BODY_T, TW_BODY_W, body_len), GREY, ss, rot)
        _box(stage, f"/tweezers/{name}/tip", pt, (TIP_T, TIP_W, tip_len), (0.15, 0.15, 0.15), ss, rot)
        halves[name] = prim
    j = UsdPhysics.RevoluteJoint.Define(stage, "/tweezers/hinge")
    j.CreateBody0Rel().SetTargets([halves["half_a"].GetPath()])
    j.CreateBody1Rel().SetTargets([halves["half_b"].GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(math.degrees(tw_open_angle() - tw_closed_angle()))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateStiffnessAttr(TW_SPRING * math.pi / 180.0)  # USD angular drive units are per degree
    drive.CreateDampingAttr(TW_SPRING_DAMPING * math.pi / 180.0)
    drive.CreateMaxForceAttr(1.0e4)
    stage.GetRootLayer().Save()
    return path


ASSET_NAMES = ("cassette", "wafer", "stand", "tweezers")


def asset_paths(folder: Path) -> dict[str, Path]:
    paths = {n: folder / f"{n}.usda" for n in ASSET_NAMES}
    missing = [str(p) for p in paths.values() if not p.is_file()]
    if missing:
        raise FileNotFoundError(f"run `python parts.py` to write {missing}")
    return paths


def write_all(folder: Path) -> dict[str, Path]:
    folder.mkdir(parents=True, exist_ok=True)
    return {"cassette": write_cassette(folder / "cassette.usda"), "wafer": write_wafer(folder / "wafer.usda"),
            "stand": write_stand(folder / "stand.usda"), "tweezers": write_tweezers(folder / "tweezers.usda")}


if __name__ == "__main__":
    for name, p in write_all(Path(__file__).resolve().parent / os.environ.get("ASSET_DIR", "assets")).items():
        print(name, p)
    print(f"seated centre {seated_center_z() * 1000:.1f} mm, rail top {rail_top() * 1000:.1f} mm, "
          f"wafer {WAFER_T * 1e6:.0f} um drawn, {WAFER_MASS * 1000:.2f} g")
