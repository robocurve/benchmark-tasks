"""Assets of tasks/lift-out-tea-infuser, made from two NVIDIA USDCraft-10K models (Apache-2.0,
huggingface.co/datasets/nvidia/USDCraft-10K):

    python make_assets.py <folder with R089_03.usdc and R086_06.usdc> <task assets folder>

  rigid/kitchen_dining/R089_teapot/03_glass_teapot/physx/model.usdc          -> R089_03.usdc
  rigid/kitchen_dining/R086_teacup_with_saucer/06_glass_tea_cup_and_saucer/physx/model.usdc -> R086_06.usdc
Needs pxr (usd-core). Writes, each as one rigid body on /Asset and all in the teapot's frame (origin at the centre of
its foot, z up):
  teapot.usdc   body, spout and handle of the glass teapot (the model's lid and knob taken out), mass 0.40 kg + 0.50 kg
                of water (the real pot is filled; water is not simulated)
  lid.usdc      the model's lid and knob as their own loose body, 0.08 kg
  infuser.usdc  the model's steel infuser basket (8.5 cm wide, 13 cm tall, a bar handle across its top), 0.139 kg
  saucer.usdc   the model's 14.2 cm glass saucer, 0.143 kg
and prints the geometry the task file needs.
"""
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

WATER = 0.50


def keep(src: Path, dst: Path, link: str, drop: list[str], mass: float | None) -> None:
    st = Usd.Stage.Open(str(src))
    links = st.GetPrimAtPath("/Asset/Links")
    for child in list(links.GetChildren()):
        if child.GetName() != link:
            st.RemovePrim(child.GetPath())
    for part in drop:
        for scope in ("Visuals", "Collisions"):
            st.RemovePrim(f"/Asset/Links/{link}/{scope}/{part}")
    for j in list(st.GetPrimAtPath("/Asset/Joints").GetChildren()) if st.GetPrimAtPath("/Asset/Joints") else []:
        st.RemovePrim(j.GetPath())
    lk = st.GetPrimAtPath(f"/Asset/Links/{link}")
    old = lk.GetAttribute("physics:mass").Get()
    lk.RemoveAPI(UsdPhysics.MassAPI)
    lk.RemoveAPI(UsdPhysics.RigidBodyAPI)
    for name in [a.GetName() for a in lk.GetAttributes() if a.GetName().startswith("physics:")]:
        lk.RemoveProperty(name)
    root = st.GetPrimAtPath("/Asset")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(mass if mass is not None else old))
    st.GetRootLayer().Export(str(dst))


def primitive_colliders(path: Path) -> None:
    """Replace the infuser's collision meshes (a perforated, open mesh whose signed-distance field makes every leaf inside it
    count as inside the wall: the leaves, the basket and the pot all blew apart at the start) by primitives: a disc floor,
    a ring of 36 boxes for the wall, a ring of 36 for the rim, and a box for the handle bar. The wall is solid where the
    model's is a mesh; the visuals are the model's own."""
    import math

    st = Usd.Stage.Open(str(path))
    col = "/Asset/Links/infuser_basket/Collisions"
    for child in list(st.GetPrimAtPath(col).GetChildren()):
        st.RemovePrim(child.GetPath())

    def cube(name, centre, size, deg=0.0):
        c = UsdGeom.Cube.Define(st, f"{col}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if deg:
            c.AddRotateZOp().Set(deg)
        c.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
        c.MakeInvisible()

    n = 36
    # (local to the basket link: floor centre at 0, wall from 1.5 to 132 mm, rim 130.4 to 133.6 mm, bar 129.15 to 139.7 mm)
    cyl = UsdGeom.Cylinder.Define(st, f"{col}/floor")
    cyl.CreateRadiusAttr(0.0426)
    cyl.CreateHeightAttr(0.0015)
    cyl.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    cyl.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
    cyl.MakeInvisible()
    for k in range(n):
        a = 2 * math.pi * k / n
        r_w = 0.0426 - 0.00075
        cube(f"wall{k:02d}", (r_w * math.cos(a), r_w * math.sin(a), (0.0015 + 0.132) / 2),
             (0.0015, 2 * math.pi * r_w / n * 1.04, 0.132 - 0.0015), math.degrees(a))
        r_r = 0.0441 - 0.001
        cube(f"rim{k:02d}", (r_r * math.cos(a), r_r * math.sin(a), (0.1304 + 0.1336) / 2),
             (0.002, 2 * math.pi * r_r / n * 1.04, 0.0032), math.degrees(a))
    cube("bar", (0.0, 0.0, (0.12915 + 0.1397) / 2), (0.0867, 0.0024, 0.1397 - 0.12915))
    st.GetRootLayer().Save()


def pot_primitives(path: Path) -> None:
    """The teapot's body as primitives: with the model's signed-distance-field collider, a basket resting on the 6 mm floor
    sank 3 mm into it and, once the leaves were in the basket, the pot, basket and leaves were all thrown out at the start.
    Here the body's collision mesh is read for its inner radius by height, and rebuilt as a disc floor and 30 x 12 boxes
    (4 mm thick) following that profile up to the 96 mm mouth. The spout and handle keep their meshes as convex hulls."""
    import math

    st = Usd.Stage.Open(str(path))
    col = "/Asset/Links/teapot_glass/Collisions"
    pts = []
    for p in Usd.PrimRange(st.GetPrimAtPath(f"{col}/body")):
        if p.GetTypeName() == "Mesh":
            pts.append(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), float))
    P = np.vstack(pts)
    r = np.hypot(P[:, 0], P[:, 1])
    zs = np.linspace(0.006, 0.150, 13)
    prof = []
    for z in zs:
        m = (np.abs(P[:, 2] - z) < 0.0015) & (r > 0.040)
        prof.append(float(r[m].min()))
    prof[0] = 0.0455  # the floor's edge: the inner radius at 8 mm is 46.5, at 12 mm 52.4
    print("pot inner radius by height (mm):", [(round(z * 1000), round(q * 1000, 1)) for z, q in zip(zs, prof)])
    st.RemovePrim(f"{col}/body")
    for part in ("spout", "handle"):
        for p in Usd.PrimRange(st.GetPrimAtPath(f"{col}/{part}")):
            if p.GetTypeName() == "Mesh":
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
    cyl = UsdGeom.Cylinder.Define(st, f"{col}/floor")
    cyl.CreateRadiusAttr(prof[0])
    cyl.CreateHeightAttr(0.006)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.003))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    cyl.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
    cyl.MakeInvisible()
    n, t = 30, 0.004
    for j in range(len(zs) - 1):
        r0, r1, z0, z1 = prof[j], prof[j + 1], zs[j], zs[j + 1]
        L = math.hypot(r1 - r0, z1 - z0) * 1.04
        phi = math.degrees(math.atan2(r1 - r0, z1 - z0))
        rc = (r0 + r1) / 2 + t / 2
        for k in range(n):
            a = 2 * math.pi * k / n
            c = UsdGeom.Cube.Define(st, f"{col}/belly_{j:02d}_{k:02d}")
            c.CreateSizeAttr(1.0)
            c.AddTranslateOp().Set(Gf.Vec3d(rc * math.cos(a), rc * math.sin(a), (z0 + z1) / 2))
            c.AddRotateZOp().Set(math.degrees(a))
            c.AddRotateYOp().Set(phi)
            c.AddScaleOp().Set(Gf.Vec3f(t, 2 * math.pi * rc / n * 1.06, L))
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            c.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
            c.MakeInvisible()
    st.GetRootLayer().Save()


def lid_hulls(path: Path) -> None:
    """The lid's plate and knob as convex hulls."""
    st = Usd.Stage.Open(str(path))
    for part in ("lid", "knob"):
        for p in Usd.PrimRange(st.GetPrimAtPath(f"/Asset/Links/teapot_glass/Collisions/{part}")):
            if p.GetTypeName() == "Mesh":
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
    st.GetRootLayer().Save()


def bounds(path: Path, prim: str) -> np.ndarray:
    st = Usd.Stage.Open(str(path))
    c = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_], useExtentsHint=False)
    r = c.ComputeWorldBound(st.GetPrimAtPath(prim)).ComputeAlignedRange()
    return np.array([list(r.GetMin()), list(r.GetMax())])


if __name__ == "__main__":
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    pot, saucer = src / "R089_03.usdc", src / "R086_06.usdc"
    keep(pot, dst / "teapot.usdc", "teapot_glass", ["lid", "knob"], 0.40 + WATER)
    keep(pot, dst / "lid.usdc", "teapot_glass", ["body", "spout", "handle"], 0.08)
    keep(pot, dst / "infuser.usdc", "infuser_basket", [], None)
    primitive_colliders(dst / "infuser.usdc")
    pot_primitives(dst / "teapot.usdc")
    lid_hulls(dst / "lid.usdc")
    keep(saucer, dst / "saucer.usdc", "saucer", [], None)
    np.set_printoptions(precision=4, suppress=True)
    for f, prim in (("teapot", "/Asset"), ("lid", "/Asset"), ("infuser", "/Asset"), ("saucer", "/Asset")):
        b = bounds(dst / f"{f}.usdc", prim)
        print(f, "min", b[0], "max", b[1], "size_mm", (b[1] - b[0]) * 1000)
    # the pot's mouth: radius of the body mesh's rim by height
    st = Usd.Stage.Open(str(dst / "teapot.usdc"))
    for part in ("body",):
        for p in st.GetPrimAtPath(f"/Asset/Links/teapot_glass/Visuals/{part}").GetChildren():
            pass
    pts = []
    for p in Usd.PrimRange(st.GetPrimAtPath("/Asset/Links/teapot_glass/Visuals/body")):
        if p.GetTypeName() == "Mesh":
            pts.append(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), float))
    P = np.vstack(pts)
    print("body points", len(P), "centre xy", P[:, :2].mean(0), "x range", P[:, 0].min(), P[:, 0].max())
    r = np.hypot(P[:, 0], P[:, 1])
    for z in (0.002, 0.05, 0.075, 0.10, 0.12, 0.13, 0.14, 0.145, 0.15):
        s = (P[:, 2] > z - 0.002) & (P[:, 2] < z + 0.002)
        if s.any():
            print(f"  z={z * 1000:.0f} mm: r {r[s].min() * 1000:.1f}..{r[s].max() * 1000:.1f}")
    # the infuser: radius by height and its handle bar
    st = Usd.Stage.Open(str(dst / "infuser.usdc"))
    for name in ("basket_floor", "basket_wall", "basket_rim", "basket_handle"):
        q = []
        for p in Usd.PrimRange(st.GetPrimAtPath(f"/Asset/Links/infuser_basket/Visuals/{name}")):
            if p.GetTypeName() == "Mesh":
                q.append(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), float))
        Q = np.vstack(q)
        print(name, "x", Q[:, 0].min(), Q[:, 0].max(), "y", Q[:, 1].min(), Q[:, 1].max(), "z", Q[:, 2].min(), Q[:, 2].max())
