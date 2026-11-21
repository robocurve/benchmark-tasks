"""zipkit: a press-to-close zip as rigid profile segments held shut by breakable joints, for clothkit sheets.

A real zip is two plastic profiles, one on each film, that snap together. Here each profile is a row of short rigid
ribs (one per cell along the zip), each fixed to its film's zip-line vertices (a PhysX vertex-to-body attachment). The
two ribs of a cell are held together by a PhysX fixed joint with a break force and a break torque: pulled apart
harder than that, PhysX breaks the joint for good and the cell is open. The ribs do not collide (the films do).

Ribs are authored next to the host body when it is spawned, read back through one rigid-body view after every
control step and put back, with fresh joints, at every reset. Pure Python at import time.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import bodykit
import clothkit


@dataclass
class Zip:
    name: str
    host: str
    top_sheet: str
    bottom_sheet: str
    cells: list  # [(top vertex, bottom vertex, (top neighbours...), (bottom neighbours...))]
    break_force: float = 1.0  # N per cell
    break_torque: float = 0.004  # N m per cell
    rib_mass: float = 0.0005  # kg per rib
    rib_size: tuple = (0.006, 0.003, 0.0015)  # along the zip, across, thick (only its inertia matters)
    open_gap: float = 0.0025  # ribs this far apart: the cell is open


_ZIPS: dict[str, Zip] = {}
_STATE: dict[str, dict] = {}


def zip_(*args, **kwargs) -> Zip:
    z = Zip(*args, **kwargs)
    _ZIPS[z.name] = z
    return z


def _rib_paths(z: Zip, root: str):
    return [f"{root}/rib_t{i:02d}" for i in range(len(z.cells))], [f"{root}/rib_b{i:02d}" for i in range(len(z.cells))]


def _author(stage, prim_path: str, spec) -> None:
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    for z in _ZIPS.values():
        if z.host != spec.name:
            continue
        root = f"{parent}/{z.name}"
        UsdGeom.Scope.Define(stage, root)
        top = clothkit.get(z.top_sheet)
        bot = clothkit.get(z.bottom_sheet)
        tstart = top.points if top.start is None else top.start
        bstart = bot.points if bot.start is None else bot.start
        tp, bp = _rib_paths(z, root)
        start = []
        for i, (vt, vb, nt, nb) in enumerate(z.cells):
            for path, p in ((tp[i], tstart[vt]), (bp[i], bstart[vb])):
                x = UsdGeom.Xform.Define(stage, path)
                x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, p)))
                x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
                prim = x.GetPrim()
                UsdPhysics.RigidBodyAPI.Apply(prim)
                m = UsdPhysics.MassAPI.Apply(prim)
                m.CreateMassAttr(float(z.rib_mass))
                a, b, c = z.rib_size
                k = z.rib_mass / 12.0
                m.CreateDiagonalInertiaAttr(Gf.Vec3f(k * (b * b + c * c), k * (a * a + c * c), k * (a * a + b * b)))
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                rb.CreateSolverPositionIterationCountAttr(32)
                rb.CreateSolverVelocityIterationCountAttr(1)
                rb.CreateLinearDampingAttr(0.5)
                rb.CreateAngularDampingAttr(2.0)
                rb.CreateMaxDepenetrationVelocityAttr(0.3)
                start.append(np.asarray(p, float))
        _STATE[z.name] = {"root": root, "parent": parent, "view": None, "start": np.array(start),
                          "top": tp, "bottom": bp, "now": np.array(start), "open": set()}
        for i in range(len(z.cells)):
            _author_cell_pins(stage, z, i)
            _author_joint(stage, z, i)


def _author_cell_pins(stage, z: Zip, i: int) -> None:
    """Fix rib i of each profile to its film's zip vertex and its neighbours along the zip."""
    from pxr import Gf, Sdf, Vt

    st = _STATE[z.name]
    vt, vb, nt, nb = z.cells[i]
    for sheet, verts, body in ((z.top_sheet, (vt, *nt), st["top"][i]), (z.bottom_sheet, (vb, *nb), st["bottom"][i])):
        s = clothkit.get(sheet)
        pts = s.points if s.start is None else s.start
        origin = pts[verts[0]]
        local = pts[list(verts)] - origin
        path = f"{body}/pin"
        prim = stage.DefinePrim(path, "OmniPhysicsVtxXformAttachment")
        prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(f"{st['parent']}/{sheet}/sim_mesh")])
        prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(body)])
        prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(
            Vt.IntArray([int(v) for v in verts]))
        prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
            Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in local]))
        prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)


def _author_joint(stage, z: Zip, i: int) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    st = _STATE[z.name]
    path = f"{st['root']}/cell{i:02d}"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    j = UsdPhysics.FixedJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(st["top"][i])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(st["bottom"][i])])
    pt = st["start"][2 * i]
    pb = st["start"][2 * i + 1]
    mid = (pt + pb) / 2
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, mid - pt)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, mid - pb)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateBreakForceAttr(float(z.break_force))
    j.CreateBreakTorqueAttr(float(z.break_torque))
    j.CreateCollisionEnabledAttr(False)


bodykit.EXTRA_AUTHORS.append(_author)


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_rigid_body_view(f"{st['root']}/rib_*")
    return st["view"]


def ribs(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(top rib positions, bottom rib positions), N x 3 each, env frame."""
    st = _STATE[name]
    return st["top_now"], st["bottom_now"]


def open_cells(name: str) -> list[int]:
    return sorted(_STATE[name]["open"])


def separations(name: str) -> np.ndarray:
    t, b = ribs(name)
    return np.linalg.norm(t - b, axis=1)


@bodykit.on_step
def _read(env) -> None:
    import warp as wp

    for name, st in _STATE.items():
        v = _view(name)
        origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
        tr = wp.to_torch(v.get_transforms()).cpu().numpy().astype(float)[:, :3] - origin
        # the view orders prims by path: rib_b.. before rib_t..
        paths = list(v.prim_paths) if hasattr(v, "prim_paths") else sorted(st["bottom"] + st["top"])
        idx = {p: k for k, p in enumerate(paths)}
        st["top_now"] = np.array([tr[idx[p]] for p in st["top"]])
        st["bottom_now"] = np.array([tr[idx[p]] for p in st["bottom"]])
        z = _ZIPS[name]
        sep = np.linalg.norm(st["top_now"] - st["bottom_now"], axis=1)
        for i in np.where(sep > z.open_gap)[0]:
            st["open"].add(int(i))


@bodykit.before_reset
def _put_back() -> None:
    import torch
    import warp as wp

    for name, st in _STATE.items():
        if st["view"] is None:
            continue
        v = st["view"]
        paths = list(v.prim_paths) if hasattr(v, "prim_paths") else sorted(st["bottom"] + st["top"])
        start = {}
        z = _ZIPS[name]
        for i in range(len(z.cells)):
            start[st["top"][i]] = st["start"][2 * i]
            start[st["bottom"][i]] = st["start"][2 * i + 1]
        pose = torch.zeros((len(paths), 7), dtype=torch.float32, device="cuda:0")
        for k, p in enumerate(paths):
            pose[k, :3] = torch.tensor(start[p], dtype=torch.float32)
            pose[k, 6] = 1.0
        idx = wp.array(np.arange(len(paths), dtype=np.int32), dtype=wp.int32, device="cuda:0")
        v.set_transforms(wp.from_torch(pose), indices=idx)
        v.set_velocities(wp.from_torch(torch.zeros((len(paths), 6), dtype=torch.float32, device="cuda:0")),
                         indices=idx)
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        for i in list(st["open"]):
            _author_joint(stage, z, i)
        st["open"].clear()
