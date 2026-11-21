"""The gum's stick: when a strip may come off, and the joints that hold the wad in the simulator.

Chewing gum trodden flat grips a sole in shear and gives way to peeling: a thin edge slid under it at a low angle
lifts it, a blunt fingertip pushed against it does not. Rigid-body joints can't tell the two apart, so this file
does: every strip starts LOCKED to the sole (a joint nothing here can break), and a strip is UNLOCKED, for good, at
the first control step where the knife's working edge gets under it the way a scraper must (:func:`engaged`): down
on the sole, laid low, edge leading. An unlocked strip is tied to its place by a spring, as gum
stretches before it lets go: the blade wedging under it has to draw it ``let_go`` from its place, and until then the
pull goes into the shoe. Neighbouring strips are hinged to each other, so the wad lifts and curls as one sheet until
a hinge is torn at ``cohesion_force``.

Pure Python at import. In Isaac Sim, :func:`reset` (run after every reset) and :func:`update` (run every control
step, from the task's score()) write the joints with pxr; anywhere else they only keep the bookkeeping, so the
scorer and its tests run without a simulator.
"""

from __future__ import annotations

import math

import numpy as np

ROOT = "/World/envs/env_0"
JOINTS = ROOT + "/GumJoints"
LOCKED_FORCE = 1.0e6  # N: a locked piece's joint; nothing on the table reaches it

_STATE = {"cfg": None, "pieces": {}, "unlocked": set(), "released": set(), "live": False, "fouled": "", "taken": set()}


def rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(quat_xyzw, dtype=float) / np.linalg.norm(quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_local(state, point) -> np.ndarray:
    """A world point in an object's own frame (origin at its bounding-box centre)."""
    return rot(state.quat_xyzw).T @ (np.asarray(point, dtype=float) - np.asarray(state.pos, dtype=float))


def engaged(shoe, knife, piece_local, cfg) -> bool:
    """Whether the knife is scraping this piece right now. All four must hold, measured in the shoe's frame:

    - the working edge is at the piece: the piece's centre is within ``reach`` of the edge line, along the sole;
    - the edge is down on the sole: no higher above it than this piece is thick (``piece_local`` is the piece's
      centre, half its thickness above the sole), plus ``edge_slack``;
    - the blade is laid low: at most ``max_angle_deg`` between the blade and the sole;
    - the edge leads: the blade points from its handle toward the piece, not away from it.
    """
    Rk = rot(knife.quat_xyzw)
    a = to_local(shoe, np.asarray(knife.pos) + Rk @ np.asarray(cfg["edge"][0]))
    b = to_local(shoe, np.asarray(knife.pos) + Rk @ np.asarray(cfg["edge"][1]))
    blade = rot(shoe.quat_xyzw).T @ (Rk @ np.asarray(cfg["blade_dir"], dtype=float))
    sole_z = cfg["sole_z"]
    thickness = 2.0 * (float(piece_local[2]) - sole_z)
    if max(a[2], b[2]) - sole_z > thickness + cfg["edge_slack"]:
        return False
    if "under" in cfg and max(a[2], b[2]) - sole_z > cfg["under"]:  # not down on the sole: over the gum, not under it
        return False
    if min(a[2], b[2]) - sole_z < -0.004:  # under the sole's surface: the knife is beside the shoe, not on it
        return False
    angle = math.degrees(math.asin(min(1.0, abs(float(blade[2])) / max(np.linalg.norm(blade), 1e-9))))
    if angle > cfg["max_angle_deg"]:
        return False
    p, a2, b2 = np.asarray(piece_local[:2]), a[:2], b[:2]
    ab = b2 - a2
    t = float(np.clip(np.dot(p - a2, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    nearest = a2 + t * ab
    if np.linalg.norm(p - nearest) > cfg["reach"]:
        return False
    return bool(np.dot(p - nearest, blade[:2]) >= -cfg["piece_size"] / 2)  # the piece is ahead of the edge


def setup(cfg, pieces) -> None:
    """``cfg`` is the task's GUM settings; ``pieces`` maps each piece's name to its start centre in the shoe's
    frame. Called once when the task file loads."""
    _STATE["cfg"], _STATE["pieces"] = dict(cfg), {k: np.asarray(v, dtype=float) for k, v in pieces.items()}
    _STATE["unlocked"] = set()


def unlocked() -> frozenset:
    return frozenset(_STATE["unlocked"])


def fouled() -> str:
    """Why the trial is fouled ("" when clean): something other than the blade moved gum from its place."""
    return _STATE["fouled"]


def reset() -> None:
    """After every reset: every piece locked again, the neighbours joined again, no foul."""
    _STATE["unlocked"], _STATE["released"], _STATE["fouled"], _STATE["taken"] = set(), set(), "", set()
    stage = _stage()
    _STATE["live"] = stage is not None
    if stage is None:
        return
    if stage.GetPrimAtPath(JOINTS).IsValid():
        stage.RemovePrim(JOINTS)
    cfg = _STATE["cfg"]
    for name in _STATE["pieces"]:
        _joint(stage, f"stick_{name}", f"{ROOT}/shoe", f"{ROOT}/{name}", LOCKED_FORCE, LOCKED_FORCE)
    for a, b in cfg["neighbours"]:
        if "hinge_axis" in cfg:  # neighbours hinge, so the wad curls up over the blade as one sheet
            _hinge(stage, f"hold_{a}_{b}", f"{ROOT}/{a}", f"{ROOT}/{b}", cfg)
        else:
            _joint(stage, f"hold_{a}_{b}", f"{ROOT}/{a}", f"{ROOT}/{b}", cfg["cohesion_force"], cfg["cohesion_torque"])


def update(objects) -> None:
    """Every control step: unlock the pieces the knife is scraping (once unlocked, a piece stays unlocked), then
    check for a foul: a piece off its place that the blade can't have moved."""
    cfg = _STATE["cfg"]
    if cfg is None or "shoe" not in objects or "knife" not in objects:
        return
    shoe, knife = objects["shoe"], objects["knife"]
    for name, local in _STATE["pieces"].items():
        if name in _STATE["unlocked"] or not engaged(shoe, knife, local, cfg):
            continue
        _STATE["unlocked"].add(name)
        stage = _stage() if _STATE["live"] else None
        if stage is not None:
            path = f"{JOINTS}/stick_{name}"
            if stage.GetPrimAtPath(path).IsValid():
                stage.RemovePrim(path)
            # held again where the piece is now, but by a spring: gum stretches before it lets go
            _tether(stage, f"stick_{name}", f"{ROOT}/shoe", f"{ROOT}/{name}", cfg)
    _let_go(shoe, objects, cfg)
    _check_foul(shoe, knife, objects, cfg)


def released() -> frozenset:
    return frozenset(_STATE["released"])


def _let_go(shoe, objects, cfg) -> None:
    """An unlocked strip is tied to its place on the sole by a spring (``tether_k``): the blade wedging under it has to
    stretch that, and the pull goes into the shoe, which slides along with the knife unless something holds it. The
    strip lets go for good once it has been drawn ``let_go`` from its place."""
    for name in _STATE["unlocked"] - _STATE["released"]:
        if name not in objects:
            continue
        if np.linalg.norm(to_local(shoe, objects[name].pos) - _STATE["pieces"][name]) <= cfg["let_go"]:
            continue
        _STATE["released"].add(name)
        stage = _stage() if _STATE["live"] else None
        if stage is not None and stage.GetPrimAtPath(f"{JOINTS}/stick_{name}").IsValid():
            stage.RemovePrim(f"{JOINTS}/stick_{name}")


def _check_foul(shoe, knife, objects, cfg) -> None:
    """The rule for real trials: only the blade takes gum off. A piece more than ``foul_move`` from its place on the
    sole is fouled if it was never unlocked, or if the knife's edge is more than ``foul_reach`` from that place (a
    gripper, the handle or a shake moved it). Once a piece has left with the edge beside it, it is taken, and where
    it goes after that (the table, carried on the blade) is the blade's doing."""
    if _STATE["fouled"]:
        return
    move, reach = cfg.get("foul_move", 0.002), cfg.get("foul_reach", 0.06)
    Rk = rot(knife.quat_xyzw)
    mid = (np.asarray(cfg["edge"][0], dtype=float) + np.asarray(cfg["edge"][1], dtype=float)) / 2
    edge = to_local(shoe, np.asarray(knife.pos) + Rk @ mid)
    for name, local in _STATE["pieces"].items():
        if name in _STATE["taken"] or name not in objects:
            continue
        if np.linalg.norm(to_local(shoe, objects[name].pos) - local) <= move:
            continue
        if name not in _STATE["unlocked"]:
            _STATE["fouled"] = f"{name} moved while still stuck: not by the blade"
        elif np.linalg.norm(edge - local) > reach:
            _STATE["fouled"] = f"{name} moved with the knife's edge away from it: not by the blade"
        else:
            _STATE["taken"].add(name)
            continue
        return


# -- Isaac Sim only ---------------------------------------------------------------------------------------------------

def _stage():
    try:
        import omni.usd
    except ImportError:
        return None
    return omni.usd.get_context().get_stage()


def _joint(stage, name, body0, body1, break_force, break_torque) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    for body in (body0, body1):
        if not stage.GetPrimAtPath(body).IsValid():
            raise RuntimeError(f"gum joint {name}: no body at {body}")
    joint = UsdPhysics.FixedJoint.Define(stage, f"{JOINTS}/{name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    cache = UsdGeom.XformCache()
    t0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body0))
    t1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body1))
    rel = t1 * t0.GetInverse()  # body1's frame in body0's (USD row vectors)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat()))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0)))
    joint.CreateBreakForceAttr().Set(float(break_force))
    joint.CreateBreakTorqueAttr().Set(float(break_torque))
    joint.CreateCollisionEnabledAttr().Set(False)


def _tether(stage, name, body0, body1, cfg) -> None:
    """A spring between a strip and its place on the sole: a joint free on all six axes, each with a drive that pulls
    it back (``tether_k`` N/m and ``tether_c`` N s/m along, ``tether_kr`` N m/deg and ``tether_cr`` about)."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    joint = UsdPhysics.Joint.Define(stage, f"{JOINTS}/{name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    cache = UsdGeom.XformCache()
    t0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body0))
    t1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body1))
    rel = t1 * t0.GetInverse()
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat()))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0)))
    joint.CreateCollisionEnabledAttr().Set(False)
    for axis, k, c in (("transX", "tether_k", "tether_c"), ("transY", "tether_k", "tether_c"), ("transZ", "tether_k", "tether_c"),
                       ("rotX", "tether_kr", "tether_cr"), ("rotY", "tether_kr", "tether_cr"), ("rotZ", "tether_kr", "tether_cr")):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateTargetPositionAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(float(cfg[k]))
        drive.CreateDampingAttr().Set(float(cfg[c]))


def _hinge(stage, name, body0, body1, cfg) -> None:
    """A hinge between two neighbouring strips, on the line where they meet, turning about the strips' long axis. It
    tears at the cohesion force."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    t0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body0))
    t1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body1))
    mid = (t0.ExtractTranslation() + t1.ExtractTranslation()) / 2.0
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{JOINTS}/{name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateAxisAttr().Set(cfg["hinge_axis"])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(t0.GetInverse().Transform(mid)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(t1.GetInverse().Transform(mid)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0)))
    joint.CreateLowerLimitAttr().Set(-float(cfg["hinge_limit_deg"]))
    joint.CreateUpperLimitAttr().Set(float(cfg["hinge_limit_deg"]))
    joint.CreateBreakForceAttr().Set(float(cfg["cohesion_force"]))
    joint.CreateBreakTorqueAttr().Set(float(cfg["cohesion_torque"]))
    joint.CreateCollisionEnabledAttr().Set(False)


try:  # a frozen task folder has labkit, whose on_reset runs after every reset in Isaac Sim
    from labkit import on_reset as _on_reset
except ImportError:
    _on_reset = None
if _on_reset is not None:
    _on_reset(reset)
