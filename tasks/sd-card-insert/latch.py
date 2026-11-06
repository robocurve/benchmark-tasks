"""latch: a push-to-click catch between a card and the reader it slides into, switched on at run time.

:func:`catch` registers a prismatic joint between two task-file boxes, authored switched off when the card spawns:
body 0 is the reader, body 1 the card. Its frame on the reader is ``at`` (where the card's centre sits when it has
clicked in, in the reader's own frame) with the reader's axes, its frame on the card is the card's centre and axes,
and it slides along the reader's x axis from 0 (latched) to ``travel`` metres further in. So it holds the card level
and centred in the slot, free only to go deeper, and stops it from coming back out past the click. While it is on, the
card and the reader don't collide: the joint alone holds the card where the slot's walls would. Its frames don't
depend on where the card starts, so switching it on with :func:`engage` once the card has been pushed in far enough
holds it where it is. Every reset switches it off again.

Pure Python at import time. ``pxr`` is imported when the card spawns, inside Isaac Sim.
"""

from __future__ import annotations

from bonds import on_reset, on_spawn

__all__ = ["catch", "engage", "engaged"]

_SPEC: dict = {}
_PRIM: dict = {}
_ON: dict = {"on": False}


def catch(card: str, reader: str, at: tuple[float, float, float], travel: float) -> None:
    """Register the catch: ``card`` is held at ``at`` (reader frame, m) once engaged, free to go ``travel`` m deeper."""
    if not travel > 0:
        raise ValueError("travel must be positive")
    _SPEC.update(card=card, reader=reader, at=tuple(float(v) for v in at), travel=float(travel))


@on_spawn
def _author(name: str, prim) -> None:
    if not _SPEC or name != _SPEC["card"]:
        return
    from pxr import Gf, Sdf, UsdPhysics

    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("latch"))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(_SPEC["reader"]))])
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*_SPEC["at"]))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(_SPEC["travel"])
    joint.CreateCollisionEnabledAttr(True)
    joint.CreateJointEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")  # a little damping, so the card settles on the catch
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(20.0)
    _PRIM["joint"] = joint
    _ON["on"] = False


def _set(on: bool) -> None:
    if "joint" in _PRIM and _ON["on"] != on:
        # Engaged, the joint alone holds the card in the slot: the card stops colliding with the reader, so the two
        # don't fight over it. Off, they collide again.
        _PRIM["joint"].CreateCollisionEnabledAttr(not on)
        _PRIM["joint"].CreateJointEnabledAttr(bool(on))
    _ON["on"] = bool(on)


def engage() -> None:
    """Switch the catch on (a no-op if it is on). Call it from an ``on_step`` callback."""
    if "joint" not in _PRIM:
        raise KeyError("the latch has not spawned (it exists inside the simulation)")
    _set(True)


def engaged() -> bool:
    """Whether the catch is on."""
    return _ON["on"]


@on_reset
def _release(env=None) -> None:
    _set(False)
