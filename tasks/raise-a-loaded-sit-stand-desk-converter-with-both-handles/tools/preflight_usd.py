"""USD preflight: open the asset with pxr exactly as Isaac Lab composes it (referenced under
/World/envs/env_0/desk_converter_fixture) and check, before any simulator run:

* no two sibling prims share a name, every relationship target exists;
* one articulation root, the expected bodies and joints, masses as declared;
* every joint's two frames coincide in the world at the authored (start) pose, so all joints read 0 at spawn,
  which is also the reset target;
* no two colliders that can touch interpenetrate at the start: bodies of the converter that are not joined to each
  other, the loads, the card, the table top; and the lock geometry over the whole travel (pins clear of teeth when
  squeezed, inside a slot at every setting, never inside a tooth when engaged at a slot).

Run: ``<python with pxr> tools/preflight_usd.py``. Prints a report and exits non-zero on any failure.
"""

import itertools
import math
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rdb_desk_converter import fixed_scene as scene  # noqa: E402
from rdb_desk_converter import layout as L  # noqa: E402

FIXTURE = "/World/envs/env_0/desk_converter_fixture"
FAIL = []


def check(ok, text):
    print(("PASS " if ok else "FAIL ") + text)
    if not ok:
        FAIL.append(text)


def compose():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim("/World/envs", "Xform")
    stage.DefinePrim("/World/envs/env_0", "Xform")
    prim = stage.DefinePrim(FIXTURE, "Xform")
    prim.GetReferences().AddReference(str(scene.USD))
    return stage


def boxes_of(stage, body_path):
    """(name, centre, rotation 3x3, half sizes) of each collider Cube under a body, world frame."""
    cache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(body_path)):
        if prim.GetTypeName() == "Cube" and prim.HasAPI(UsdPhysics.CollisionAPI):
            m = cache.GetLocalToWorldTransform(prim)
            t = np.array(m.ExtractTranslation())
            basis = np.array([[m[i][j] for j in range(3)] for i in range(3)])   # rows = scaled axes
            half = np.linalg.norm(basis, axis=1) * float(prim.GetAttribute("size").Get()) / 2
            rot = (basis / np.linalg.norm(basis, axis=1)[:, None]).T             # columns = unit axes
            out.append((prim.GetName(), t, rot, half))
    return out


def obb_gap(a, b):
    """Separation along the best SAT axis (negative = overlap depth)."""
    _, ca, ra, ha = a
    _, cb, rb, hb = b
    axes = [ra[:, i] for i in range(3)] + [rb[:, i] for i in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i in range(3) for j in range(3)]
    best = -1e9
    d = cb - ca
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        pa = sum(ha[i] * abs(ra[:, i] @ ax) for i in range(3))
        pb = sum(hb[i] * abs(rb[:, i] @ ax) for i in range(3))
        best = max(best, abs(d @ ax) - pa - pb)
    return best


def main():
    stage = compose()
    root = stage.GetPrimAtPath(FIXTURE)
    check(root.IsValid() and root.GetChildren(), f"asset composes under {FIXTURE}")
    # duplicate sibling names / broken relationships
    dup = []
    broken = []
    for prim in Usd.PrimRange(root):
        names = [c.GetName() for c in prim.GetChildren()]
        if len(names) != len(set(names)):
            dup.append(str(prim.GetPath()))
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if not stage.GetPrimAtPath(target.GetPrimPath()).IsValid():
                    broken.append(f"{prim.GetPath()}.{rel.GetName()} -> {target}")
    check(not dup, f"no duplicate sibling prim names {dup[:3]}")
    check(not broken, f"every relationship target exists {broken[:3]}")
    roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check(len(roots) == 1, f"one articulation root: {[str(p.GetPath()) for p in roots]}")
    conv = f"{FIXTURE}/{scene.ROOT_NAME}"
    bodies = {p.GetName(): p for p in stage.GetPrimAtPath(conv + "/Bodies").GetChildren()}
    check(tuple(bodies) == scene.BODIES, f"bodies {tuple(bodies)}")
    joints = {p.GetName(): p for p in stage.GetPrimAtPath(conv + "/Joints").GetChildren()}
    check(set(joints) == {"fix_base", *scene.JOINTS}, f"joints {sorted(joints)}")
    loads = {p.GetName(): p for p in stage.GetPrimAtPath(FIXTURE + "/Loads").GetChildren()}
    check(tuple(loads) == scene.LOADS, f"loads {tuple(loads)}")
    masses = {n: float(p.GetAttribute("physics:mass").Get()) for n, p in {**bodies, **loads}.items()}
    expected = {"base": L.BASE_MASS, "lift_arm": L.ARM_MASS, "rear_arm": L.REAR_ARM_MASS, "top": L.TOP_MASS,
                "latch_left": L.LATCH_MASS, "latch_right": L.LATCH_MASS, "monitor": L.MONITOR_MASS,
                "keyboard": L.KEYBOARD_MASS}
    check(all(abs(masses[k] - v) < 1e-6 for k, v in expected.items()), f"masses {masses}")
    print("converter total", round(sum(masses[k] for k in scene.BODIES), 2), "kg (published 21.04)")

    # joint frames coincide at the authored pose (=> all joints at 0 at spawn = reset target)
    cache = UsdGeom.XformCache()
    for name, prim in joints.items():
        j = UsdPhysics.Joint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        p0 = Gf.Vec3d(j.GetLocalPos0Attr().Get())
        p1 = Gf.Vec3d(j.GetLocalPos1Attr().Get())
        w0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b0[0])).Transform(p0) if b0 else p0
        w1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b1[0])).Transform(p1)
        err = (np.array(w0) - np.array(w1))
        check(np.linalg.norm(err) < 1e-6, f"joint {name}: frames coincide at spawn (error {np.linalg.norm(err):.2e} m)")
        if name != "fix_base":
            mim = prim.GetAttribute("physxMimicJoint:rotX:gearing")
            if mim and mim.HasAuthoredValue():
                ref = prim.GetRelationship("physxMimicJoint:rotX:referenceJoint").GetTargets()
                print(f"     mimic: {name} + {mim.Get()} x {ref[0].name} = 0")
    # spawn poses vs reset targets: top at setting 4, loads on their marks
    top_w = np.array(cache.GetLocalToWorldTransform(bodies["top"]).ExtractTranslation()) - (0, 0, scene.TOP)
    check(np.allclose(top_w, L.top_origin(scene.START_THETA), atol=1e-6), f"top spawned at setting 4 {np.round(top_w, 4)}")
    check(abs(top_w[2] - L.surface_height(L.seated(4))) < 1e-4,
          f"surface at {top_w[2]:.4f} m = setting 4 seated (slot centre {L.surface_height(L.theta(4)):.4f} m)")
    # the seated pose vs the reset check: spawn is the seated pose; the check's window covers the whole slot play
    play_top = float(np.linalg.norm(L.top_origin(L.theta(4) + L.SEAT_PLAY) - L.top_origin(L.seated(4))))
    check(play_top < scene.RESET_TOP_TOL and 2 * L.SEAT_SETTINGS < scene.RESET_SETTING_TOL,
          f"reset window ({1000 * scene.RESET_TOP_TOL:.1f} mm, {scene.RESET_SETTING_TOL:.3f} settings) covers the slot play "
          f"({1000 * play_top:.1f} mm, {2 * L.SEAT_SETTINGS:.3f} settings, seat {L.SEAT_PLAY:.5f} rad)")

    # collider interpenetration at spawn
    groups = {n: boxes_of(stage, str(p.GetPath())) for n, p in {**bodies, **loads}.items()}
    groups["card"] = boxes_of(stage, FIXTURE + "/Marks")
    table = ("table", np.array((0., 0., scene.TOP - .03)), np.eye(3), np.array((2., 2., .03)))
    adjacent = {frozenset(p) for p in (("base", "lift_arm"), ("lift_arm", "top"), ("base", "rear_arm"),
                                       ("top", "latch_left"), ("top", "latch_right"))}
    worst = []
    for a, b in itertools.combinations(groups, 2):
        if frozenset((a, b)) in adjacent:
            continue
        for ba in groups[a]:
            for bb in groups[b]:
                g = obb_gap(ba, bb)
                if g < 0.:
                    worst.append((g, a, ba[0], b, bb[0]))
                    # resting contacts of loads on the top are expected to be >= 0 (they are spawned 0.5 mm up)
    for n, items in groups.items():
        if n in ("base", "card"):
            continue                                   # base rails and the card stand on the table
        for item in items:
            g = obb_gap(item, table)
            if g < 0.:
                worst.append((g, n, item[0], "table", "top"))
    check(not worst, f"no interpenetrating colliders at spawn {sorted(worst)[:4]}")
    load_gaps = {n: min(obb_gap(x, y) for x in groups[n] for y in groups["top"]) for n in loads}
    check(all(0. <= v < .002 for v in load_gaps.values()), f"loads resting just above the top {load_gaps}")

    # lock geometry over the travel (pure geometry from the composed prims, in the arm frame at the start)
    arm_world = np.array(cache.GetLocalToWorldTransform(bodies["lift_arm"]).ExtractTranslation())
    teeth = [b for b in groups["lift_arm"] if b[0].startswith("tooth_")]
    check(len(teeth) == 2 * (L.SETTINGS + 1), f"{len(teeth)} lock teeth")
    pivot_w = arm_world + (L.upper_pivot(scene.START_THETA) - L.PIVOT)
    for side in ("left", "right"):
        pin = [b for b in groups[f"latch_{side}"] if b[0] == "pin"][0]
        my_teeth = [t for t in teeth if t[1][1] * (1 if side == "left" else -1) > 0]
        report = []
        for s in np.arange(L.LIFT_LIMITS[0], L.LIFT_LIMITS[1], math.radians(.25)):
            u = s - scene.START_THETA
            # relative to the arm, the top turns by -u about the upper pivot (the arm turns +u relative to the top)
            rot = L.rot_xz(-u)
            for lift, label in ((0., "engaged"), (L.PADDLE_TRAVEL, "squeezed")):
                c = pivot_w + rot @ (pin[1] + (0, 0, lift) - pivot_w)
                moved = (pin[0], c, rot @ pin[2], pin[3])
                g = min(obb_gap(moved, t) for t in my_teeth)
                report.append((s, label, g))
        squeezed = min(g for s, lab, g in report if lab == "squeezed")
        check(squeezed > .0008, f"{side}: squeezed pin clears every tooth over the travel (min gap {1000 * squeezed:.2f} mm)")
        for k in range(3, L.SETTINGS + 1):
            for angle, label, ok in ((L.theta(k), "centre", lambda g: g > .0008), (L.seated(k), "seated", lambda g: abs(g) < 2e-5)):
                u = angle - scene.START_THETA
                rot = L.rot_xz(-u)
                c = pivot_w + rot @ (pin[1] - pivot_w)
                g = min(obb_gap((pin[0], c, rot @ pin[2], pin[3]), t) for t in my_teeth)
                check(ok(g), f"{side}: engaged pin in slot {k} at its {label} pose: gap to the teeth {1000 * g:.3f} mm")
        blocked = [s for s, lab, g in report if lab == "engaged" and g < 0.]
        frac = len(blocked) / max(1, sum(1 for _, lab, _ in report if lab == "engaged"))
        check(frac > .5, f"{side}: an engaged pin is blocked by a tooth over {100 * frac:.0f}% of the travel")
    print("\n" + ("PREFLIGHT OK" if not FAIL else f"PREFLIGHT FAILED: {len(FAIL)} checks"))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
