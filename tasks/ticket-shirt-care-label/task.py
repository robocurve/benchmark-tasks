"""Ticket a garment through its care label only: lift the label clear of the shirt, push a tagging gun's needle through the label alone, fire one
fastener and draw the needle back.

A child's cotton T-shirt lies flat on the table, front up, and the care label sewn into its side seam has been drawn out onto the front panel (30 x 60 mm,
sewn along its short edge, its free edge curled up a few millimetres). A standard tagging gun (Avery Dennison Mark III class: 125 x 25 x 130 mm, 100 g by a
retailer's listing) lies on its side beside it with a card price ticket on its needle.

Sim (stated, all in code): the shirt is two sewn layers of native surface-FEM cloth (front and back, 0.6 mm, 180 g/m2; gravity, friction and self-contact).
The label is a chain of three thin rigid segments (15, 15 and 30 mm) on sprung hinges, its root a stitch strip held to the shirt's front and back layers by
PhysX's deformable attachment, so it bends, curls, and pulls the shirt up with it. The needle has a 1.9 mm collider on a stiff sprung seat in the gun's
nose; the push along it is read from the seat's give. Pierce rule: at least 4 N held while the tip advances 1 mm into a segment's face; then the collider
stays, filtered against that segment only (no finger point within 1 cm of the collider's tip while it presses), so the shaft passes the hole. Through: the needle on the far side of the label, crossing it within 7 mm of its hole, having got
there through the hole. A hole in the shirt: the needle's tip within 3 mm of the cloth's simulated surface, at any physics step (240 Hz) along its path since the last check, or the shaft crossing the cloth. Torn: the label folded at
its sewn edge past 130 degrees (5 degrees short of the stitches' 135-degree stop), or the needle cut sideways out of its hole. The trigger is a sprung squeeze lever
along the grip's front (25 N preload); a fastener fires when it travels 2.5 mm.

Full score: one fastener fired while the needle was through the label, the shirt never touched by the needle, the needle drawn back out, the label let down
(under 30 degrees) and let go (every fingertip of both hands 5 cm from the label's middle), held 1 s. Partial, from the state at the end, held 1 s: 0.5 a fastener in the
label, or the needle through it now; 0.25 the label's free edge 2 cm or more over the shirt beside it now (lifting the shirt itself does not count). A hole in the shirt (the needle's tip within 3 mm of
its cloth; the shaft lying on it does not count), a torn label (checked at every physics step, 240 Hz) or a gun off the table (5 cm below the tabletop) is 0 for good.

    python demokit.py task.py OUT --mode scene --stills 1
    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Use the tagging gun to fasten the price ticket through this shirt's care label only, not the shirt, then draw the needle out, "
               "lay the label back down and let go.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

# The T-shirt lies flat, front up: two sewn surface-FEM layers (README). x runs from the hem (X_HEM) to the shoulders (X_SHO), y across; the side seam is at y = Y_SIDE.
X_HEM, X_SHO, Y_SIDE, BODY_W = 0.18, 0.48, -0.005, 0.28
SLEEVE_X0, SLEEVE_L = 0.38, 0.09  # the sleeves run out sideways from the shoulder end, 10 cm along x, 9 cm out
NECK = (0.44, 0.095, 0.175)  # the neck opening: x from 0.44 to the shoulder edge, y from 0.095 to 0.175
SHIRT = np.array([0.32, 0.005])  # the mechanism frame's origin: the label's sewn edge, mid-width
BODY_T = float(os.environ.get("SHIRT_TOP", "0.0025"))  # the shirt's settled top face over the table (two layers, measured in the settle test)
SHIRT_SIZE = (X_SHO - X_HEM, BODY_W, BODY_T)
LABEL_W, LABEL_L, LABEL_T = 0.030, 0.060, 0.0003
SEAM = np.array([0.32, 0.005, SHIRT_SIZE[2]])  # the label's sewn edge, along x, on the side seam, drawn out onto the front panel
GUN0 = np.array([0.12, -0.16])  # where the gun lies, its grip's middle
FWD, UP = 0.115, 0.06
NEEDLE_L = 0.025  # the needle's length beyond the gun's nose
HOLD_U = float(os.environ.get("TAG_HOLD_U", "0.048"))  # where the left pads pinch the label, from its sewn edge
PIERCE_BELOW = float(os.environ.get("TAG_PIERCE_BELOW", "0.018"))  # m: pierce this far below the pinch along the label, under the left fingertips
PIERCE_U = HOLD_U - PIERCE_BELOW  # where the demo pierces the label (30 mm out from its sewn edge, under the left fingertips): the fastener is drawn there, at the hole

LOOKS = {"cotton": m.Look(color=m.rgb(0.10, 0.18, 0.36), roughness=0.95), "hem": m.Look(color=m.rgb(0.08, 0.15, 0.31), roughness=0.95),
         "label": m.Look(color=m.rgb(0.94, 0.94, 0.90), roughness=0.45), "print": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.6),
         "gun": m.Look(color=m.rgb(0.82, 0.83, 0.85), roughness=0.5), "grip": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.6),
         "trigger": m.Look(color=m.rgb(0.75, 0.12, 0.10), roughness=0.5), "steel": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=1.0, roughness=0.25),
         "ticket": m.Look(color=m.rgb(0.97, 0.96, 0.92), roughness=0.7), "price": m.Look(color=m.rgb(0.80, 0.10, 0.10), roughness=0.6),
         "fastener": m.Look(color=m.rgb(0.92, 0.92, 0.92), roughness=0.4)}


SHIRT_GSM, SHIRT_THICK = 0.18, 0.0006  # 180 g/m2 cotton jersey, 0.6 mm per layer
SHIRT_YOUNG = float(os.environ.get("SHIRT_YOUNG", "2e6"))  # Pa, effective in-plane (unmeasured: an approximation of knitted cotton)
SHIRT_BEND = float(os.environ.get("SHIRT_BEND", "1.8e5"))
LAYER_GAP = 0.0016
CELL = 0.01


def _tee_cells():
    """The tee's outline on a 1 cm grid: the body, two short sleeves out from the shoulder end, the neck notch cut away."""
    cells = set()
    nx = int(round((X_SHO - X_HEM) / CELL))
    y_lo, y_hi = Y_SIDE - SLEEVE_L, Y_SIDE + BODY_W + SLEEVE_L
    ny = int(round((y_hi - y_lo) / CELL))
    for i in range(nx):
        x = X_HEM + (i + 0.5) * CELL
        for j in range(ny):
            y = y_lo + (j + 0.5) * CELL
            body = Y_SIDE < y < Y_SIDE + BODY_W
            sleeve = x > SLEEVE_X0
            neck = x > NECK[0] and NECK[1] < y < NECK[2]
            if (body or sleeve) and not neck:
                cells.add((i, j))
    return cells, nx, ny, y_lo


def _cloth_mesh():
    """The two-layer tee's points and triangles, as written to the stage (front layer, back layer, then the seam strips). Pure geometry: the scorer uses the
    same triangles to find the shirt's surface from its simulated nodes."""
    from collections import Counter

    cells, nx, ny, y_lo = _tee_cells()
    ids, uv = {}, []

    def vid(i, j):
        if (i, j) not in ids:
            ids[(i, j)] = len(uv)
            uv.append((X_HEM + i * CELL, y_lo + j * CELL))
        return ids[(i, j)]

    tris = []
    for (i, j) in sorted(cells):
        a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
        tris += [(a, b, c), (a, c, d)]
    n = len(uv)
    edges = Counter(tuple(sorted((a, b))) for t in tris for a, b in zip(t, t[1:] + t[:1]))
    directed = {(a, b) for t in tris for a, b in zip(t, t[1:] + t[:1])}
    seams = []
    for (a, b), cnt in edges.items():
        if cnt != 1:
            continue
        mx, my = (uv[a][0] + uv[b][0]) / 2, (uv[a][1] + uv[b][1]) / 2
        hem = mx < X_HEM + 1e-6
        cuff = (my < y_lo + 1e-6) or (my > y_lo + ny * CELL - 1e-6)
        neck = NECK[0] - 1e-6 <= mx and NECK[1] - 1e-6 <= my <= NECK[2] + 1e-6 and not (mx > X_SHO - 1e-6)
        if not (hem or cuff or neck):
            seams.append((a, b))
    z_back = 0.0012
    pts = [(x, y, z_back + LAYER_GAP) for x, y in uv] + [(x, y, z_back) for x, y in uv]  # front layer, then back layer
    faces = list(tris) + [(a + n, c + n, b + n) for a, b, c in tris]
    for a, b in seams:
        if (a, b) not in directed:
            a, b = b, a
        faces += [(b, a, a + n), (b, a + n, b + n)]
    return pts, faces, seams


def _build_cloth(usd_path: str) -> None:
    """Write the shirt: a front and a back layer of a flat tee, each a triangle mesh in Isaac's surface-deformable (FEM) body, sewn along the shoulders, sleeve
    undersides and side seams by narrow strips joining their edges; the neck, cuffs and hem stay open. One body, so the layers collide with each other."""
    from omni.physx.scripts import deformableUtils as du
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdShade

    pts, faces, seams = _cloth_mesh()
    idx = [k for t in faces for k in t]

    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/P_shirt_cloth")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/P_shirt_cloth/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.18, 0.36)])
    ok = du.set_physics_surface_deformable_body(stage, mesh.GetPath())
    prim = mesh.GetPrim()
    print("[shirt] rest attrs: " + ", ".join(f"{a.GetName()}={a.Get() if a.GetName().endswith('Default') else '...'}" for a in prim.GetAttributes() if "rest" in a.GetName().lower() or "bend" in a.GetName().lower()), flush=True)
    if os.environ.get("SHIRT_RESTBEND", "1") == "1":
        # the sewn seams fold the layers 90 degrees at their edges: tell the solver that fold is the unstressed bend, or its default (flat) inflates the edges
        for name, val in (("omniphysics:restBendAnglesDefault", "restShapeDefault"),):
            a = prim.GetAttribute(name)
            print(f"[shirt] {name}: attribute {'present' if a else 'missing'}; set {a.Set(val) if a else False}", flush=True)
    col = PhysxSchema.PhysxCollisionAPI.Apply(prim)  # the deformable default keeps contacts 2 cm off everything (kit section 11)
    col.CreateContactOffsetAttr(float(os.environ.get("SHIRT_CONTACT", "0.002")))
    col.CreateRestOffsetAttr(float(os.environ.get("SHIRT_REST", "0.0006")))
    prim.CreateAttribute("physxDeformableBody:selfCollision", Sdf.ValueTypeNames.Bool).Set(os.environ.get("SHIRT_SELF", "1") == "1")
    if os.environ.get("SHIRT_FILTER", "0.0012"):
        prim.CreateAttribute("physxDeformableBody:selfCollisionFilterDistance", Sdf.ValueTypeNames.Float).Set(float(os.environ.get("SHIRT_FILTER", "0.0012")))
    if os.environ.get("SHIRT_CCD", "0") == "1":  # speculative CCD and frequent collision-pair updates keep thin layers from passing through each other
        prim.CreateAttribute("physxDeformableBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("physxDeformableBody:collisionPairUpdateFrequency", Sdf.ValueTypeNames.Int).Set(int(os.environ.get("SHIRT_PAIRS", "8")))
    iters = os.environ.get("SHIRT_ITERS", "")
    if iters:
        prim.CreateAttribute("physxDeformableBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(int(iters))
    damp = float(os.environ.get("SHIRT_DAMP", "0.5"))
    if damp > 0:
        prim.CreateAttribute("physxDeformableBody:linearDamping", Sdf.ValueTypeNames.Float).Set(damp)
    mat = UsdShade.Material.Define(stage, "/P_shirt_cloth/fabric")
    ok2 = du.add_surface_deformable_material(stage, mat.GetPath(), density=SHIRT_GSM / SHIRT_THICK, static_friction=0.8, dynamic_friction=0.6, youngs_modulus=SHIRT_YOUNG,
                                             poissons_ratio=0.3, surface_thickness=SHIRT_THICK, surface_bend_stiffness=SHIRT_BEND)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    look = UsdShade.Material.Define(stage, "/P_shirt_cloth/look")  # the rendered cloth: navy cotton (the display colour alone renders grey)
    shader = UsdShade.Shader.Define(stage, "/P_shirt_cloth/look/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.07, 0.13, 0.30))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    stage.GetRootLayer().Save()
    print(f"[shirt] built {len(pts)} nodes, {len(faces)} triangles, {len(seams)} seam edges; body {ok}, material {ok2}", flush=True)


CLOTH = m.Part("shirt_cloth", pos=(0.0, 0.0, TOP), size=(0.1, 0.1, 0.1), role="cloth", build=_build_cloth)
_VIEW: list = [None]


def nodes(env):
    """The shirt's simulation nodes now, in the station frame, through PhysX's tensor view; None if the view cannot be made."""
    if _VIEW[0] is None:
        try:
            import omni.physics.tensors as tensors

            sim_view = tensors.create_simulation_view("torch")
            sim_view.set_subspace_roots("/")
            _VIEW[0] = (sim_view, sim_view.create_surface_deformable_body_view(f"{m.ENV_PATH}/shirt_cloth*/mesh"))
        except Exception as e:  # noqa: BLE001
            print(f"[shirt] no view: {e!r}", flush=True)
            _VIEW[0] = (None, None)
    view = _VIEW[0][1]
    if view is None or view.count == 0:
        return None
    p = view.get_simulation_nodal_positions()
    p = p.cpu().numpy() if hasattr(p, "cpu") else np.asarray(p)
    return p.reshape(-1, 3) - env.unwrapped.scene.env_origins[0].cpu().numpy()


def on_shirt(x: float, y: float) -> bool:
    """Whether a point is over the tee's outline (body and sleeves, the neck notch ignored)."""
    return bool(X_HEM <= x <= X_SHO and ((Y_SIDE <= y <= Y_SIDE + BODY_W) or (x >= SLEEVE_X0 and Y_SIDE - SLEEVE_L <= y <= Y_SIDE + BODY_W + SLEEVE_L)))


_FACES: list = [None]
NEEDLE_R = 0.00095  # m: the needle's radius (1.9 mm across)
BODY_CONTACT = 0.003  # m: the needle's tip this close to the cloth's mid-surface touches it (its 0.95 mm radius plus the cloth's 2 mm contact offset)


def _faces():
    """The cloth's triangles as index rows into its simulation nodes, and the node count they assume (the stage's points, in the order written)."""
    if _FACES[0] is None:
        pts, faces, _ = _cloth_mesh()
        _FACES[0] = (len(pts), np.asarray(faces, dtype=np.int64))
    return _FACES[0]


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-18), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def tri_dist(p, a, b, c) -> np.ndarray:
    """The distance from a point p to each triangle (a, b, c) (rows): to its plane where the point projects inside it, else to its nearest edge."""
    p = np.broadcast_to(np.asarray(p, float), a.shape)
    n = np.cross(b - a, c - a)
    nn = np.linalg.norm(n, axis=1)
    nh = n / np.maximum(nn, 1e-18)[:, None]
    h = np.einsum("ij,ij->i", p - a, nh)
    q = p - h[:, None] * nh
    inside = (np.einsum("ij,ij->i", np.cross(b - a, q - a), n) >= 0) & (np.einsum("ij,ij->i", np.cross(c - b, q - b), n) >= 0) & \
             (np.einsum("ij,ij->i", np.cross(a - c, q - c), n) >= 0) & (nn > 1e-12)
    edge = np.minimum(np.minimum(_seg_dist(p, a, b), _seg_dist(p, b, c)), _seg_dist(p, c, a))
    return np.where(inside, np.abs(h), edge)


def needle_to_shirt(tip, axis, cloth, faces=None) -> float:
    """The least distance from the needle's tip to the shirt's simulated surface: the triangles of its two layers and seams, placed at the cloth's nodes now.
    A hole in the shirt is this under BODY_CONTACT (the tip alone; the per-physics-step check with the tip's path and the shaft is shirt_touch)."""
    faces = _faces()[1] if faces is None else faces
    tip, axis = np.asarray(tip, float), np.asarray(axis, float)
    pts = tip[None, :]
    near = np.linalg.norm(cloth - tip, axis=1) < NEEDLE_L + 0.03  # only triangles with a node near the needle
    sel = faces[near[faces].any(axis=1)]
    if not len(sel):
        return 1.0
    a, b, c = cloth[sel[:, 0]], cloth[sel[:, 1]], cloth[sel[:, 2]]
    return float(min(tri_dist(p, a, b, c).min() for p in pts))


NODE_SLACK = CELL * math.sqrt(0.5)  # m: any point of the cloth's surface is within this of one of its nodes (1 cm grid, half a cell's diagonal)


def _along(a, b, step=0.001):
    """Points from a to b, at most `step` apart, both ends included."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, min(400, int(math.ceil(float(np.linalg.norm(b - a)) / step))))
    return a + (b - a) * (np.arange(n + 1) / n)[:, None]


def seg_hits_tri(p, q, a, b, c) -> bool:
    """Whether the segment p-q crosses any of the triangles (a, b, c) (rows): Moller-Trumbore, the segment's ends included."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    e1, e2 = b - a, c - a
    h = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(det) > 1e-14
    f = 1.0 / np.where(ok, det, 1.0)
    s = p - a
    u = f * np.einsum("ij,ij->i", s, h)
    sv = np.cross(s, e1)
    v = f * (sv @ d)
    t = f * np.einsum("ij,ij->i", e2, sv)
    return bool((ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t >= 0) & (t <= 1)).any())


def shirt_touch(prev_tip, tip, prev_base, base, cloth, faces=None) -> float:
    """The needle's least distance to the shirt's simulated surface over one check interval (a physics step, 240 Hz): the tip along its whole path since the last
    check (a point every mm), 0 if the shaft (base to tip), or the path the tip or the base swept, crosses a triangle of the cloth (the needle through it). The shaft
    lying on the cloth without crossing it counts for nothing. `faces` None: no triangles to test against, so the cloth's nodes alone, which are 1 cm apart:
    the distance to the nearest node less NODE_SLACK, so no tip between nodes is missed. `cloth` None: the shirt's outline and settled top face."""
    tip, base = np.asarray(tip, float), np.asarray(base, float)
    pt = tip if prev_tip is None else np.asarray(prev_tip, float)
    pb = base if prev_base is None else np.asarray(prev_base, float)
    pts = _along(pt, tip)
    if cloth is None or not len(cloth):
        hit = any(on_shirt(x, y) and z < TOP + SHIRT_SIZE[2] + 0.004 for x, y, z in pts)
        return 0.0 if hit else 1.0
    if faces is None:
        dn = min(float(np.linalg.norm(cloth - p, axis=1).min()) for p in pts)
        return max(0.0, dn - NODE_SLACK)
    near = (np.linalg.norm(cloth - tip, axis=1) < NEEDLE_L + 0.03) | (np.linalg.norm(cloth - pt, axis=1) < NEEDLE_L + 0.03)
    sel = faces[near[faces].any(axis=1)]
    if not len(sel):
        return 1.0
    a, b, c = cloth[sel[:, 0]], cloth[sel[:, 1]], cloth[sel[:, 2]]
    if any(seg_hits_tri(p, q, a, b, c) for p, q in ((base, tip), (pt, tip), (pb, base))):
        return 0.0
    return float(min(tri_dist(p, a, b, c).min() for p in pts))


# -- the care label: a short chain of thin rigid segments, its sewn edge a strip attached to the shirt's cloth ---------------------------------
SEG_U = (0.0, 0.015, 0.030, 0.060)  # the segments along the label's 60 mm length: two of 15 mm by the seam, where it bends most, and the 30 mm free end (README, size table)
NSEG = len(SEG_U) - 1
K_SEG = float(os.environ.get("TAG_KSEG", "1e-2"))  # N m/rad between segments (estimate: a stiff woven label, bending rigidity about 1.5e-4 N m2 over its 30 mm width; range 3e-3 to 3e-2)
K_SEAM = float(os.environ.get("TAG_KSEAM", "3e-3"))  # N m/rad at the sewn edge, where the label folds most easily against its stitches (estimate; range 1e-3 to 1e-2)
CURL_DEG = 20.0  # the free segment's set curl (estimate): it rests about 12 degrees up under its own weight, its free edge about 4 mm up
LABEL_DROP = float(os.environ.get("TAG_LABEL_DROP", "0.0016"))  # m: the label spawns this much higher, clear of the cloth's 2 mm contact offset, and settles onto the shirt under its own weight
ZL = BODY_T + 0.0010 + 0.00075 + LABEL_DROP  # the label's mid-plane over the table as spawned (its collider's underside 2.3 mm over the cloth), in the label's own frame (origin at the sewn edge, on the table)
LABEL_COL_T = 0.0015  # the label's collider thickness; the real label is about 0.3 mm (README, size table: PhysX needs a thicker box to hold a contact)
STRIP = (LABEL_W + 0.004, 0.018, 0.0018)  # the sewn edge's stitch strip: rigid, attached by PhysX's deformable attachment to the shirt's front-panel nodes under it


def _label() -> m.Part:
    """The care label as a free mechanism: its root is the stitch strip, sewn to the shirt's cloth on the live stage (attach_label), and the label itself
    three thin rigid segments (15, 15 and 30 mm) on sprung hinges about x, so it flexes, curls at its free edge and moves with the shirt."""
    z0 = ZL - LABEL_T / 2  # the label's underside
    frame = [("strip", m.box(STRIP, (0.0, -0.002, BODY_T + 0.0009 + STRIP[2] / 2)), "label")]
    frame_cols = [m.col_box("strip_c", STRIP, (0.0, -0.002, BODY_T + 0.0009 + STRIP[2] / 2))]
    links = []
    for k in range(NSEG):
        u0, sl = SEG_U[k], SEG_U[k + 1] - SEG_U[k]
        parts = [(f"seg{k + 1}", m.box((LABEL_W, sl - 0.0004, LABEL_T), (0.0, u0 + sl / 2, ZL)), "label")]
        parts += [(f"print{k}{j}", m.box((0.020, 0.0015, 0.0002), (0.0, u0 + 0.006 + 0.008 * j, ZL + LABEL_T / 2 + 0.0001)), "print") for j in range(2)]
        cols = [m.col_box(f"seg{k + 1}_c", (LABEL_W, sl - 0.0004, LABEL_COL_T), (0.0, u0 + sl / 2, ZL))]
        if k == NSEG - 1:
            parts.append(("curl", m.box((LABEL_W, 0.004, LABEL_T), (0.0, LABEL_L - 0.0015, ZL + 0.0015)), "label"))
        if k == NSEG - 1:  # once fired: the fastener's T-end through the label, its barb running out past the free edge to the card ticket
            parts += [("tag_fastener", m.box((0.0015, 0.0015, 0.006), (0.0, PIERCE_U, ZL)), "fastener", True),
                      ("tag_barb", m.box((0.0012, 0.040, 0.0012), (0.0, PIERCE_U + 0.020, ZL + 0.0015)), "fastener", True),
                      ("tag_ticket", m.box((0.030, 0.050, 0.0006), (0.0, PIERCE_U + 0.065, ZL + 0.0021)), "ticket", True),
                      ("tag_price", m.box((0.016, 0.010, 0.0003), (0.0, PIERCE_U + 0.060, ZL + 0.0026)), "price", True)]
        links.append(m.hinge(f"seg{k + 1}", "X", (0.0, u0, z0), -60.0 if k else SEAM_RANGE[0], 60.0 if k else SEAM_RANGE[1], parts, cols, mass=LABEL_MASS / NSEG, spring=K_SEG if k else K_SEAM,
                             damping=(K_SEG if k else K_SEAM) / 4, friction=0.0, gravity=True, parent=None if k == 0 else f"seg{k}"))
    return m.mechanism("label", (SEAM[0], SEAM[1], TOP), links, LOOKS, frame_parts=frame, frame_colliders=frame_cols, friction=(0.9, 0.8), free_mass=0.0004,
                       frame_inertia=(2e-8, 2e-8, 4e-8), solver_iters=48)


SEAM_RANGE = (-25.0, 135.0)  # deg: the sewn edge's fold, from the stitches' stop folded under to their stop folded back over the seam (estimate)
TEAR_DEG = SEAM_RANGE[1] - 5.0  # folded to within 5 degrees of the stop: the pull is on the stitches, and the label counts as torn from its seam
LABEL_MASS = 0.002  # a woven satin label 30 x 60 mm (estimate, about 1.1 g/cm3 at 0.3 mm with its print)
_ATTACHED = [False]
ATTACH_REACH = 0.0025  # m: the strip holds the cloth nodes this close to it


def attach_label(env) -> None:
    """Sew the label's stitch strip to the shirt's cloth on the live stage, once (Isaac's attachment helper needs the running stage; kit README 11)."""
    if _ATTACHED[0]:
        return
    import omni.usd
    from omni.physx.scripts import deformableUtils as du
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    base = f"{m.ENV_PATH}/shirt_cloth"
    ok = du.create_auto_deformable_attachment(stage, Sdf.Path(f"{base}/attach_label"), Sdf.Path(f"{base}/mesh"), Sdf.Path(f"{m.ENV_PATH}/label/frame"))
    prim = stage.GetPrimAtPath(f"{base}/attach_label")
    for a in prim.GetAttributes() if prim and prim.IsValid() else []:  # take the cloth nodes within 2.5 mm of the strip: the front and back layers under it, as a label sewn into the seam
        if ("verlap" in a.GetName() or "Offset" in a.GetName()) and str(a.GetTypeName()) == "float":
            a.Set(ATTACH_REACH)
    attrs = {a.GetName(): a.Get() for a in prim.GetAttributes()} if prim and prim.IsValid() else {}
    print(f"[tag] label strip attached to the shirt on the live stage: {ok}; {attrs}", flush=True)
    _ATTACHED[0] = True


m.on_reset(attach_label)


# -- the gun --------------------------------------------------------------------------------------------------------------------------------------
TRIG_TRAVEL, TRIG_FIRE = 0.004, 0.0025  # the trigger's travel back into the grip, and how far it must go to fire (unmeasured: approximations)
TRIG_K, TRIG_PRE = 2000.0, 25.0  # its spring: 25 N preload at rest, 33 N at full travel (unmeasured): a carrying pinch of 20 N does not move it, a 35 N squeeze fires it
K_NEEDLE = 5000.0  # N/m: the needle's seat in the nose, stiff; the push on the needle's tip is read from how far it gives (0.8 mm at 4 N)
F_PIERCE = 4.0  # N: the push along the needle that pierces a woven polyester care label (estimate, range 2 to 6 N, for a 1.9 mm bevelled needle)
PAD_CLEAR = 0.010  # m: no gripper finger point within this of the needle collider's tip while it presses: the push must come from the needle on the label, not from a pad
PIERCE_TRAVEL = 0.001  # m: the tip's advance into the label face while pushing at F_PIERCE or more, before it is through (the label tenting ahead of the point)
RETRACT = -0.040  # the needle seat's lower travel limit only (the collider is never withdrawn: after the pierce it stays, filtered against the pierced segment)


def _gun() -> m.Part:
    """The tagging gun in its own frame: origin at the grip's middle, +y forward along the barrel and needle, +x up from the grip to the barrel, z its thickness.
    It lies on its side, so its frame is the table's. The trigger slides back into the grip against its spring. The needle's collider (1.9 x 1.9 mm) rides on a
    stiff sprung seat along the needle, so its push on the label is read from the seat's give."""
    t = 0.025
    frame = [("barrel", m.box((0.040, 0.120, t), (UP, 0.030, 0.0)), "gun"),
             ("grip", m.box((0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0)), "grip"),
             ("nose", m.cylinder(0.006, 0.010, (UP, 0.095, 0.0), axis="y"), "steel"),
             ("needle", m.cylinder(0.00095, NEEDLE_L, (UP, FWD - NEEDLE_L / 2, 0.0), axis="y"), "steel"),
             ("clip", m.box((0.010, 0.040, t * 0.7), (UP + 0.024, 0.0, 0.0)), "fastener"),
             ("ticket", m.box((0.050, 0.0006, 0.030), (UP - 0.022, 0.104, 0.0)), "ticket"),  # the card ticket on the needle
             ("price", m.box((0.016, 0.0003, 0.010), (UP - 0.030, 0.1035, 0.0)), "price")]
    cols = [m.col_box("barrel_c", (0.040, 0.120, t), (UP, 0.030, 0.0)), m.col_box("grip_c", (0.090, 0.030, t * 0.9), (0.0, -0.012, 0.0))]
    trig = m.box((0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))  # a squeeze lever along the grip's whole front: a finger on the front always bears on it
    trigger = m.slider("trigger", "Y", -TRIG_TRAVEL, 0.0, [("trigger", trig, "trigger")], [m.col_box("trigger_c", (0.080, 0.010, t * 0.9), (0.0, 0.012, 0.0))],
                       mass=0.004, spring=TRIG_K, damping=2.0, friction=0.0, effort=60.0)
    needle = m.slider("needle", "Y", RETRACT - 0.005, 0.0005, [], [m.col_box("needle_c", (0.0019, NEEDLE_L - 0.002, 0.0019), (UP, FWD - NEEDLE_L / 2 + 0.001, 0.0))],
                      at=(UP, FWD - NEEDLE_L / 2, 0.0), mass=0.003, spring=K_NEEDLE, damping=15.0, friction=0.0, effort=400.0)
    return m.mechanism("gun", (GUN0[0], GUN0[1], TOP + t / 2 + 0.0005), [trigger, needle], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.10,
                       friction=(0.8, 0.7), frame_inertia=(1.6e-4, 0.6e-4, 2.0e-4))


OBJECTS = [CLOTH, _label(), _gun()]


def _rotx(a: float) -> np.ndarray:
    c, sn = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -sn], [0.0, sn, c]])


def bodies(env, part: str) -> dict:
    """Each body of a mechanism now, in the station frame: name -> (position, rotation matrix)."""
    base = env.unwrapped
    asset = base.scene[part]
    origin = base.scene.env_origins[0].cpu().numpy()
    pose = asset.data.body_link_pose_w[0].detach().cpu().numpy().astype(np.float64)
    return {n: (pose[i, :3] - origin, m.quat_matrix(pose[i, 3:7])) for i, n in enumerate(asset.body_names)}


def label_frames(env):
    """For each segment k: (the point on its underside at u = 0 of the label's own coordinates, its rotation). A point (x, u) on the label's underside is
    then hinge + R @ (x, u, 0) for the segment that holds u. Also the sewn edge's point and the label's chord angle from the shirt (seam to free edge)."""
    b = bodies(env, "label")
    segs = []
    for k in range(NSEG):
        t, R = b[f"seg{k + 1}"]
        segs.append((t + R @ np.array([0.0, 0.0, ZL - LABEL_T / 2]), R))
    t0, R0 = b["frame"]
    seam = t0 + R0 @ np.array([0.0, 0.0, ZL - LABEL_T / 2])
    edge = label_point(segs, LABEL_L)
    v = edge - seam  # the chord in the station frame: the shirt lies flat, the label's free edge toward +y at rest
    return segs, seam, R0, math.atan2(v[2], v[1])


def label_point(segs, u: float, x: float = 0.0) -> np.ndarray:
    k = min(int(np.searchsorted(SEG_U, u, side="right")) - 1, NSEG - 1)
    k = max(k, 0)
    h, R = segs[k]
    return h + R @ np.array([x, u, 0.0])


def needle_tip(env):
    g = m.state(env, "gun")
    return g.to_world(np.array([[UP, FWD, 0.0]]))[0], g


def let_go(tips, mid, a) -> bool:
    """The label let down under 30 degrees and let go: every fingertip of both hands 5 cm or more from the label's middle."""
    return bool(np.min(np.linalg.norm(np.asarray(tips) - mid, axis=1)) > 0.05 and a < math.radians(30.0))


def needle_through(tip, base_pt, hinge, Rl, top, u0: float = 0.0, u1: float = LABEL_L):
    """Whether the needle is through a label segment now: its tip and its base on opposite sides of the segment's face, crossing it inside its outline
    (u from u0 to u1, x within half the width). Returns that and the crossing's height over the shirt (`top`), for the log: the shirt is kept clear by
    the needle's contact with its surface (needle_to_shirt), not by a height."""
    rt, rb = Rl.T @ (np.asarray(tip) - hinge), Rl.T @ (np.asarray(base_pt) - hinge)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    if ht * hb >= 0.0:
        return False, 0.0
    c = rb + (rt - rb) * (hb / (hb - ht))  # where the needle crosses the segment's plane
    clear = float((hinge + Rl @ np.array([0.0, c[1], 0.0]))[2] - top)
    return bool(u0 < c[1] < u1 and abs(c[0]) < LABEL_W / 2), clear


EDGE_R = 0.02  # m: the shirt beside the label's free edge is read within this radius of it
HOLE_TEAR = 0.007  # m: the needle's crossing of the label this far from the hole it pierced, while through it, has cut the fabric sideways: torn (estimate: a 1.9 mm needle, the weave's stretch round its hole)


def hole_step(st, tip, base_pt, segs) -> tuple:
    """The needle in the hole it pierced, one step. `st` holds the hole ('k', its segment; 'x', 'u', where on it; 'side', the sign of the side the needle came
    from) and 'entered'. Through: the tip on the far side of the segment's face, the needle crossing the face within HOLE_TEAR of the hole, having got to the
    far side through the hole (crossing the face there, or at the pierce itself). Returns (through, torn, the crossing's distance from the hole)."""
    h, R = segs[st["k"]]
    rt, rb = R.T @ (np.asarray(tip) - h), R.T @ (np.asarray(base_pt) - h)
    ht, hb = rt[2] - LABEL_T / 2, rb[2] - LABEL_T / 2
    far = ht * st["side"] < 0.0
    d = 1.0
    if far and hb * st["side"] > 0.0:
        c = rb + (rt - rb) * (hb / (hb - ht))
        d = float(math.hypot(c[0] - st["x"], c[1] - st["u"]))
    torn = False
    if not far:
        st["entered"] = False  # drawn back to the side it came from (or not yet through)
    elif not st["entered"] and not st.get("far", False) and d <= HOLE_TEAR:
        st["entered"] = True  # crossed the face at the hole this step
    elif st["entered"] and d > HOLE_TEAR:
        st["entered"], torn = False, True  # the crossing left the hole with the needle still through: cut sideways
    st["far"] = far
    return bool(st["entered"] and far and d <= HOLE_TEAR), torn, d


def press_step(st, force, ctip, axis, segs):
    """The pierce rule on the needle's real push, one step. `st`: dict with 'seg' (the segment pressed, or None), 's0' and 'a0' (the tip and the needle's
    axis when the push first reached F_PIERCE), 'adv' (the tip's advance since, along a0). The collider's tip `ctip` must bear on a segment's face inside its outline, the needle within 45 degrees of
    the face's normal, at F_PIERCE or more; once it has advanced PIERCE_TRAVEL along the needle at that force, the segment is pierced. Returns its index or None."""
    hit = None
    for k, (h, R) in enumerate(segs):
        r = R.T @ (np.asarray(ctip) - h)
        if SEG_U[k] - 0.001 < r[1] < SEG_U[k + 1] + 0.001 and abs(r[0]) < LABEL_W / 2 and abs(r[2] - LABEL_T / 2) < 0.004 and abs(float(R[:, 2] @ axis)) > math.cos(math.radians(45.0)):
            hit = k
            break
    if hit is None or force < F_PIERCE or (st.get("seg") is not None and st["seg"] != hit):
        st.update(seg=None, s0=None, a0=None, adv=0.0)
        return None
    if st.get("s0") is None:  # the tip and the needle's axis when the push first reached F_PIERCE: the advance is read along that axis from that point
        st.update(seg=hit, s0=np.asarray(ctip, float).copy(), a0=np.asarray(axis, float).copy(), adv=0.0)
    st["adv"] = float((np.asarray(ctip, float) - st["s0"]) @ st["a0"])
    return hit if st["adv"] >= PIERCE_TRAVEL else None


def free_edge_up(edge, cloth, top) -> float:
    """The label's free edge over the shirt beside it: over the highest cloth node within EDGE_R of the edge across the table (else over `top`), so lifting
    the shirt itself, label and all, lifts that cloth too and counts for nothing."""
    beside = top
    if cloth is not None and len(cloth):
        sel = np.linalg.norm(cloth[:, :2] - np.asarray(edge)[:2], axis=1) < EDGE_R
        if sel.any():
            beside = float(cloth[sel, 2].max()) + SHIRT_THICK
    return float(edge[2] - beside)


class Tag:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        was = getattr(self, "filtered", None)
        self.steps, self.pierced, self.torn, self.through, self.fired, self.fastened, self.lifted, self.gun_fell = 0, False, False, False, False, False, 0.0, False
        self.entered, self.prev_rt, self._prev, self.punct, self.press, self.fmax, self.hole = False, None, None, None, {}, 0.0, None
        self.hold, self.full, self.last, self.shown, self.tiers, self.faces_ok = 0, False, {}, False, [], None
        self.p_tip, self.p_base, self.shirt_d, self.filtered = None, None, 1.0, None
        if env is None:  # at import, before the simulator runs: the parts start hidden by themselves (touching the stage here breaks the app's start)
            return
        self._hook(env)
        if was is not None:
            try:
                self._filter(env, was, False)
            except Exception:  # noqa: BLE001
                pass
        try:
            m.set_target(env, "gun", "needle", 0.0)
        except Exception:
            pass
        for path, vis in [(f"{m.ENV_PATH}/label/seg{NSEG}/visual/{nm}", False) for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price")] + \
                         [(f"{m.ENV_PATH}/gun/frame/visual/{nm}", True) for nm in ("ticket", "price")]:
            try:  # the ticket on the gun's needle, nothing on the label
                m.set_visible(path, vis)
            except Exception:
                pass

    def _hook(self, env):
        """Check the hole at every physics step (240 Hz), not only at each control step: wrap the scene's per-physics-step update once."""
        scene = env.unwrapped.scene
        if getattr(scene, "_tag_wrapped", False):
            return
        orig = scene.update
        scene._tag_wrapped = True

        def update(*a, **k):
            r = orig(*a, **k)
            try:
                self.substep(env)
            except Exception:  # noqa: BLE001
                pass
            return r

        scene.update = update

    def substep(self, env):
        """One physics step: the needle in the hole it pierced; a crossing more than HOLE_TEAR from it while through is a tear, however briefly."""
        if not self.pierced:
            self._shirt_check(env)
        if math.degrees(m.state(env, "label").joint("seg1")) > TEAR_DEG:
            self.torn = True  # folded back to the stop of its sewn edge, however briefly
        if self.punct is None or self.hole is None:
            return
        segs = label_frames(env)[0]
        tip, g = needle_tip(env)
        axis = g.rotation() @ np.array([0.0, 1.0, 0.0])
        through, ripped, _ = hole_step(self.hole, tip, tip - NEEDLE_L * axis, segs)
        if ripped:
            self.torn = True
        self._fire(g, through)

    def _fire(self, g, in_label) -> None:
        """The trigger read at one physics step: only the first squeeze counts; a fastener through the label alone if the needle is through it now."""
        if -float(g.joint("trigger")) >= TRIG_FIRE and not self.fired:
            self.fired = True
            self.fastened = bool(in_label and not self.pierced)

    def _shirt_check(self, env, cloth=None):
        """The shirt check at one physics step (and at each control step): the needle's tip along its path since the last check, and its shaft, against the cloth
        (shirt_touch). A hole in the shirt is a distance under BODY_CONTACT, or the shaft crossing the cloth, at any of these checks: latched."""
        tip, g = needle_tip(env)
        base = tip - NEEDLE_L * (g.rotation() @ np.array([0.0, 1.0, 0.0]))
        cloth = nodes(env) if cloth is None else cloth
        n_pts, faces = _faces()
        if self.faces_ok is None and cloth is not None:  # the simulated nodes must be the stage's points in their order: every triangle edge near its 1 cm
            ok = len(cloth) == n_pts
            if ok:
                e = np.linalg.norm(cloth[faces[:, 0]] - cloth[faces[:, 1]], axis=1)
                ok = bool(np.nanmax(e) < 0.03)
            self.faces_ok = ok
            print(f"[shirt] surface triangles on the simulated nodes: {ok} ({len(cloth)} nodes, {n_pts} written)", flush=True)
        d = shirt_touch(self.p_tip, tip, self.p_base, base, cloth, faces if self.faces_ok else None)
        self.p_tip, self.p_base, self.shirt_d = tip, base, d
        if d < BODY_CONTACT:
            self.pierced = True
        return d, cloth

    def _filter(self, env, k, on):
        """Needle shaft against the pierced label segment: no collision (the shaft passes the hole it made), every other pair unchanged (the shirt, the other
        segments, the gun's body). Set on the live stage with PhysX's filtered-pairs relationship, both ways; undone at reset."""
        import omni.usd
        from pxr import Sdf, Usd, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        paths = {}
        for pr in Usd.PrimRange(stage.GetPrimAtPath(m.ENV_PATH)):
            if pr.GetName() in ("needle_c", f"seg{k + 1}_c"):
                paths[pr.GetName()] = pr.GetPath()
        a, b = paths.get("needle_c"), paths.get(f"seg{k + 1}_c")
        if a is None or b is None:
            print(f"[tag] filter: colliders not found {sorted(paths)}", flush=True)
            return
        for x, y in ((a, b), (b, a)):
            rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(x)).CreateFilteredPairsRel()
            (rel.AddTarget if on else rel.RemoveTarget)(Sdf.Path(y))
        self.filtered = k if on else None
        print(f"[tag] needle shaft filtered against label segment {k + 1}: {on}", flush=True)

    def step(self, env):
        self.steps += 1
        m.set_target(env, "gun", "trigger", TRIG_PRE / TRIG_K)  # the spring's rest beyond the stop: its preload
        m.set_target(env, "label", f"seg{NSEG}", math.radians(CURL_DEG))  # the free segment's set curl
        segs, seam, R0, a = label_frames(env)
        tip, g = needle_tip(env)
        axis = g.rotation() @ np.array([0.0, 1.0, 0.0])
        top = TOP + SHIRT_SIZE[2] + 0.001
        cloth = nodes(env)
        near_cloth = self.shirt_d if self.pierced else self._shirt_check(env, cloth)[0]  # a hole in the shirt: checked at 240 Hz in substep, again here
        seam_deg = math.degrees(m.state(env, "label").joint("seg1"))
        if seam_deg > TEAR_DEG:
            self.torn = True  # folded back to the stop of its sewn edge (also checked at every physics step in substep)
        edge = label_point(segs, LABEL_L)
        edge_up = free_edge_up(edge, cloth, top)
        self.lifted = max(self.lifted, edge_up)
        # the pierce rule: the needle's push, read from its seat's give, along the needle into a segment's face
        q_n = float(g.joint("needle"))
        force = 0.0 if self.punct is not None else max(0.0, -q_n) * K_NEEDLE
        self.fmax = max(self.fmax, force)
        if self.punct is None:
            ctip = tip + q_n * axis
            pad = float(np.min(np.linalg.norm(m.fingertips(env) - ctip, axis=1)))
            if pad < PAD_CLEAR:
                self.press.update(seg=None, s0=None, a0=None, adv=0.0)  # a finger is at the press point: the 4 N is not the needle's push on the label
                k = None
            else:
                k = press_step(self.press, force, ctip, axis, segs)
            if k is not None:
                self.punct = k
                h, R = segs[k]
                r = R.T @ (ctip - h)
                rb = R.T @ (tip - NEEDLE_L * axis - h)
                # the hole, where the collider's tip bears on the face; the needle came from its base's side; at the pierce the tip is in the hole
                self.hole = {"k": k, "x": float(r[0]), "u": float(r[1]), "side": 1.0 if rb[2] - LABEL_T / 2 > 0 else -1.0, "entered": True, "far": True}
                self._filter(env, k, True)  # the label is pierced: the shaft passes the hole it made, its collider stays for everything else
        base_pt = tip - NEEDLE_L * axis
        in_label, clear_cm = False, 0.0
        hole_mm = None
        if self.punct is not None:
            h, R = segs[self.punct]
            under = top  # the shirt's top under the needle (for the log): its highest node within 15 mm of the tip across the table
            if cloth is not None and len(cloth):
                sel = np.linalg.norm(cloth[:, :2] - tip[:2], axis=1) < 0.015
                if sel.any():
                    under = max(top, float(cloth[sel, 2].max()) + SHIRT_THICK)
            clear_cm = needle_through(tip, base_pt, h, R, under, -1.0, 1.0)[1]
            in_label, ripped, dh = hole_step(self.hole, tip, base_pt, segs)
            hole_mm = round(dh * 1000, 1) if dh < 1.0 else None
            if ripped:
                self.torn = True  # the needle dragged sideways out of its hole through the label's fabric
        self.through = self.through or in_label
        travel = -float(g.joint("trigger"))
        self._fire(g, in_label)  # also read at every physics step in substep
        if g.pos[2] < TOP - 0.05:
            self.gun_fell = True  # off the table
        mid = label_point(segs, LABEL_L / 2)
        out = bool(not in_label and np.linalg.norm(tip - mid) > 0.04)  # the needle drawn back out of the label
        clear = let_go(m.fingertips(env), mid, a)
        if self.shown != self.fastened:
            self.shown = self.fastened
            for nm in ("tag_fastener", "tag_barb", "tag_ticket", "tag_price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/label/seg{NSEG}/visual/{nm}", self.fastened)
                except Exception:
                    pass
            for nm in ("ticket", "price"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/gun/frame/visual/{nm}", not self.fastened)
                except Exception:
                    pass
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "label_deg": round(math.degrees(a), 1), "seam_deg": round(seam_deg, 1), "lifted_mm": round(self.lifted * 1000, 1), "edge_mm": round(edge_up * 1000, 1),
                     "push_N": round(force, 2), "push_max_N": round(self.fmax, 2), "press_adv_mm": round(1000 * float(self.press.get("adv") or 0.0), 2),
                     "pierced_seg": self.punct, "hole_mm": hole_mm, "q_needle_mm": round(q_n * 1000, 3),
                     "tip_in_free_seg_mm": [round(float(v) * 1000, 1) for v in segs[NSEG - 1][1].T @ (tip - segs[NSEG - 1][0])], "clear_of_body_mm": round(clear_cm * 1000, 1), "seam_up_mm": round(float(seam[2] - top) * 1000, 1),
                     "through": self.through, "in_label": bool(in_label), "trigger_mm": round(travel * 1000, 2), "fired": self.fired, "fastened": self.fastened,
                     "pierced": self.pierced, "torn": self.torn, "gun_fell": self.gun_fell, "out": out, "clear": clear,
                     "tip_above_body_mm": round((tip[2] - top) * 1000, 1), "tip_to_cloth_mm": round(near_cloth * 1000, 1)}
        tier = _tier(self.last)
        self.tiers = (self.tiers + [tier])[-HOLD_STEPS:]
        self.hold = self.hold + 1 if tier == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0 and os.environ.get("SHIRT_LOG", "1") == "1" and cloth is not None:
            spd = 0.0 if self._prev is None or self._prev.shape != cloth.shape else float(np.linalg.norm(cloth - self._prev, axis=1).max() / (10 * float(env.unwrapped.step_dt)))
            self._prev = cloth.copy()
            z = (cloth[:, 2] - TOP) * 1000
            print(f"[shirt] t {self.last['t']} nodes {len(cloth)} nonfinite {int((~np.isfinite(cloth)).any(axis=1).sum())} z {np.nanmin(z):.1f}..{np.nanmax(z):.1f} mm "
                  f"max node speed {spd:.2f} m/s", flush=True)
        if self.steps % 10 == 0 or (self.press.get("s0") is not None):
            print("[tag] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    """The tier of one step's state. Failures (pierced, torn, gun dropped) latch; every tier above them is read from the state now: a fastener in the label
    (fired while the needle was through it) stays there, the needle through the label and the label lifted do not."""
    if not r or r["pierced"] or r["torn"] or r["gun_fell"]:
        return 0.0
    if r["fastened"] and r["out"] and r["clear"]:
        return 1.0
    if r["fastened"] or r["in_label"]:
        return 0.5
    if r["edge_mm"] >= 20.0:
        return 0.25
    return 0.0


TAG = Tag()
m.on_step(TAG.step)
m.on_reset(TAG.reset)


def score(objects) -> float:
    """The lowest tier over the last HOLD_STEPS steps (1 s): the state at the end, held."""
    return float(min(TAG.tiers)) if TAG.tiers else 0.0


def success(objects) -> bool:
    """True while the score is 1.0 now: the state held for 1 s at full, read from the last HOLD_STEPS steps, not a latch."""
    return bool(len(TAG.tiers) >= HOLD_STEPS and min(TAG.tiers) == 1.0)


def size_check() -> list:
    """The scene's key sizes against the real ones (README, size table): (what, in the scene, real, source)."""
    rows = [("shirt body, hem to shoulder x across (mm)", f"{(X_SHO - X_HEM) * 1000:.0f} x {BODY_W * 1000:.0f}", "300 x 280", "a child's tee, about age 4 to 6 (estimate from makers' size charts)"),
            ("jersey, per layer", f"{SHIRT_GSM * 1000:.0f} g/m2, {SHIRT_THICK * 1000:.1f} mm", "150 to 200 g/m2, 0.5 to 0.7 mm", "single-knit cotton jersey (estimate)"),
            ("care label (mm)", f"{LABEL_W * 1000:.0f} x {LABEL_L * 1000:.0f}, {LABEL_MASS * 1000:.0f} g", "25 to 40 x 50 to 80", "woven satin care label (estimate)"),
            ("label as simulated: segments along its length (mm)", " + ".join(f"{(SEG_U[k + 1] - SEG_U[k]) * 1000:.0f}" for k in range(NSEG)), "continuous", "departure: three rigid segments on sprung hinges"),
            ("tagging gun (mm, g)", "125 x 25 x 130, 100", "about 125 x 25 x 130, about 100", "standard tagging gun, retailer's listing (estimate)"),
            ("needle: length beyond the nose, across (mm)", f"{NEEDLE_L * 1000:.0f}, {2 * NEEDLE_R * 1000:.1f}", "about 25, about 2", "standard needle (estimate)"),
            ("push to pierce the label (N)", f"{F_PIERCE:.0f} over {PIERCE_TRAVEL * 1000:.0f} mm", "2 to 6", "estimate for a 2 mm bevelled needle through woven polyester"),
            ("trigger preload, travel to fire", f"{TRIG_PRE:.0f} N, {TRIG_FIRE * 1000:.1f} of {TRIG_TRAVEL * 1000:.0f} mm", "not published", "estimate")]
    for r in rows:
        print(f"[size] {r[0]}: scene {r[1]}; real {r[2]} ({r[3]})")
    return rows


# -- the scripted demo ----------------------------------------------------------------------------

G_DOWN = 0.0  # the needle level as it goes in
LIFT_DEG = float(os.environ.get("TAG_LIFT", "75"))  # the label swung up to this about its seam, pinched
PUSH_S = float(os.environ.get("TAG_PUSH_S", "6.0"))  # s: the push's limit; the tip stops on the label's face until the push reaches the pierce force
PUSH_GAIN = float(os.environ.get("TAG_PUSH_GAIN", "0.3"))  # the push's feedback gain on the tip's error: a stopped tip winds the command ahead, and the arm's joint springs turn that lead into force
PUSH_PAST = float(os.environ.get("TAG_PUSH_PAST", "0.006"))  # how far past the label's face the push aims the needle's tip (it stops at the face until pierced)
RAISE = float(os.environ.get("TAG_RAISE", "0.015"))  # m: the pinch raised straight up after the swing
SPIN = float(os.environ.get("TAG_SPIN", "0.0"))  # deg: the left hand turned about the label's face normal through its pinch, so its wrist and forearm swing clear of the right arm's
RELAX = float(os.environ.get("TAG_RELAX", "0.0"))  # m: how far the pinch comes back toward the seam after the lift
R_TILT = float(os.environ.get("TAG_RTILT", "40"))  # deg: the right hand leans back toward the robot as it holds the gun (fingers still across the grip), so its forearm stays clear of the left's


DBG = os.environ.get("TAG_DBG", "0") == "1"  # debug: log both arms' joint commands against their positions, and what the gun's body overlaps, each step
_GUN_BOXES = (("barrel", (0.040, 0.120, 0.025), (UP, 0.030, 0.0)), ("grip", (0.090, 0.030, 0.0225), (0.0, -0.012, 0.0)),
              ("nose", (0.012, 0.010, 0.012), (UP, 0.095, 0.0)), ("needle", (0.0019, NEEDLE_L, 0.0019), (UP, FWD - NEEDLE_L / 2, 0.0)))


def _overlaps(env, g) -> dict:
    """What each of the gun's boxes, 1 mm larger all round, overlaps now (scene query), other than the gun itself."""
    try:
        import carb
        from omni.physx import get_physx_scene_query_interface

        sq = get_physx_scene_query_interface()
        origin = env.unwrapped.scene.env_origins[0].cpu().numpy()
        R = g.rotation()
        q = g.quat_xyzw
        out = {}
        for nm, size, c in _GUN_BOXES:
            p = R @ np.asarray(c) + g.pos + origin
            hits = []

            def rep(hit, hits=hits):
                path = str(getattr(hit, "rigid_body", "") or getattr(hit, "collision", ""))
                if "/gun" not in path:
                    hits.append(path.replace(m.ENV_PATH + "/", ""))
                return True

            sq.overlap_box(carb.Float3(*[s / 2 + 0.001 for s in size]), carb.Float3(*[float(v) for v in p]), carb.Float4(*[float(v) for v in q]), rep, False)
            if hits:
                out[nm] = sorted(set(hits))
        return out
    except Exception as e:  # noqa: BLE001
        return {"error": repr(e)[:120]}


def _dbg(ctx, env, label):
    """One debug line: each arm's commanded joints, observed joints and their difference (deg), the TCPs, the gun in the right hand, the tip, overlaps."""
    from yam_arena import ik

    tip, g = needle_tip(env)
    row = {"t": TAG.last.get("t"), "label": label, **{k: TAG.last.get(k) for k in ("label_deg", "edge_mm", "seam_up_mm", "push_N", "tip_in_free_seg_mm")}}
    for side, A in (("L", ctx.left), ("R", ctx.right)):
        qo = np.asarray(ctx.observed("left" if side == "L" else "right")[:6], dtype=float)
        row[f"{side}_dq_deg"] = [round(float(v), 1) for v in np.degrees(A.q - qo)]
        row[f"{side}_tcp_cmd_mm"] = [round(float(v) * 1000, 1) for v in ik.tcp_pose(A.q, A.base)[0]]
        row[f"{side}_tcp_obs_mm"] = [round(float(v) * 1000, 1) for v in A.tcp()]
    row["R_q_obs_deg"] = [round(float(v), 1) for v in np.degrees(np.asarray(ctx.observed("right")[:6], dtype=float))]
    hand = ctx.right.grip.rot
    row["gun_in_hand_mm"] = [round(float(v) * 1000, 1) for v in hand.T @ (g.pos - ctx.right.tcp())]
    row["gun_turn_in_hand_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(hand.T @ g.rotation()) - 1) / 2, -1, 1)))), 1)
    row["tip_mm"] = [round(float(v) * 1000, 1) for v in tip]
    row["overlaps"] = _overlaps(env, g)
    print("[dbg] " + json.dumps(row), flush=True)


def _logged(gen, ctx, env, label):
    for _ in gen:
        if DBG:
            _dbg(ctx, env, label)
        yield


def demo_script(ctx):
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "scene":
        yield from L.wait(float(os.environ.get("SHIRT_SETTLE_S", "4.0")))
        segs, seam, R0, a = label_frames(env)
        print(f"[tag] settled label: chord {math.degrees(a):.1f} deg, seam {1000 * (seam[2] - TOP):.1f} mm up, free edge {1000 * (label_point(segs, LABEL_L)[2] - TOP):.1f} mm up", flush=True)
        print("[tag] END " + json.dumps({"mode": mode, **TAG.last}), flush=True)
        return
    if mode == "shirtlift":  # failure check: the shirt lifted by its body beside the label, the label left on it (the label's own lift is what counts)
        gl0 = d.tilted(60.0, -90.0)
        gl = d._as_grip(gl0.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2))
        back = -L.approach_axis(gl)
        P = np.array([float(os.environ.get("TAG_SL_X", SEAM[0])), float(os.environ.get("TAG_SL_Y", SEAM[1] + 0.085)), TOP + float(os.environ.get("TAG_SL_Z", "0.0"))])

        def top_up():  # the shirt's highest cloth node over its settled top face now, mm
            c = nodes(env)
            return None if c is None else round(float((c[:, 2].max() - (TOP + SHIRT_SIZE[2])) * 1000), 1)

        yield from L.move(P + 0.06 * back + np.array([0.0, 0.0, 0.02]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="beside the label, behind the shirt's fabric")
        yield from L.move(P + 0.03 * back, t=0.8, label="jaws either side of the cloth")
        yield from L.move(P, t=0.8, label="pads on the cloth beside the label")
        bunch = float(os.environ.get("TAG_SL_BUNCH", "0.0"))
        if bunch > 0:  # the cloth lies flat and cannot be pinched: close the pads on it and drag them along the table so it gathers into a fold, then pinch the fold
            yield from L.close(force=30.0, t=1.0, label="pads closed on the cloth")
            yield from L.move(L.at() + bunch * np.array([float(os.environ.get("TAG_SL_DX", "0")), float(os.environ.get("TAG_SL_DY", "-1")), 0.0]), t=2.0, label="drag the cloth into a fold")
            yield from L.move(L.at(), finger=d.FINGER_OPEN, t=1.0, label="open the pads")
            yield from L.move(L.at() + float(os.environ.get("TAG_SL_BACK", "0.01")) * np.array([0.0, 1.0, 0.0]), t=1.0, label="the pads over the fold")
            print("[tag] fold: top node " + str(top_up()) + " mm", flush=True)
        yield from L.close(force=40.0, t=2.0, label="pinch the shirt")
        print("[tag] pinched: top node " + str(top_up()) + " mm " + json.dumps(TAG.last), flush=True)
        peak = 0.0
        for _ in L.move(L.at() + np.array([0.0, 0.0, 0.06]), t=2.0, label="lift the shirt 6 cm, the label with it"):
            yield
            peak = max(peak, top_up() or 0.0)
        for _ in L.wait(2.0):
            yield
            peak = max(peak, top_up() or 0.0)
        print(f"[tag] shirtlift top node peak {peak} mm (grasp counts when 30 or more)", flush=True)
        print("[tag] END " + json.dumps({"mode": mode, **TAG.last}), flush=True)
        return
    nolift = mode == "nolift"  # failure check: the label left lying on the shirt, the needle pushed at it as in the demo
    segs, hinge, R0, a_rest = label_frames(env)

    gl0 = d.tilted(60.0, -90.0)
    gl = d._as_grip(gl0.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2))
    back = -L.approach_axis(gl)
    P = label_point(segs, HOLD_U) + np.array([0.0, 0.0, 0.001])
    if not nolift:
        yield from L.move(P + 0.06 * back + np.array([0.0, 0.0, 0.02]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="behind the label's curled free edge")
        yield from L.move(P + 0.03 * back, t=0.8, label="jaws either side of the free edge")
        yield from L.move(label_point(label_frames(env)[0], HOLD_U) + np.array([0.0, 0.0, 0.001]), t=0.8, label="pads 12 mm in from the edge")
        yield from L.close(force=30.0, label="pinch the label")
        yield from L.arc(hinge, (1.0, 0.0, 0.0), LIFT_DEG - math.degrees(a_rest), t=2.0, label=f"swing the label up about its seam to {LIFT_DEG:.0f} degrees")
        if RAISE > 0:  # raise the pinch straight up: the label stands higher, and the shirt rises a little at the seam with it
            yield from L.move(L.at() + np.array([0.0, 0.0, RAISE]), t=0.8, label="raise the label")
        if SPIN:  # the pads stay on the label; the left forearm leaves the space the right forearm needs for the push
            nrm = label_frames(env)[0][NSEG - 1][1][:, 2]
            yield from L.arc(L.at(), nrm, SPIN, t=1.0, label="turn the left hand about the label's face, its forearm clear of the right")
        if RELAX > 0:  # ease the pull on the seam: the pinch comes back toward the seam, so the shirt, raised by the label, settles and the pinch is not pulled on
            to_seam = label_frames(env)[1] - L.at()
            yield from L.move(L.at() + RELAX * to_seam / np.linalg.norm(to_seam), t=0.8, label="ease the pull on the seam")
    print("[tag] lifted: " + json.dumps(TAG.last), flush=True)
    g0 = d.tilted(R_TILT, 0.0) if R_TILT else d.down(0.0)
    grip = m.state(env, "gun").pos
    yield from R.move(grip + np.array([0.0, 0.0, 0.07]), grip=g0, finger=d.FINGER_OPEN, t=2.0, label="over the gun's grip")
    yield from R.move(grip, t=1.0, label="fingers either side of the grip and trigger")
    yield from R.close(0.044, force=20.0, label="hold the gun at 20 N, under the trigger's 25 N preload")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift the gun")
    tip, _ = needle_tip(env)
    in_hand = R.grip.rot.T @ (tip - R.at())
    down = 15.0 if mode == "shirt" else 30.0 if mode == "shirtpass" else G_DOWN  # the shirt check tilts the needle 15 degrees down at the body; shirtpass 30, with the needle through the label low by its seam (u 12 mm, about 11 mm over the cloth), so that it goes on into the shirt behind it (the needle is 25 mm long: through a higher point its tip cannot reach the cloth)
    n = np.array([0.0, math.cos(math.radians(down)), -math.sin(math.radians(down))])
    aim = d._as_grip(_rotx(-math.radians(down)) @ g0.rot)
    tip_off = aim.rot @ in_hand
    segs, h0, _, a0 = label_frames(env)
    u = HOLD_U - PIERCE_BELOW  # below the left fingertips
    if mode == "shirtpass":
        u = 0.012
    if mode == "padpush":  # failure check: the needle pushed into the label right beside the left hand's pads, so that the 4 N can come from a pad, not from the label
        u = HOLD_U - 0.003
    if mode == "shirt":  # failure check: the needle pushed into the shirt's body beside the label instead of through the label
        pierce = np.array([SEAM[0] + 0.035, SEAM[1] + 0.04, TOP + BODY_T])
    else:
        pierce = label_point(segs, u)
    print("[tag] aim: " + json.dumps({"label_deg": round(math.degrees(a0), 1), "pierce_u_mm": round(u * 1000, 1), "pierce": [round(float(v), 4) for v in pierce]}), flush=True)
    yield from _logged(R.move(pierce - 0.05 * n - tip_off, grip=aim, t=2.0, label="aim the needle at the label"), ctx, env, "aim")
    yield from _logged(R.servo(pierce - 0.05 * n, tol=0.0015, timeout=1.5, read=lambda: needle_tip(env)[0], label="the needle's tip 5 cm short of the label"), ctx, env, "short")
    if mode != "nopush":
        past = -0.001 if mode == "light" else 0.03 if mode == "shirtpass" else PUSH_PAST  # light: the tip brought to the label's face and stopped there, a touch
        target = (lambda: pierce) if mode == "shirt" else (lambda: label_point(label_frames(env)[0], u))  # the label's point read again each step: it moves as the shirt does
        yield from _logged(R.servo(lambda s: target() + (-0.05 + min(s / 1.5, 1.0) * (0.05 + past)) * n, tol=0.0015, timeout=PUSH_S, gain=PUSH_GAIN, read=lambda: needle_tip(env)[0],
                                   label="push the needle into the label until it pierces it, and on through"), ctx, env, "push")
    print("[tag] through: " + json.dumps(TAG.last), flush=True)
    if mode == "drag":  # failure check: the needle, through the label, dragged 15 mm sideways across it before the squeeze: it cuts the label
        yield from R.move(R.at() + np.array([0.015, 0.0, 0.0]), t=0.8, label="drag the needle sideways across the label")
    if mode == "flick":  # failure check: the needle, through the label, yanked 25 mm sideways and back in a tenth of a second each way, then the squeeze
        yield from R.move(R.at() + np.array([0.025, 0.0, 0.0]), t=0.1, label="flick the needle sideways")
        yield from R.move(R.at() - np.array([0.025, 0.0, 0.0]), t=0.1, label="and back")
    if mode != "nofire":  # nofire, a failure check: the needle drawn back out of the label unfired, the label let down
        for _ in R.close(0.030, force=35.0, t=1.0, label="squeeze at up to 35 N: the trigger fires a fastener"):
            yield
            if TAG.fired:
                break
        yield from R.close(0.044, force=20.0, t=0.3, label="ease off the trigger as it fires")
    print("[tag] fired: " + json.dumps(TAG.last), flush=True)
    yield from R.move(R.at() - 0.06 * n, t=1.2, label="draw the needle back")
    yield from R.move(R.at() + np.array([-0.06, -0.08, 0.04]), t=1.5, label="the gun away from the shirt, held")
    if not nolift:
        yield from L.arc(hinge, (1.0, 0.0, 0.0), 25.0 - LIFT_DEG, t=1.5, label="let the label down to 25 degrees")
        yield from L.move(L.at(), finger=-0.006, t=0.5, label="let go of the label: jaws 12 mm apart")
        yield from L.move(L.at() + np.array([0.0, 0.05, 0.0]), t=1.2, label="draw the jaws out level, off the label's edge")
        yield from L.move(L.at() + np.array([0.0, 0.02, 0.06]), t=1.0, label="left clear")
    if mode == "shaftrest":  # not a failure: after the act, the gun laid down with its needle's shaft on the shirt and its tip clear of it (a shaft resting there makes no hole)
        top = TOP + BODY_T
        up = -12.0  # the needle pitched 12 degrees up: its tip 5 mm higher than the shaft's base, 25 mm back
        aim2 = d._as_grip(_rotx(-math.radians(up)) @ g0.rot)
        n2 = np.array([0.0, math.cos(math.radians(up)), -math.sin(math.radians(up))])
        tip_off2 = aim2.rot @ in_hand
        rest = np.array([SEAM[0] + 0.06, SEAM[1] + 0.12, top + 0.0065])  # the tip 6.5 mm over the cloth, the shaft's base about 1.3 mm over it
        yield from R.move(rest - 0.04 * n2 + np.array([0.0, 0.0, 0.03]) - tip_off2, grip=aim2, t=2.0, label="over the shirt, needle pitched up")
        yield from R.servo(rest, tol=0.001, timeout=2.0, read=lambda: needle_tip(env)[0], label="lower the gun until its shaft rests just over the shirt")
        yield from L.wait(1.5)
        print("[tag] shaftrest: " + json.dumps(TAG.last), flush=True)
    if mode == "undo":  # failure check: the full act at 1.0, then the gun brought back and its needle pushed into the shirt's body: the score goes to 0 and success() goes False
        yield from L.wait(1.5)
        print("[tag] full: " + json.dumps(TAG.last), flush=True)
        down2 = 15.0
        aim2 = d._as_grip(_rotx(-math.radians(down2)) @ g0.rot)
        n2 = np.array([0.0, math.cos(math.radians(down2)), -math.sin(math.radians(down2))])
        tip_off2 = aim2.rot @ in_hand
        stab = np.array([SEAM[0] + 0.035, SEAM[1] + 0.1, TOP + BODY_T])
        yield from R.move(stab - 0.05 * n2 - tip_off2 + np.array([0.0, 0.0, 0.02]), grip=aim2, t=2.0, label="the gun back over the shirt")
        yield from R.servo(lambda s: stab + (-0.05 + min(s / 1.5, 1.0) * 0.056) * n2, tol=0.0015, timeout=4.0, gain=PUSH_GAIN, read=lambda: needle_tip(env)[0],
                           label="push the needle into the shirt's body")
        yield from L.wait(1.0)
        print("[tag] stabbed: " + json.dumps({**TAG.last, "success": success(None), "score": score(None)}), flush=True)
    yield from L.wait(1.5)
    print("[tag] END " + json.dumps({"mode": mode, **TAG.last}), flush=True)
