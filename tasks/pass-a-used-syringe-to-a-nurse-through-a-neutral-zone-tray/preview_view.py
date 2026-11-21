"""Offline: a rough picture of the start (and the demo's end) from the station's top camera at 320 x 176, drawn from
the meshes (flat shading, no lighting). Checks framing and what is readable before a GPU run.
    python preview_view.py out.png          (needs RSKIT_CACHE with the syringe package)"""
import pathlib
import sys
import xml.etree.ElementTree as ET

import matplotlib
import numpy as np
import trimesh
from pxr import Usd, UsdGeom
from yam_arena import YAM_URDF, station, task_file
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix, rpy_matrix

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

T = task_file.load("task.py").module
cam = station.top_camera()
R, P, L = quat_xyzw_to_matrix(np.array(cam.rot_xyzw)), np.array(cam.pos), cam.lens
S = 0.5                                          # 640 x 360 lens drawn at 320 x 180


def project(w):
    v = (np.asarray(w) - P) @ R
    return np.c_[L.fx * v[:, 0] / -v[:, 2] + L.cx, L.cy - L.fy * v[:, 1] / -v[:, 2]] * S, -v[:, 2]


def draw(ax, parts, under=()):
    """``under`` (the table) is drawn first; the rest back to front by triangle depth."""
    for v, f, c in under:
        uv, _ = project(v)
        ax.add_collection(PolyCollection(uv[f], facecolors=[c] * len(f), edgecolors="none"))
    tris, cols, depth = [], [], []
    for v, f, c in parts:
        uv, d = project(v)
        tris.append(uv[f]); depth.append(d[f].mean(1))
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.55 + 0.45 * np.abs(n @ np.array([0.2, -0.3, 0.93]))
        base = np.broadcast_to(c, (len(f), 3)) if np.ndim(c) == 1 else c
        cols.append(np.clip(base * shade[:, None], 0, 1))
    tri, col, dep = np.concatenate(tris), np.concatenate(cols), np.concatenate(depth)
    o = np.argsort(-dep)
    ax.add_collection(PolyCollection(tri[o], facecolors=col[o], edgecolors="none"))
    ax.set_xlim(0, L.width * S); ax.set_ylim(L.height * S, 0); ax.set_aspect("equal"); ax.axis("off")


def box(lo, hi):
    x0, y0, z0 = lo; x1, y1, z1 = hi
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
    f = np.array([[0, 2, 6], [0, 6, 4], [1, 3, 7], [1, 7, 5], [0, 1, 3], [0, 3, 2], [4, 5, 7], [4, 7, 6], [0, 1, 5],
                  [0, 5, 4], [2, 3, 7], [2, 7, 6]])
    return v, f


def usd_parts(path, keep):
    st = Usd.Stage.Open(str(path)); cache = UsdGeom.XformCache(); out = []
    for p in st.Traverse(Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Mesh) and any(k in str(p.GetPath()) for k in keep):
            g = UsdGeom.Mesh(p)
            pts = np.array(g.GetPointsAttr().Get(), float)
            M = np.array(cache.GetLocalToWorldTransform(p)).T
            w = (np.c_[pts, np.ones(len(pts))] @ M.T)[:, :3]
            cnt, idx = np.array(g.GetFaceVertexCountsAttr().Get()), np.array(g.GetFaceVertexIndicesAttr().Get())
            faces, k = [], 0
            for c in cnt:
                faces += [[idx[k], idx[k + j], idx[k + j + 1]] for j in range(1, c - 1)]
                k += c
            out.append((p.GetPath().pathString, w, np.array(faces)))
    return out


def place(v, pos, q):
    return np.asarray(v) @ quat_xyzw_to_matrix(np.asarray(q, float)).T + np.asarray(pos)


syr_file = T.rskit.simready_file("syringe_a01", "sm_syringe_a01_01.usd")
COL = {"plunger": (0.92, 0.92, 0.9), "barrel": (0.8, 0.85, 0.88), "needle": (0.75, 0.75, 0.78), "part_02": (0.1, 0.1, 0.1),
       "part_01": (0.95, 0.33, 0.02), "barrelDecal": (0.1, 0.1, 0.1), "cap": (0.95, 0.33, 0.02)}
syr_parts = usd_parts(syr_file, [k + "_01" if k not in ("part_02", "part_01") else k for k in COL])
urdf = pathlib.Path(YAM_URDF)
MESH = {}
for link in ET.parse(urdf).getroot().findall("link"):
    vis = link.find("visual"); o = vis.find("origin")
    xyz = np.array([float(a) for a in o.get("xyz").split()]); rpy = np.array([float(a) for a in o.get("rpy").split()])
    tm = trimesh.load(urdf.parent / vis.find("geometry/mesh").get("filename"))
    MESH[link.get("name")] = (tm.vertices @ rpy_matrix(rpy).T + xyz, tm.faces)


def arm(q6, base):
    qd = {n: float(a) for n, a in zip(ARM_JOINTS, q6)}; qd["joint7"] = qd["joint8"] = FINGER_OPEN
    fk = forward_kinematics(YAM_URDF, qd)
    return [(v @ fk[n][:3, :3].T + fk[n][:3, 3] + np.asarray(base), f, np.array((0.12, 0.12, 0.12) if n in ("gripper", "tip_left", "tip_right", "link5") else (0.9, 0.9, 0.9)))
            for n, (v, f) in MESH.items()]


nurse = np.load("assets/nurse_mesh.npz")
nv = T._nurse_world(nurse["v"]); nc = np.power(nurse["c"], 1 / 2.2)[nurse["f"]].mean(1)
dish = np.load("assets/tray_mesh.npz")
fig, axes = plt.subplots(1, 2, figsize=(13, 3.8))
for ax, grip in zip(axes, (T.SYR_START, T.GRIP_IN_TRAY)):
    q = T.SYR_ROT_START if grip is T.SYR_START else T.SYR_ROT
    so = T.syringe_origin_for_grip(grip, q)
    table = [(*box((-0.19, -0.65, T.TOP - 0.06), (0.91, 0.65, T.TOP)), np.array((0.86, 0.73, 0.55)))]
    parts = [(nv, nurse["f"], nc),
             (place(dish["v"], (*T.TRAY_XY, T.TOP + 0.001), T.TRAY_ROT), dish["f"], np.array((0.66, 0.67, 0.7)))]
    for name, w, f in syr_parts:
        key = next(k for k in COL if k in name)
        if key == "cap":
            parts.append((place(w, (*T._CAP_ORIGIN, T.TOP + 0.001 - 0.0081), T.SYR_ROT_START), f, np.array(COL[key])))
        else:
            z = T.TOP + 0.001 + (0.0 if grip is T.SYR_START else 0.001)
            parts.append((place(w, (*so, z), q), f, np.array(COL[key])))
    for side in ("left", "right"):
        parts += arm(READY, station.BASES[side])
    draw(ax, parts, table)
    ax.set_title("start" if grip is T.SYR_START else "after the demo (arm not drawn moved)")
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=90)
