"""Build orca_hand.npz: ORCA Hand v2's palm (carpals, with its TPU skin), four fingers and the thumb's two base pieces (T-TP and T-AP), posed by the URDF's forward kinematics,
all in T-PP's own STL frame (mm), so that the thumb's proximal phalanx sits on the real hand exactly where the CAD puts it.

Source: orcahand_description (MIT), v2/models/urdf/orcahand_right.urdf with its visual STLs (ORCA's own CAD, orcahand_hardware, CC BY 4.0) and its collision STLs (ORCA's convex pieces).
The fingers are at rest (MCP 0.55, PIP 0.75 rad: curled a little, tendons slack). The thumb is at CMC, abduction and MCP of the constants below. The file also holds the CMC joint (the thumb's base
joint on the palm): its point and axis in T-PP's frame, and its range from the URDF, relative to the rest angle.
    python buildhand.py right.urdf assets_dir orca_hand.npz
"""
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

FLEX = {"mcp": 0.55, "pip": 0.75}  # radians: the fingers' rest curl with the tendons slack
THUMB = {"right_thumb_cmc": -0.55, "right_thumb_abd": -0.20, "right_thumb_mcp": 0.55}  # radians: the thumb's rest pose (in range: CMC -0.79 to 0.58, abduction -0.31 to 0.96, MCP -0.44 to 1.75)
MOVING = ("right_thumb_tp", "right_thumb_ap")  # turn with T-PP on the CMC joint
SKIP = ("right_thumb_pp", "right_thumb_dp", "right_thumb_tip", "right_thumb_pad", "right_tower", "right_forearm")


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50 * n == len(b):
        a = np.frombuffer(b[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
        tri = a["v"].astype(float).reshape(-1, 3)
    else:
        tri = np.array([[float(x) for x in l.split()[1:]] for l in b.decode().splitlines() if l.strip().startswith("vertex")])
    v, inv = np.unique(np.round(tri, 6), axis=0, return_inverse=True)
    return v, inv.reshape(-1, 3)


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def axis_rot(a, q):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def tf(el):
    o = el.find("origin") if el is not None else None
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = rpy(*[float(x) for x in o.get("rpy", "0 0 0").split()])
        T[:3, 3] = [float(x) for x in o.get("xyz", "0 0 0").split()]
    return T


def cluster(v, f, cell):
    key = np.floor(v / cell).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    nv = np.zeros((len(idx), 3))
    np.add.at(nv, inv, v)
    nv /= np.bincount(inv, minlength=len(idx))[:, None]
    nf = inv[f]
    nf = nf[(nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])]
    used = np.unique(nf)
    remap = -np.ones(len(nv), int)
    remap[used] = np.arange(len(used))
    return nv[used], remap[nf]


def main():
    urdf, assets, out = sys.argv[1:4]
    root = ET.parse(urdf).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    parent, jt, jaxis, jname, jlim = {}, {}, {}, {}, {}
    for j in root.findall("joint"):
        c = j.find("child").get("link")
        parent[c] = j.find("parent").get("link")
        jt[c] = tf(j)
        a = j.find("axis")
        jaxis[c] = [float(x) for x in a.get("xyz").split()] if a is not None else None
        jname[c] = j.get("name")
        lim = j.find("limit")
        jlim[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None
    q = dict(THUMB)
    for f in ("index", "middle", "ring", "pinky"):
        for k, v in FLEX.items():
            q[f"right_{f}_{k}"] = v

    def world(name):
        T = np.eye(4)
        while name in parent:
            J = np.eye(4)
            if jaxis[name] is not None:
                J[:3, :3] = axis_rot(jaxis[name], q.get(jname[name], 0.0))
            T = jt[name] @ J @ T
            name = parent[name]
        return T

    to_pp = np.linalg.inv(world("right_thumb_pp"))
    res = {}
    for name, l in links.items():
        if name in SKIP:
            continue
        T = to_pp @ world(name)
        group = "m" if name in MOVING else "h"  # m: on the CMC joint with T-PP; h: the fixed hand
        for kind in ("visual", "collision"):
            for g in l.findall(kind):
                mesh = g.find("geometry/mesh")
                fn = mesh.get("filename").split("/")[-1]
                s = float(mesh.get("scale", "1 1 1").split()[0])
                v, f = read_stl(f"{assets}/{fn}")
                Tg = T @ tf(g)
                mm = ((Tg[:3, :3] @ (v * s).T).T + Tg[:3, 3]) * 1000.0
                key = f"{group}{kind[0]}__{name.replace('right_', '')}__{fn.replace('.stl', '').replace('-', '_')}"
                if kind == "visual":
                    mm, f = cluster(mm, f, 1.6 if "Carpals" in fn and "Skin" not in fn else 0.7)
                res[key + "_v"], res[key + "_f"] = mm.astype(np.float32), f.astype(np.int32)
                print(key, len(mm), "verts; x", mm[:, 0].min().round(1), mm[:, 0].max().round(1), "y", mm[:, 1].min().round(1), mm[:, 1].max().round(1), "z", mm[:, 2].min().round(1), mm[:, 2].max().round(1))
    Ttp = to_pp @ world("right_thumb_tp")  # the CMC joint's frame is T-TP's: its point, and its axis in that frame
    res["cmc_at"] = (Ttp[:3, 3] * 1000.0).astype(np.float64)
    res["cmc_axis"] = (Ttp[:3, :3] @ np.asarray(jaxis["right_thumb_tp"], float)).astype(np.float64)
    lo, hi = jlim["right_thumb_cmc"]
    res["cmc_range"] = np.array([lo - THUMB["right_thumb_cmc"], hi - THUMB["right_thumb_cmc"]])
    res["rest"] = np.array([THUMB["right_thumb_cmc"], THUMB["right_thumb_abd"], THUMB["right_thumb_mcp"], FLEX["mcp"], FLEX["pip"]])
    Tc = to_pp @ world("right_carpals")
    res["carpals_T"] = Tc  # the palm's frame (m) in T-PP's frame: x across (index to little finger), y the palm side, z along the fingers
    print("CMC at", res["cmc_at"].round(1), "mm, axis", res["cmc_axis"].round(3), "range", np.degrees(res["cmc_range"]).round(1), "deg")
    cpal = np.linalg.inv(world("right_carpals")) @ world("right_thumb_tp")
    print("CMC in the palm's frame (mm):", (cpal[:3, 3] * 1000).round(1), "axis", (cpal[:3, :3] @ np.asarray(jaxis["right_thumb_tp"], float)).round(3))
    np.savez_compressed(out, **res)


main()
