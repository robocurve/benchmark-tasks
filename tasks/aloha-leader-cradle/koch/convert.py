"""Convert the Koch v1.1 follower's published meshes for the simulator: each visual mesh the URDF uses (GLB or STL) to a binary STL, clustered on a 1.5 mm
grid (needs trimesh to read GLB; run once, offline).

    python koch/convert.py ManiSkill/mani_skill/assets/robots/koch koch

Reads follower_arm_v1.1.urdf and meshes/ from ManiSkill's Koch v1.1 assets (from jess-moss/koch-v1-1, Apache-2.0) and writes OUT/<stem>.stl for each
visual mesh, and copies the URDF and its LICENSE. The shapes keep their outline to the grid size; the simulator's colliders are convex hulls of them.
"""

import pathlib
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

GRID = 0.0015  # m


def cluster(tri, grid=GRID):
    v = tri.reshape(-1, 3)
    key = np.floor(v / grid).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inv, v)
    centre = sums / np.bincount(inv, minlength=len(uniq))[:, None]
    f = inv.reshape(-1, 3)
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = np.unique(np.sort(f[keep], axis=1), axis=0, return_index=True)[1]
    f = inv.reshape(-1, 3)[keep][np.sort(f)]
    return centre[f]


def write(path, tri):
    with open(path, "wb") as fh:
        fh.write(b"Koch v1.1 mesh, clustered 1.5 mm (jess-moss/koch-v1-1 via ManiSkill, Apache-2.0)"[:80].ljust(80, b" "))
        fh.write(struct.pack("<I", len(tri)))
        for t in tri.astype(np.float32):
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            nrm = nrm / (np.linalg.norm(nrm) or 1.0)
            fh.write(struct.pack("<12fH", *nrm, *t.reshape(-1), 0))


if __name__ == "__main__":
    src, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    out.mkdir(exist_ok=True)
    shutil.copy(src / "follower_arm_v1.1.urdf", out / "follower_arm_v1.1.urdf")
    shutil.copy(src / "LICENSE", out / "LICENSE")
    root = ET.parse(src / "follower_arm_v1.1.urdf").getroot()
    names = {v.find("geometry/mesh").get("filename").split("/")[-1] for v in root.iter("visual")}
    for name in sorted(names):
        mesh = trimesh.load(src / "meshes" / name, force="mesh")
        tri = np.asarray(mesh.vertices)[np.asarray(mesh.faces)]
        dec = cluster(tri)
        write(out / (pathlib.Path(name).stem + ".stl"), dec)
        print(f"{name}: {len(tri)} -> {len(dec)} triangles, bounds {np.round(dec.reshape(-1, 3).min(0), 3)} {np.round(dec.reshape(-1, 3).max(0), 3)}")
