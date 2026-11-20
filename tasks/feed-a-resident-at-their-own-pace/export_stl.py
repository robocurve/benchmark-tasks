"""Writes the printable mouth module for the real rig, in millimetres, from the same boxes the simulation collides
with (make_assets.py): print/mouth_skull.stl (upper teeth, palate, cheeks, throat wall and the swallow cup: one
rigid block fixed inside the head; the face itself is the mannequin's), print/mouth_jaw.stl (lower teeth and chin
plate: on the jaw servo's horn, hinge axis at JAW_HINGE), print/mouth_tongue.stl (the tongue plate with its raised
tip and sides: on the tongue servo's horn, hinge axis at its front edge) and print/lip_moulds.stl (the two lips as
positives, to take silicone moulds from). Frame: origin at the upper incisor edge on the mid-line, x into the head,
y to the manikin's right, z up. Every part is a set of closed boxes (overlaps are merged by the slicer).
NumPy only: python export_stl.py
"""

import struct
from pathlib import Path

import numpy as np

import make_assets as A

HERE = Path(__file__).resolve().parent


def box_tris(lo, hi):
    pts, faces = A.box_mesh(lo, hi)
    P = np.asarray(pts, float) * 1000.0
    return [(P[f[0]], P[f[i]], P[f[i + 1]]) for f in faces for i in (1, 2)]


def write(name, boxes, skip=()):
    tris = [t for n, lo, hi in boxes if not n.startswith(skip) for t in box_tris(lo, hi)]
    out = HERE / "print" / f"{name}.stl"
    out.parent.mkdir(exist_ok=True)
    with open(out, "wb") as f:
        f.write(name.encode().ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            nrm = np.cross(b - a, c - a)
            nrm = nrm / (np.linalg.norm(nrm) or 1.0)
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))
    print(f"wrote {out} ({len(tris)} triangles)")


def main() -> None:
    write("mouth_skull", A.SKULL_BOXES, skip=("lip_", "face_upper"))   # the mannequin head is the face
    write("mouth_jaw", A.JAW_BOXES, skip=("lip_",))
    write("mouth_tongue", A.TONGUE_BOXES)
    write("lip_moulds", [b for b in A.SKULL_BOXES + A.JAW_BOXES if b[0].startswith("lip_")])


if __name__ == "__main__":
    main()
