"""Write print files for the real-trial props from the same procedural geometry the sim uses (weed.py).

Run from the task folder:  python3 fabrication/make_stl.py   (needs the Robocurve runtime's yam_arena on the
path, as simkit.py imports it). Writes, in metres scaled to millimetres:
  bulb_pan_8in.stl        the pot shell with its ribs (8 in top, 6.75 in base, 4 in deep)
  crown_upper_root.stl    crown + upper taproot (22 mm crown, 11 mm root, 28 mm long), one print
  lower_root.stl          lower taproot (9 mm tapering to 3 mm, 35 mm long)
"""
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import weed as W  # noqa: E402


def write_stl(path, meshes):
    tris = []
    for p, t in meshes:
        p = np.asarray(p, float) * 1000.0
        for a, b, c in np.asarray(t, int):
            tris.append((p[a], p[b], p[c]))
    with open(path, "wb") as f:
        f.write(b"robocurve potted-dandelion-weed prop".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    print(path, len(tris), "triangles")


def part_meshes(part, names):
    return [(v[1], v[2]) for v in part.vmeshes if any(v[0].startswith(n) for n in names)]


def pot_solid():
    """A closed 2 mm-wall shell with a 3 mm floor and a 7 mm-wide rim band, revolved about the axis."""
    import meshlib as M
    H, t = W.POT_H, 0.002
    prof = [(0.0, 0.0), (W.R_BOT, 0.0)]
    for k in range(1, 9):
        z = H * k / 9
        prof.append((W.wall_r(z), z))
    prof += [(W.R_TOP + 0.0035, H - 0.007), (W.R_TOP + 0.0035, H), (W.R_TOP - t, H)]
    for k in range(8, 0, -1):
        z = 0.003 + (H - 0.006) * k / 8
        prof.append((W.wall_r(z) - t, z))
    prof += [(W.R_BOT - t, 0.003), (0.0, 0.003)]
    return [M.revolve(prof, n=120, axis="Z")]


if __name__ == "__main__":
    write_stl(HERE / "bulb_pan_8in.stl", pot_solid())
    write_stl(HERE / "crown_upper_root.stl", part_meshes(W.weed_object(), ("crown", "taproot_upper")))
    write_stl(HERE / "lower_root.stl", part_meshes(W.root_object(), ("taproot_lower",)))
