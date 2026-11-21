"""clearance.py: the closest approach of each arm's links and fingertips (YAM collision meshes, URDF FK) to the sharps
container's box over both demo plans (played as lancet_fingerstick.TiltPlan, as the patched scripted demo plays them), at the eight corners of the layout box and on --seed 0, 2, 4. Fingers set closed on what they hold. No sim.
WHY: the first layout put the container 43 mm from the stand's axis; the fingers reach 52 mm from the TCP axis, so the
tab twist swung a finger into the container's wall, pushed the arm 20 mm and tipped the lancet out of the socket."""
import glob
import pathlib
import re
import struct
import sys
import warnings
import zlib

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import yam_arena  # noqa: E402
import lancet_fingerstick as F  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world  # noqa: E402

MIN_MM = 10.0
Y = pathlib.Path(yam_arena.__file__).parent / "assets" / "yam"
URDF = (Y / "yam.urdf").read_text()
LINKS = ("tip_left", "tip_right", "gripper", "link5", "link4", "link3", "link2", "link1")
HELD = {"right": F.L_T / 2 - 0.0002, "left": F.FINGERS["ring"][4] - 0.0005}   # m: the finger joint stops on the
#                                  housing or the ring finger


def _offset(link):   # WHY: the collision meshes sit in the visual frame; this URDF has no <collision> origins
    blk = re.search(rf'<link name="{link}">(.*?)</link>', URDF, re.S).group(1)
    return np.array([float(v) for v in re.search(r'<visual>\s*<origin xyz="([^"]+)"', blk).group(1).split()])


def _stl(path):
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float)


MESH = {k: (np.vstack([_stl(pathlib.Path(p)) for p in glob.glob(str(Y / "collision" / k / "*.stl"))]) + _offset(k))[::5]
        for k in LINKS}
cx, cy = F.SHARPS
BOX = np.array([[cx - F.SH_L / 2, cy - F.SH_W / 2, F.TOP], [cx + F.SH_L / 2, cy + F.SH_W / 2, F.TOP + F.SH_H]])
worst = (np.inf, None)
for seed in F.corners() + [zlib.crc32(f"{c}:None:0".encode()) for c in (0, 2, 4)]:
    for side, ws in zip(("right", "left"), F.plan(0.0, seed)):
        p, finger = F.TiltPlan(ws, station.BASES[side]), {w.label: w.finger for w in ws}
        t = 0.0
        while t <= p.duration:
            q7, label = p.action7(t)
            q = {f"joint{i}": float(q7[i - 1]) for i in range(1, 7)}
            q["joint7"] = q["joint8"] = min(finger[label], -HELD[side])
            pose = fk_link_poses_world(q, station.BASES[side])
            for k, v in MESH.items():
                w = v @ pose[k][:3, :3].T + pose[k][:3, 3]
                d = float(np.linalg.norm(np.maximum(np.maximum(BOX[0] - w, w - BOX[1]), 0.0), axis=1).min())
                if d < worst[0]:
                    worst = (d, f"layout {F.shift_turn(seed)} {side} t {t:.2f} s '{label}' {k}")
            t += 0.05
print(f"closest to the sharps container: {1000 * worst[0]:.1f} mm ({worst[1]}); floor {MIN_MM:.0f} mm")
print("CLEARANCE", "OK" if 1000 * worst[0] >= MIN_MM else "BAD")
sys.exit(0 if 1000 * worst[0] >= MIN_MM else 1)
