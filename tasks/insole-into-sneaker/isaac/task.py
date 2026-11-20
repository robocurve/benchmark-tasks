"""Put a soft full-length foam insole back into a lace-up sneaker (Societal relatability). yam_arena task file
(Isaac Lab-Arena, PhysX).

A men's US 9 lace-up running shoe (Brooks Ghost 18 proportions; one free rigid body, 0.27 kg, with a soft tongue
jointed to it) stands on the table, toe away from the robot, its insole removed. Beside it the shoe box; on its lid
lies the insole (a chain of foam slices with constant bending springs, insolekit.py), top up, toe away from the
robot, heel end 4 cm over the lid's near edge. The insole (267 mm) is twice as long as the collar opening (130 mm),
so it has to be bent to go in; its toe goes out of sight under the laced vamp, and it is pushed home from the heel
end. The tongue leans back over the front of the opening and has to be held out of the way.

Score (insole_scoring.py), every control step from measured poses, the value at the end of the trial counts:
0 unless the shoe stands upright on the table and both grippers are at least 3 cm from the shoe and the insole;
0.25 insole inside the shoe (nothing above the collar, not folded back, the mark 20 mm from its toe under the vamp);
0.5 also home (heel within 6 mm of the counter and down on the bed, toe within 10 mm of the toe box end); 1.0 also
flat (its underside within 3 mm of the bed at the heel, arch and ball marks). The trial ends after 1.0 has held 4 s.
"""
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import geom  # noqa: E402
import insole_scoring  # noqa: E402
import insolekit  # noqa: E402
import partkit  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 4.0   # the trial ends once 1.0 has held for 4 s (time for both arms to reach READY)
GRIP_N = 40.0          # each YAM finger drive capped at 40 N
INSTRUCTION = ("Put the blue insole into the sneaker so that it lies flat on the bottom of the shoe, pushed all the way "
               "in to the toe with its heel end down inside the heel, then let go of it and the shoe. You may hold "
               "the shoe and move its tongue; leave the laces tied.")

shoe = partkit.part("shoe", pos=tuple(geom.to_world((0.0, 0.0, 0.0))),
                    boxes=tuple(geom.shoe_boxes()),
                    mass=geom.SHOE_MASS, friction=float(os.environ.get("INSOLE_SHOE_MU", "0.8")), color=geom.UPPER,
                    damping=0.05, fixed=os.environ.get("INSOLE_SHOE_FIXED") == "1")   # env knobs: diagnostics only
shoebox = Box("shoebox", pos=geom.BOX_C, size=geom.BOX_SIZE, color=(0.85, 0.80, 0.70), mass=0.30)
heel, heading = geom.insole_start()
insole = insolekit.insole("insole", "chain", heel, heading)
tongue = insolekit.flap("tongue", "shoe", geom.TONGUE_ROOT, geom.tongue_rest_rot(), tuple(geom.to_world((0, 0, 0))),
                        root_rot=geom.tongue_rest_rot(geom.TONGUE_MID_DEG),
                        n_seg=geom.TONGUE_N, seg_len=geom.TONGUE_FREE / geom.TONGUE_N, width=geom.TONGUE_W,
                        thick=geom.TONGUE_T, mass_kg=geom.TONGUE_MASS, k_bend=geom.TONGUE_K, color=geom.PAD,
                        tip_color=geom.OVERLAY)
OBJECTS = [shoe, shoebox, insole] + ([] if os.environ.get("INSOLE_NO_TONGUE") == "1" else [tongue])   # diagnostic knob

SHOE_BOX = ((geom.X_BACK - 0.005, -0.060, 0.0), (geom.X_TIP + 0.003, 0.060, 0.140))   # outline box incl. the tongue
STATE = {"score": 0.0, "why": "", "m": None, "steps": 0}


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def measure(env):
    st = insolekit.insole_state(env, "insole")
    p, q, _ = partkit.pose(env, "shoe")
    fingers = np.vstack([np.asarray(pts) for pts, _ in partkit.gripper_points(env).values()])
    return insole_scoring.measure(st.points, st.under, st.cloud, p, _rot(q), fingers, li=geom.LI, throat=geom.THROAT,
                                  half_in=geom.half_in, bed=geom.bed, top_h=geom.top_h, thick=geom.T,
                                  shoe_rest_z=geom.SHOE_Z, shoe_box=SHOE_BOX)


@partkit.on_step
def _score_step(env):
    m = measure(env)
    STATE["m"] = m
    STATE["score"], STATE["why"] = insole_scoring.score(m)
    STATE["steps"] += 1


@partkit.on_reset
def _reset(env):
    import torch
    STATE.update(score=0.0, why="", m=None, steps=0)
    for key in ("left_robot", "right_robot"):
        r = env.unwrapped.scene[key]
        ids = [list(r.joint_names).index(j) for j in ("joint7", "joint8")]
        r.write_joint_effort_limit_to_sim(torch.full((1, 2), GRIP_N, device=r.device), joint_ids=ids)


def score(objects):
    return float(STATE["score"])


def success(objects):
    return STATE["score"] >= 1.0


def demo():
    return {}  # yam_scripted holds READY (do-nothing baseline); the scripted demo is demo/run_demo.py
