"""Prime the paper in an already open, wall-mounted lever roll-towel dispenser."""
from pathlib import Path
import math
import os
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
from feed import Feed, LEADER_RESET, WIDTH, NIP_X, NIP_Z, LEAD_LENGTH
from yam_arena.constants import FINGER_CLOSED, GRIPPER

FINGER_OPEN = -.03745
# The one-task runner loads this before it constructs the native arm drives.
GRIPPER["effort"] = 40.

SCENE = "station"
INSTRUCTION = "Start the new roll in the open paper-towel dispenser. Feed a straight, unbroken tail through the outlet and leave the lever released."
SUCCESS_HOLD_S = 0.
TOWEL_TASK = True

LOOKS = {
    "shell": m.Look(color=m.rgb(.18, .25, .32), roughness=.65),
    "plate": m.Look(color=m.rgb(.76, .79, .80), roughness=.75),
    "rubber": m.Look(color=m.rgb(.13, .15, .15), roughness=.9),
    "paper": m.Look(color=m.rgb(.96, .89, .69), roughness=.95),
    "lever": m.Look(color=m.rgb(.13, .36, .57), roughness=.55),
    "core": m.Look(color=m.rgb(.55, .40, .25), roughness=.9),
}
LEVER_PIVOT = (.365, -.167, .953)
LEVER_LENGTH = .075

features = [
    ("mount", (.018, .410, .410), (.557, 0., .9525), "plate"),
    ("back", (.008, .318, .324), (.544, 0., .982), "shell"),
    ("base_front", (.059, .318, .008), (.369, 0., .824), "shell"),
    ("base_rear", (.103, .318, .008), (.489, 0., .824), "shell"),
    ("side_a", (.198, .008, .324), (.445, .155, .982), "shell"),
    ("side_b", (.198, .008, .324), (.445, -.155, .982), "shell"),
    ("cover_open", (.195, .310, .006), (.247, 0., .813), "shell"),
]
frame_parts = [(name, m.box(size, pos), look) for name, size, pos, look in features]
frame_colliders = [m.col_box(name + "_shape", size, pos) for name, size, pos, _ in features]
for name, x, z, radius, w, look in [
    ("roll", .475, 1.065, .075, WIDTH, "paper"),
    ("core", .475, 1.065, .018, .218, "core"),
    ("front_roller", .408, .927, .0198, .219, "rubber"),
    ("rear_roller", .448, .927, .0198, .219, "rubber"),
]:
    mesh = m.cylinder(radius, w, (x, 0., z), axis="y", segments=48)
    frame_parts.append((name, mesh, look))
    frame_colliders.append(m.col_hull(name + "_shape", mesh).replace("contactOffset = 0.004", "contactOffset = 0.0003"))

lever = m.lever("lever", LEVER_PIVOT, (-1., 0., 0.), axis="Y", look="lever",
                length=LEVER_LENGTH, radius=.009, ball_r=.016, lower=-38.,
                upper=0., spring=1.5, damping=.05, friction=.016, mass=.080)
OBJECTS = [m.mechanism("dispenser", (0, 0, 0), [lever], LOOKS,
                       frame_parts=frame_parts, frame_colliders=frame_colliders)]
_feed = Feed()
_paper = None


def _np(value):
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy()


def _hands(base):
    from yam_arena.runtime import link_poses, tcp_world

    origin = _np(base.scene.env_origins[0])
    tips, axes, apertures = [], [], []
    for side in ["left", "right"]:
        robot = base.scene[side + "_robot"]
        links = link_poses(robot)
        # The gripper link's local X axis is the prismatic closing
        # direction. Tip body origins are offset along other axes and
        # their connecting vector is skewed as the gap closes.
        grip = links["gripper"][:3, :3]
        tips.append(np.asarray(tcp_world(robot)) - origin)
        axes.append(grip)
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        apertures.append(float(np.abs(_np(robot.data.joint_pos)[0, ids]).sum()))
    return np.array(tips), np.array(axes), np.array(apertures)


def _paper_rows(model):
    start = np.array([[.400, WIDTH / 2, 1.055], [.400, -WIDTH / 2, 1.055]])
    bend = np.array([[.396, WIDTH / 2, 1.014], [.396, -WIDTH / 2, 1.014]])
    sample_delta = model.leader[0] - model.leader[1]
    ends = np.stack((model.leader[0] + (WIDTH / .15 - 1) * sample_delta / 2,
                     model.leader[1] - (WIDTH / .15 - 1) * sample_delta / 2))
    if not model.seated:
        ends[:, 2] -= LEAD_LENGTH
    guide = np.array([[NIP_X, WIDTH / 2, NIP_Z], [NIP_X, -WIDTH / 2, NIP_Z]])
    mid = guide if model.seated else bend + (ends - bend) * .55
    return np.stack((start, bend, mid, ends)).reshape(-1, 3)


def _draw(base):
    global _paper
    from pxr import Gf, UsdGeom, Vt

    if _paper is None:
        _paper = UsdGeom.Mesh.Define(base.sim.stage, "/World/FeedPaper")
        _paper.CreateSubdivisionSchemeAttr("none")
        _paper.CreateDoubleSidedAttr(True)
        _paper.CreateFaceVertexCountsAttr([4] * 3)
        _paper.CreateFaceVertexIndicesAttr([j for i in range(0, 6, 2) for j in [i, i + 1, i + 3, i + 2]])
        colors = _paper.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
        colors.Set([Gf.Vec3f(.89, .78, .51), Gf.Vec3f(.98, .91, .70), Gf.Vec3f(.94, .83, .56)])
    points = _paper_rows(_feed) + _np(base.scene.env_origins[0])
    _paper.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))


@m.on_reset
def reset(env):
    global _feed, _paper
    _feed, _paper = Feed(), None
    if os.environ.get("TOWEL_TRACE"):
        Path(os.environ["TOWEL_TRACE"]).write_text("")
    base = env.unwrapped
    _draw(base)
    # Invalidate the frames recorded by the stock reset before replacing
    # the paper. Render the restored scene before it reaches the policy.
    for camera in ("top_cam", "left_cam", "right_cam"):
        base.scene[camera].reset()
    for _ in range(4):
        base.sim.render()


@m.on_step
def update(env):
    import json

    base = env.unwrapped
    tcp, axes, apertures = _hands(base)
    lever_angle = m.state(env, "dispenser").joint("lever")
    _feed.update(tcp, axes, apertures, lever_angle)
    _draw(base)
    if os.environ.get("TOWEL_TRACE"):
        with open(os.environ["TOWEL_TRACE"], "a") as trace:
            trace.write(json.dumps({"tcp": tcp.tolist(), "aperture": apertures.tolist(),
                                    "lever_deg": math.degrees(lever_angle), "leader": _feed.leader.tolist(),
                                    "grips": list(_feed.grip_to_point), "seated": _feed.seated,
                                    "exposed_m": _feed.exposed, "damage": _feed.damaged,
                                    "dwell": _feed.dwell, "axes": axes.tolist(), "score": _feed.score}) + "\n")


def score(objects):
    return _feed.score


def success(objects):
    return score(objects) == 1.


class Pitch(float):
    """Yaw with a pitch for a downward grasp that clears the mounted roll."""

    def __new__(cls, yaw, degrees):
        item = float.__new__(cls, yaw)
        item.pitch = math.radians(degrees)
        return item


def demo():
    from yam_arena import demo as motion
    from yam_arena.urdf import axis_angle_matrix

    # The stock planner accepts yaw-only waypoints. Extend that conversion
    # for the paper handling poses while preserving every stock yaw.
    if not getattr(motion.grip_rotation, "towel_pitch", False):
        plain = motion.grip_rotation

        def grip_rotation(yaw=0.):
            rotation = plain(float(yaw))
            if isinstance(yaw, Pitch):
                return axis_angle_matrix(np.array([0., 1., 0.]), yaw.pitch) @ rotation
            return rotation

        grip_rotation.towel_pitch = True
        motion.grip_rotation = grip_rotation
    Waypoint = motion.Waypoint

    plans = {}
    for side, index in (("left", 0), ("right", 1)):
        x, y, z = LEADER_RESET[index]
        heading = math.pi / 2 if side == "right" else -math.pi / 2
        yaw = Pitch(heading, -22.)
        feed_pitch = Pitch(heading, -62.)
        # Fixed compensation for the reproducible left/right tracking
        # offset measured during the 200-step development grasp. The
        # controller still uses only the preset scene geometry and time.
        if side == "left":
            pickup = (x + .003, y - .004, z - .001)
        else:
            pickup = (x - .0035, y + .004, z - .0015)
        path = [
            Waypoint(2., (x - .055, y, z + .045), FINGER_OPEN, "reach the visible edge", yaw),
            Waypoint(2., pickup, FINGER_OPEN, "straddle the edge", yaw),
            Waypoint(1.2, pickup, FINGER_CLOSED, "pinch", yaw),
            Waypoint(1.6, (x, y, z), FINGER_CLOSED, "tilt palm clear of the roll", feed_pitch),
            Waypoint(2.2, (NIP_X - .014, y, NIP_Z + LEAD_LENGTH + .008), FINGER_CLOSED, "turn the paper down", feed_pitch),
            Waypoint(1., (NIP_X - .001, y, NIP_Z + LEAD_LENGTH + .001), FINGER_CLOSED, "set the edge into the nip", feed_pitch),
            Waypoint(1., (NIP_X - .001, y, NIP_Z + LEAD_LENGTH + .001), FINGER_OPEN, "release the paper", feed_pitch),
            Waypoint(2., (x - .060, y, z + .048), FINGER_OPEN, "back away", yaw),
        ]
        if side == "right":
            p = np.asarray(LEVER_PIVOT)
            for _ in range(5):
                for duration, degrees, dz, label in [
                    (1.2, 0, .052, "above lever"),
                    (1.0, 0, .024, "touch lever"),
                    (1.4, -35, .010, "press lever"),
                    (1.0, 0, .052, "release lever"),
                ]:
                    a = math.radians(degrees)
                    tip = p + [-LEVER_LENGTH * math.cos(a), 0., LEVER_LENGTH * math.sin(a) + dz]
                    path.append(Waypoint(duration, tuple(tip), FINGER_CLOSED, label, yaw))
            path.append(Waypoint(2., (.21, -.21, 1.025), FINGER_OPEN, "hand clear", yaw))
        else:
            # Look through the open feed bay toward the hanging edge.
            # The fixed wrist pose keeps the fingers away from the outlet.
            path.append(Waypoint(3., (.320, .110, .915), FINGER_OPEN,
                                 "show the outlet from the wrist", Pitch(math.radians(45.), -30.)))
        plans[side] = path
    return plans
