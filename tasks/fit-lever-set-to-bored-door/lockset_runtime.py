"""Our model of the two mounting screws, the ratcheting screwdriver and the Phillips bit (pure NumPy; the simulator
hook is installed lazily on the GPU host only).

PhysX cannot cut a #8-32 thread, so the screws are modelled:
- Each screw's head is part of the interior lever base; its shank is not simulated. Once a screw's tip is in its
  post's mouth (at least SHANK_IN into the post, within SHANK_RADIAL_IN of its axis), the shank sits in the post's
  bore: a light spring between the two halves (at most F_SHANK_MAX at each end of the screw) holds the screw's head
  and tip on the post's axis, so the base hangs square on the posts as it does on real screws pushed into the posts'
  mouths; the tip in the mouth lets the base be pushed further on but holds it (at most F_SHANK_AX a screw) against
  being drawn off. It lets go when the tip comes out of the post.
- A screw can be driven once its tip is in its post's mouth, with the outside half through the latch and the base on
  the door face over the blade (as a real 1-1/2 in screw catches the post's thread even with the outside half
  standing proud).
- The Phillips bit is in a screw's recess when the bit's tip is within RADIAL_IN of the screw's axis, its axis within
  ANGLE_IN of the screw's, and its tip at the head (AXIAL_IN). While it is in, a light spring (the recess) holds the
  tip on the screw's axis; drawn back AXIAL_OUT or pushed RADIAL_OUT off, it comes out.
- The ratchet: while the bit is in a screw on the thread, every forward (clockwise, seen from the handle) turn of the
  screwdriver about its own axis turns the screw by the same angle; turns back slip. Two turns make a screw snug
  (a real one is spun in about a dozen turns first); past snug it turns no further.
- The draw, as a thread does it: a caught screw shortens, with its turns, the most the base's head may stand off its
  post (a stiff one-sided spring, K_DRAW, on that separation), from where it caught to the roses on the door by
  CLAMP_FROM turns; it pulls at most DRAW_N, rising over the last half-turn to CLAMP_N to press both roses on the
  door. So the halves are drawn together at the pace of the turning, never slammed. Equal and opposite on the halves.
- Snug: the first screw to reach snug fixes both halves to the door where they are, as a hand-tight lever set is
  (hand-tight #8-32 screws clamp the roses with roughly a kilonewton; no hand turns the set after that).
"""
from __future__ import annotations

import math

import numpy as np

# Geometry the model needs (metres, in each part's own frame); set by the task module.
SCREWS = ((0.0, 0.0114, 0.019), (0.0, 0.0114, -0.019))  # head-face centres on the base (upper, lower)
POSTS = ((0.0, 0.030, 0.019), (0.0, 0.030, -0.019))  # where each screw pulls on the outside half
POST_ENDS = ((0.0, 0.0399, 0.019), (0.0, 0.0399, -0.019))  # the posts' ends (outside half's frame)
S_SEAT = 0.0124  # head-to-post-point separation along the screw with both roses on the door
TURNS_SNUG = 2.0
RADIAL_IN, RADIAL_OUT = 0.0015, 0.003
AXIAL_IN, AXIAL_OUT = 0.002, 0.003  # tip at most this far short of the head face (and at most this far into it)
ANGLE_IN, ANGLE_OUT = 6.0, 12.0  # degrees between the bit's axis and the screw's
K_RECESS, C_RECESS, F_RECESS_MAX = 300.0, 2.0, 2.0  # N/m, N s/m, N
DRAW_N, CLAMP_N, CLAMP_FROM = 4.0, 60.0, 1.5  # N; the pull rises from DRAW_N at CLAMP_FROM turns to CLAMP_N at snug
K_DRAW = 2000.0  # N/m: the thread's hold on the separation
K_SHANK, C_SHANK, F_SHANK_MAX = 1000.0, 3.0, 3.0  # N/m, N s/m, N: the shanks in the posts' bores
SCREW_LEN = 0.0381  # #8-32 x 1-1/2 in: head face to tip
F_SHANK_AX = 2.0  # N: most the tips in the posts' mouths hold the base against being drawn off
SHANK_IN, SHANK_RADIAL_IN, SHANK_RADIAL_OUT = 0.001, 0.0025, 0.004  # the tip finds the post's chamfered mouth
# within 2.5 mm of its axis (the base's clearance holes and its play on the blade let it shift that far)
NAMES = ("outside", "inside", "driver")
ON_THREAD = None  # set by the task module: f(frames) -> bool, both halves on (from poses only)


class Model:
    def __init__(self):
        self.clear()

    def clear(self):
        self.turns = [0.0, 0.0]
        self.engaged = [False, False]
        self.psi = [0.0, 0.0]
        self.ever_engaged = [False, False]
        self.s_catch = [None, None]
        self.last_pull = [0.0, 0.0]
        self.shank = [False, False]
        self.shank_in = [0.0, 0.0]
        self.welded = False

    def pull(self, i):
        return self.last_pull[i]

    def s_max(self, i):
        """The most the head may stand off its post (m) at this screw's turns."""
        t, s0 = self.turns[i], self.s_catch[i]
        if t <= CLAMP_FROM:
            return S_SEAT + (s0 - S_SEAT) * (1.0 - t / CLAMP_FROM)
        return S_SEAT - (t - CLAMP_FROM) / (TURNS_SNUG - CLAMP_FROM) * CLAMP_N / K_DRAW

    def cap(self, i):
        t = self.turns[i]
        if t <= CLAMP_FROM:
            return DRAW_N
        return DRAW_N + (CLAMP_N - DRAW_N) * min(1.0, (t - CLAMP_FROM) / (TURNS_SNUG - CLAMP_FROM))

    def update(self, frames, acc):
        """frames: name -> (pos (3,), rot (3, 3), vel (3,), spin (3,)) in the environment frame."""
        if any(n not in frames for n in NAMES):
            return
        (pi, Ri, vi, wi), (po, Ro, vo, wo), (pd, Rd, vd, wd) = frames["inside"], frames["outside"], frames["driver"]
        on = bool(ON_THREAD(frames)) if ON_THREAD else False
        axis_in = -Ri[:, 1]  # into the door, the screw's direction of travel
        u = Ri[:, 1]  # away from the door on the inside face
        a_post = Ro[:, 1]
        tip = pd.copy()
        bit = -Rd[:, 1]  # the screwdriver's axis, handle to tip
        cos_ang = float(np.clip(bit @ axis_in, -1.0, 1.0))
        ang = math.degrees(math.acos(cos_ang))
        for i, (head_local, post_local) in enumerate(zip(SCREWS, POSTS)):
            head = pi + Ri @ np.asarray(head_local)
            d = tip - head
            axial = float(d @ axis_in)  # > 0: the tip is past the head face (pressed into it)
            radial_v = d - axial * axis_in
            radial = float(np.linalg.norm(radial_v))
            psi = _roll(Rd, Ri, axis_in)
            if not self.engaged[i]:
                if radial <= RADIAL_IN and -AXIAL_IN <= axial <= AXIAL_OUT and ang <= ANGLE_IN:
                    self.engaged[i], self.psi[i], self.ever_engaged[i] = True, psi, True
            elif radial > RADIAL_OUT or axial < -AXIAL_OUT or ang > ANGLE_OUT:
                self.engaged[i] = False
            if self.engaged[i]:
                # the recess holds the tip on the screw's axis
                v_tip = vd + np.cross(wd, tip - pd)
                v_rad = v_tip - (v_tip @ axis_in) * axis_in
                f = -K_RECESS * radial_v - C_RECESS * v_rad
                n = float(np.linalg.norm(f))
                if n > F_RECESS_MAX:
                    f *= F_RECESS_MAX / n
                _wrench(acc, "driver", pd, f, tip)
                turn = _wrap(psi - self.psi[i])
                self.psi[i] = psi
                # the ratchet passes forward turns only, and they drive the screw once its tip is in its post's mouth
                if on and self.shank[i] and turn > 0.0 and self.turns[i] < TURNS_SNUG:
                    self.turns[i] = min(TURNS_SNUG, self.turns[i] + turn / (2 * math.pi))
            # the shank in its post's bore
            end = po + Ro @ np.asarray(POST_ENDS[i])
            rel = head - end
            r_v = rel - (rel @ a_post) * a_post
            tip_s = pi + Ri @ (np.asarray(head_local) - np.array((0.0, SCREW_LEN, 0.0)))
            rel_t = tip_s - end
            r = max(float(np.linalg.norm(r_v)), float(np.linalg.norm(rel_t - (rel_t @ a_post) * a_post)))
            depth = float((end - tip_s) @ u)  # how far the screw's tip is into its post's mouth
            if not self.shank[i]:
                self.shank[i] = depth >= SHANK_IN and r <= SHANK_RADIAL_IN
                self.shank_in[i] = depth
            elif depth < 0.0 or r > SHANK_RADIAL_OUT:
                self.shank[i] = False
            if self.shank[i] and not self.welded:
                # the tip in the post's tapped mouth: the base may be pushed further on, not drawn off (at most
                # F_SHANK_AX a screw)
                self.shank_in[i] = max(self.shank_in[i], depth)
                f_ax = min(F_SHANK_AX, K_SHANK * max(0.0, self.shank_in[i] - depth))
                if f_ax > 0.0:
                    _wrench(acc, "inside", pi, -f_ax * u, head)
                    _wrench(acc, "outside", po, f_ax * u, end)
                # the shank is a pin in the post's bore: held on the post's axis at the head and at the screw's tip,
                # so it keeps the base square on the posts as well as on them
                for pt in (head, tip_s):
                    rel_p = pt - end
                    rv = rel_p - (rel_p @ a_post) * a_post
                    on_axis = pt - rv
                    v_rel = (vi + np.cross(wi, pt - pi)) - (vo + np.cross(wo, on_axis - po))
                    v_rel = v_rel - (v_rel @ a_post) * a_post
                    f = -K_SHANK * rv - C_SHANK * v_rel
                    n = float(np.linalg.norm(f))
                    if n > F_SHANK_MAX:
                        f *= F_SHANK_MAX / n
                    _wrench(acc, "inside", pi, f, pt)
                    _wrench(acc, "outside", po, -f, on_axis)
            # the draw
            f_pull = 0.0
            if self.turns[i] > 0.0:
                post = po + Ro @ np.asarray(post_local)
                s = float((head - post) @ u)
                if self.s_catch[i] is None:
                    self.s_catch[i] = max(s, S_SEAT)
                f_pull = min(self.cap(i), K_DRAW * max(0.0, s - self.s_max(i)))
                if f_pull > 0.0 and not self.welded:
                    _wrench(acc, "inside", pi, -f_pull * u, head)
                    _wrench(acc, "outside", po, f_pull * u, post)
            self.last_pull[i] = f_pull
        if not self.welded and max(self.turns) >= TURNS_SNUG - 1e-9:
            self.welded = True


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _roll(Rd, Ri, axis):
    """The screwdriver's turn about the screw's axis, measured against the base: positive is clockwise seen from
    the handle (right-handed about the direction of travel)."""
    ref = Ri[:, 0] - (Ri[:, 0] @ axis) * axis
    v = Rd[:, 0] - (Rd[:, 0] @ axis) * axis
    if np.linalg.norm(v) < 1e-9 or np.linalg.norm(ref) < 1e-9:
        return 0.0
    ref /= np.linalg.norm(ref)
    v /= np.linalg.norm(v)
    return math.atan2(float(np.cross(ref, v) @ axis), float(ref @ v))


def _wrench(acc, name, origin, force, at):
    """Force (N, world) at a world point, as a force plus a torque about the body's origin (its centre of mass is
    added by the hook)."""
    f, t, o = acc.setdefault(name, [np.zeros(3), np.zeros(3), np.asarray(origin, float)])
    f += force
    t += np.cross(np.asarray(at, float) - o, force)


MODEL = Model()


# ---- simulator hook (GPU host only) ------------------------------------------------------------------------------
def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy(), dtype=np.float64)


def _quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def install(environment):
    """Run the model before every physics substep (Isaac Lab calls ``write_data_to_sim`` then) and apply its forces;
    once the set is snug, hold both halves where they were at snug (as merged #1562's fitting kit holds its welded
    parts). Idempotent per scene object."""
    import torch
    import warp as wp

    base = environment.unwrapped
    objs = {n: base.scene.rigid_objects[n] for n in NAMES}
    anchor = objs["driver"]
    if getattr(anchor, "_lockset_hooked", False):
        return
    orig = anchor.write_data_to_sim
    origin = _np(base.scene.env_origins)[0][:3]
    weld = {}

    def write():
        orig()
        frames, raw = {}, {}
        for n, o in objs.items():
            pose = _np(o.data.root_link_pose_w)[0]
            vel = _np(o.data.root_link_vel_w)[0]
            raw[n] = pose[:7].copy()
            frames[n] = (pose[:3] - origin, _quat_matrix(pose[3:7]), vel[:3], vel[3:6])
        acc = {}
        if not MODEL.welded:
            weld.clear()
        MODEL.update(frames, acc)
        dev = base.device
        if MODEL.welded:
            if not weld:
                weld.update({n: raw[n] for n in ("outside", "inside")})
            for n, pose in weld.items():
                objs[n].write_root_link_pose_to_sim_index(
                    root_pose=torch.tensor([pose.tolist()], dtype=torch.float32, device=dev))
                objs[n].write_root_link_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
                acc.pop(n, None)
        for n, (f, t, o_body) in acc.items():
            if not f.any() and not t.any():
                continue
            o = objs[n]
            com = _np(o.data.body_com_pos_w)[0].reshape(-1)[:3] - origin if hasattr(o.data, "body_com_pos_w") else o_body
            t_com = t + np.cross(o_body - com, f)  # move the torque from the origin to the centre of mass
            o.root_view.apply_forces_and_torques_at_position(
                force_data=wp.array(f.astype(np.float32), dtype=wp.float32, device=dev),
                torque_data=wp.array(t_com.astype(np.float32), dtype=wp.float32, device=dev),
                position_data=None, indices=o._ALL_INDICES, is_global=True)

    anchor.write_data_to_sim = write
    anchor._lockset_hooked = True
