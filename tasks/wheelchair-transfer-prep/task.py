"""On the floor in front of the two arms (on their pedestals): a folding manual wheelchair, open, facing the robots, as a
Drive Medical Silver Sport 2 (SSP218FA-SF: 18 in seat, fixed arms, swing-away footrests; a serial not starting 3J). It
stands free on its two 24 in rear wheels and two 8 in swivel casters, its two push-to-lock wheel locks off, each
fitted with the maker's 8 in wheel-lock extension handle (Drive STDS801GT, listed for the Silver Sport 1 and 2
excluding serial 3J: the lock's rubber tip removed, the extension slid over the handle), and its two swing-away
footrests latched forward.

The task: ready the chair for a transfer. Engage both wheel locks (push each lock's handle forward, towards the front
of the chair, until it snaps over centre and its shoe presses into the tyre) and release and swing both footrests
aside (push the front rigging lever to release it, then swing the footrest out sideways), in either order, without
rolling the chair away: the chair rolls freely until its wheels are locked, so a lock pushed on an unbraked chair, or
a footrest pushed, drives the chair unless the other hand holds it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import VISUAL_ONLY, Joint, Mesh, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.station import FLOOR_Z  # noqa: E402

SCENE = "station"
labkit.floor_bay()  # the station without its table: the arms on pedestals, the chair on the floor
FLOOR = FLOOR_Z

CHROME = (0.79, 0.80, 0.81)  # solid chrome-plated parts (the footrest sockets and hanger tubes): they collide
RIM = (0.78, 0.79, 0.80)  # handrims: drawn only (the tyres collide)
SHOE = (0.77, 0.78, 0.79)  # the lock shoe: drawn only; the wheel lock's grip on the tyre is the locked-wheel hold drive
#   (a brake proxy), not a rigid shoe pressed into a rigid tyre (codex 0034: CHROME was visual-only, so the footrest
#   sockets and hanger tubes had no colliders)
FRAME = (0.62, 0.64, 0.66)  # silver-vein painted steel frame
BLACK = (0.05, 0.05, 0.06)
VINYL = (0.08, 0.08, 0.09)
GREY = (0.30, 0.31, 0.33)
TYRE = (0.07, 0.07, 0.07)
RED = (0.75, 0.10, 0.10)
SPOKE = (0.70, 0.71, 0.72)
for _c in (SPOKE, RIM, SHOE):
    VISUAL_ONLY.add(_c)  # spokes, handrims and the lock shoes are drawn only

# -- the wheelchair. Frame: origin on the floor below the midpoint of the two front uprights (the footrests' pintles),
# x towards the chair's rear (away from the robots), y to the robots' left, z up. -----------------------------------
TIP_X = float(os.environ.get("WC_TIP_X", "0.07"))  # the footplates' front edges, 7 cm in front of the arms' base line (reach: 0 to 10 cm, README), 2 cm
#   clear of the pedestals' columns (front faces at x 0.05)
TIP_DX = 0.30  # the footplates' front edges 30 cm in front of the front uprights
CX = TIP_X + TIP_DX  # the chair's origin in the world (x)
SEAT_W, SEAT_D, SEAT_Z = 0.457, 0.406, 0.495  # 18 x 16 in seat, 19.5 in seat to floor
FRAME_Y = SEAT_W / 2 + 0.0125  # the side frames' tube centre lines (25 mm tubes)
PINTLE_X, PINTLE_Y = -0.035, 0.232  # the footrests' vertical pintles, on brackets in front of the front uprights
TUBE = 0.0125  # tube radius
AXLE_X, WHEEL_R, WHEEL_W = 0.40, 0.305, 0.032  # 24 in rear wheels, solid urethane tyres
WHEEL_Y = 0.300  # the tyres' centre planes (26 in over the handrims)
CASTER_X, CASTER_R, CASTER_W, CASTER_TRAIL = 0.03, 0.1016, 0.025, 0.045  # 8 in casters
ARM_Z = 0.760  # armrest pad tops (fixed full-length arms)
BACK_TOP = SEAT_Z + 0.406  # 16 in back height above the seat
HANDLE_Z = 0.914  # push handles, 36 in overall height
MASS_FRAME = 11.7  # kg: 18.40 kg modelled in all with the wheels, casters, locks and footrests (maker: 18.6 kg, 41 lb)
# Rolling resistance (joint friction; PhysX units, 8 N m each, measured on the 5090): about 3 to 4 N for the chair
# (0.3 N m on each rear wheel, 0.1 N m on each caster), which also holds a chair at rest (d1 to d3: without it the
# chair crept off on contact jitter)
FRICTION_NM = 8.0
WHEEL_FRICTION, CASTER_FRICTION = 0.30 / FRICTION_NM, 0.10 / FRICTION_NM

# Push-to-lock wheel lock (each side): a lever pivoting on the side frame ahead of the tyre, its stock tip 0.52 m up. Pushed
# forward (towards the front of the chair, -x) past dead centre it snaps over and its shoe presses 1/8 in into the tyre.
LOCK_PIVOT = (0.100, 0.266, 0.440)  # (x, |y|, z): on a bracket outboard of the side frame, ahead of the tyre and
#   inboard of it (every part of the lock at least 8 mm inside the tyre's inner face, y 0.284; s1: a lock overlapping
#   the tyre drove the wheel and the chair rolled off)
LOCK_ARM = 0.085  # pivot to the stock handle's tip (its rubber tip removed for the extension)
# The wheel-lock extension (Drive STDS801GT, 8 in): a tube slid over the stock handle, in line with it. Drive's sheet
# gives no dimensions beyond the 8 in length; the section is a retail listing's for the STDS801 (0.75 x 1.1 x 8.2 in),
# an estimate here.
EXT_LEN, EXT_W, EXT_D = 0.208, 0.019, 0.028  # length, across (y), fore and aft
EXT_R0 = 0.035  # its lower end this far up the handle from the pivot (it covers the handle's upper 5 cm)
EXT_R1 = EXT_R0 + EXT_LEN  # its upper end, 0.24 m from the pivot
GRIP_R = float(os.environ.get("WC_GRIP_R", "0.170"))  # where the hand holds it, 7 cm below its upper end (CPU IK: 0.14 to 0.19 m pointing 60 degrees down)
LOCK_TILT = math.radians(20.0)  # the lever leans back (towards the tyre) this far when off
LOCK_OVER = math.radians(-24.0)  # past this (forward) it snaps over centre
LOCK_END = math.radians(-42.0)  # and comes to rest here, locked
KNOB_R = EXT_W / 2  # the extension's half width (the offline checks' end marker)
LOCK_K = float(os.environ.get("WC_LOCK_K", "1.6"))  # N m/rad of the toggle spring: 0.67 N m to push it over centre,
#   about 8 N at the stock handle's tip, 3.4 N at the extension's grip point


# Diagnostic GPU coupon (codex 7:32 PM, phase_coupon1): WC_COUPON=1 starts with both wheel locks already locked
#   (a reconstructed, disclosed state: joints posed at their locked rest, not a native checkpoint) and runs the left
#   footrest's release, first carry, let-go and retreat; WC_COUPON=2 starts with the left footrest released and swung
#   WC_COUPON_FOOT degrees (reconstructed) and runs only the final regrip/carry to SWING_END, let-go and retreat.
COUPON = os.environ.get("WC_COUPON", "")
COUPON_FOOT = math.radians(float(os.environ.get("WC_COUPON_FOOT", "60")))
CAL_DEG = float(os.environ.get("WC_CAL_DEG", "45"))  # WC_COUPON=3: the released angle the calibration starts at


def _knob_xz(theta: float, r: float = EXT_R1 - 0.01) -> tuple[float, float]:
    """A point (x, z) on the lock lever r m from its pivot (default: the extension's upper end) for a lever angle theta
    (rad, positive towards the tyre)."""
    a = LOCK_TILT + theta
    return LOCK_PIVOT[0] + r * math.sin(a), LOCK_PIVOT[2] + r * math.cos(a)


# Swing-away footrest (each side, Drive STDS3J24SF type): a hanger on a vertical pintle at the front upright, a
# footplate at its lower front end; latched straight forward until its front rigging lever is pushed in, then it
# swings out sideways, up to 90 degrees.
HANGER_TOP_Z, PLATE_Z = 0.455, 0.105
PLATE = (0.13, 0.15, 0.008)  # footplate (x, y, thickness)
LEVER_Z = 0.410  # the release lever, on the hanger's front
LEVER_TRAVEL = 0.006  # m: pushed this far in (towards the rear) it releases the latch
SWING_OPEN = math.radians(80.0)  # swung at least this far out counts as aside
SWING_MAX = math.radians(90.0)


def _tyre(c, r, w, color, n):
    """A tyre as an n-sided prism about a y axis through c (radius r, width w), colliding as its convex hull: PhysX's
    cylinder shape jittered on the floor and the free chair crept off (d1 to d4)."""
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False) + math.pi / n  # a flat at the bottom: on a corner the chair
    #   rolled 2 cm at the start while the wheels settled onto a flat (r3 to r6)
    ring = np.stack([c[0] + r * np.cos(a), np.zeros(n), c[2] + r * np.sin(a)], axis=1)
    pts = np.vstack([ring + [0.0, c[1] - w / 2, 0.0], ring + [0.0, c[1] + w / 2, 0.0],
                     [[c[0], c[1] - w / 2, c[2]], [c[0], c[1] + w / 2, c[2]]]])
    tris = []
    for i in range(n):
        k = (i + 1) % n
        tris += [(i, k, n + i), (k, n + k, n + i), (2 * n, k, i), (2 * n + 1, n + i, n + k)]
    return Mesh(points=pts, tris=np.asarray(tris, int), color=color, collide="hull")


def _side(s: int) -> str:
    return "left" if s > 0 else "right"


def _rear_wheel(s: int) -> Joint:
    y = s * WHEEL_Y
    spokes = [((AXLE_X + 0.15 * math.cos(a), y, WHEEL_R + 0.15 * math.sin(a)), (0.004, 0.003, 0.29), SPOKE,
               (0.0, a + math.pi / 2)) for a in np.linspace(0, math.pi, 6, endpoint=False)]
    return Joint(f"wheel_{_side(s)}", kind="hinge", axis="Y", anchor=(AXLE_X, y, WHEEL_R), lower=-1e4, upper=1e4,
                 initial=0.0, stiffness=0.0, damping=0.05, target=0.0, mass=1.4, friction=WHEEL_FRICTION,
                 meshes=[_tyre((AXLE_X, y, WHEEL_R), WHEEL_R, WHEEL_W, TYRE, 64)],
                 cylinders=[((AXLE_X, y, WHEEL_R), 0.035, WHEEL_W + 0.01, GREY, "Y"),  # hub
                            ((AXLE_X, y + s * 0.025, WHEEL_R), 0.270, 0.008, RIM, "Y")],  # handrim
                 boxes=spokes)


def _caster(s: int) -> list[Joint]:
    y = s * (FRAME_Y - 0.005)
    stem = Joint(f"swivel_{_side(s)}", kind="hinge", axis="Z", anchor=(CASTER_X, y, 0.26), lower=-1e4, upper=1e4,
                 initial=0.0, stiffness=0.0, damping=0.05, target=0.0, mass=0.35, friction=CASTER_FRICTION,
                 boxes=[((CASTER_X + CASTER_TRAIL / 2, y, (0.26 + CASTER_R) / 2), (CASTER_TRAIL + 0.02, 0.05, 0.26 - CASTER_R),
                         GREY)])
    wheel = Joint(f"caster_{_side(s)}", kind="hinge", axis="Y", anchor=(CASTER_X + CASTER_TRAIL, y, CASTER_R),
                  lower=-1e4, upper=1e4, initial=0.0, stiffness=0.0, damping=0.02, target=0.0, mass=0.40,
                  friction=CASTER_FRICTION,
                  parent=f"swivel_{_side(s)}",
                  meshes=[_tyre((CASTER_X + CASTER_TRAIL, y, CASTER_R), CASTER_R, CASTER_W, BLACK, 32)])
    return [stem, wheel]


def _lock(s: int) -> Joint:
    px, py, pz = LOCK_PIVOT[0], s * LOCK_PIVOT[1], LOCK_PIVOT[2]
    kx, kz = _knob_xz(0.0, LOCK_ARM)
    mid = ((px + kx) / 2, py, (pz + kz) / 2)
    ex, ez = _knob_xz(0.0, (EXT_R0 + EXT_R1) / 2)
    # the shoe: a short block below the pivot, swinging back into the tyre as the handle goes forward (drawn; the
    # wheel's lock is the joint hold below)
    return Joint(f"lock_{_side(s)}", kind="hinge", axis="Y", anchor=(px, py, pz), lower=LOCK_END - 0.05, upper=0.05,
                 initial=0.0, stiffness=LOCK_K, damping=0.02, target=LOCK_END if COUPON else 0.0, mass=0.25,
                 boxes=[(mid, (0.012, 0.010, LOCK_ARM), BLACK, (0.0, LOCK_TILT)),
                        ((px + 0.012, py, pz - 0.025), (0.030, 0.014, 0.014), SHOE),
                        ((ex, py, ez), (EXT_D, EXT_W, EXT_LEN), RED, (0.0, LOCK_TILT))])  # the extension


def _footrest(s: int) -> list[Joint]:
    py = s * PINTLE_Y
    fx = -TIP_DX + PLATE[0] / 2  # the footplate's centre (x)
    top, bot = (PINTLE_X, 0.305), (-TIP_DX + PLATE[0], PLATE_Z + 0.01)  # the hanger tube's ends (x, z)
    tube_len = math.hypot(top[0] - bot[0], top[1] - bot[1])
    tube_tilt = math.atan2(top[0] - bot[0], top[1] - bot[1])  # leans back towards its top
    hanger = Joint(f"footrest_{_side(s)}", kind="hinge", axis="Z", anchor=(PINTLE_X, py, 0.30),
                   lower=-SWING_MAX if s > 0 else -0.02, upper=0.02 if s > 0 else SWING_MAX,
                   initial=0.0, stiffness=0.0, damping=0.4, target=0.0, mass=0.85,
                   boxes=[((PINTLE_X, py, 0.38), (0.026, 0.026, 0.15), CHROME),  # the hanger's socket on the pintle
                          (((top[0] + bot[0]) / 2, py, (top[1] + bot[1]) / 2), (0.022, 0.022, tube_len), CHROME,
                           (0.0, tube_tilt)),  # the hanger tube, sloping down and forward to the footplate
                          ((fx, py, PLATE_Z), PLATE, BLACK),
                          ((PINTLE_X + 0.005, py, LEVER_Z + 0.025), (0.030, 0.030, 0.010), GREY)])  # latch housing
    lever = Joint(f"release_{_side(s)}", kind="slide", axis="X", anchor=(PINTLE_X - 0.024, py, LEVER_Z), lower=0.0,
                  upper=LEVER_TRAVEL + 0.002, initial=0.0, stiffness=300.0, damping=2.0, target=0.0, mass=0.10,
                  parent=f"footrest_{_side(s)}",
                  boxes=[((PINTLE_X - 0.024, py, LEVER_Z), (0.014, 0.028, 0.040), RED)])  # the lever's paddle, facing forward
    return [hanger, lever]


def _frame_boxes():
    out = []
    for s in (1, -1):
        y = s * FRAME_Y
        out += [
            ((0.0, y, (0.27 + HANGER_TOP_Z + 0.02) / 2), (0.025, 0.025, HANGER_TOP_Z + 0.02 - 0.27), FRAME),  # front upright
            ((SEAT_D / 2, y, SEAT_Z - 0.01), (SEAT_D + 0.03, 0.025, 0.025), FRAME),  # seat rail
            ((0.22, y, 0.30), (0.46, 0.022, 0.022), FRAME),  # lower rail
            ((SEAT_D + 0.01, y, (0.30 + HANDLE_Z) / 2), (0.025, 0.025, HANDLE_Z - 0.30), FRAME),  # rear upright
            ((SEAT_D + 0.06, y, HANDLE_Z), (0.10, 0.030, 0.030), BLACK),  # push handle grip
            # the fixed arm: front post, pad
            ((0.06, y, (SEAT_Z + ARM_Z) / 2), (0.022, 0.022, ARM_Z - SEAT_Z), FRAME),
            ((0.20, y, ARM_Z - 0.02), (0.30, 0.055, 0.040), VINYL),
            ((AXLE_X, s * 0.250, WHEEL_R), (0.06, 0.04, 0.03), FRAME),  # axle plate, clear of the hub and tyre
            ((LOCK_PIVOT[0], s * (LOCK_PIVOT[1] - 0.012), LOCK_PIVOT[2]), (0.05, 0.012, 0.03), FRAME),  # lock bracket
        ]
    out += [
        ((SEAT_D / 2, 0.0, SEAT_Z), (SEAT_D, SEAT_W, 0.012), VINYL),  # sling seat
        ((SEAT_D + 0.02, 0.0, (SEAT_Z + BACK_TOP) / 2 + 0.02), (0.012, SEAT_W, BACK_TOP - SEAT_Z - 0.04), VINYL),  # back
        ((0.20, 0.0, 0.33), (0.025, 0.30, 0.020), FRAME),  # cross brace (folded X, drawn as a bar)
    ]
    return out


CHAIR = part(
    "wheelchair", pos=(CX, 0.0, FLOOR + 0.002), mass=MASS_FRAME, friction=0.6,
    boxes=_frame_boxes(),
    joints=[_rear_wheel(1), _rear_wheel(-1), *_caster(1), *_caster(-1), _lock(1), _lock(-1), *_footrest(1),
            *_footrest(-1)],
)
OBJECTS = [CHAIR]


# Footrest hinge friction (codex 9:01 PM, option A): a passive model, parameters ESTIMATED, not measured: mu 0.2 (chrome
#   steel on steel or nylon, dry, 0.1 to 0.3) x (9.3 N hanger weight on a 10.2 mm effective thrust radius + 18.6 N
#   radial pin load from the overhung COM on a 6.35 mm pin) = 0.043 N m nominal (0.021 to 0.064). Saturating viscous
#   form: full HINGE_TF above HINGE_W0. Without it the frictionless hinge crept back 0.35 deg/s under a 0.09 deg chair
#   tilt (r18, r19). Derivation: robocurve-review/wheelchair-transfer-prep/hinge-friction-derivation-20261119.md.
HINGE_TF = float(os.environ.get("WC_HINGE_TF", "0"))  # N m: the on_step law, OFF (it chattered at 0.043 and 0.064, frick/fricl)
HINGE_MU = float(os.environ.get("WC_HINGE_MU", "0.040"))  # native PhysX joint friction coefficient (dimensionless, scaled by
#   cal3 (both footrests released, hands parked, ramps both ways): breakaway torque ~ mu + 0.003 N m at 45 deg (0.033
#   at 0.03, 0.053 at 0.05; ~0.045 at 85 deg for 0.03), so the predeclared 0.021/0.043/0.064 N m are mu 0.018/0.040/0.061;
#   the joint reaction load, applied every physics substep); calibrated against torque by WC_COUPON=3 (codex 2101:
#   prefer native friction to the 20 Hz held-effort law; frick/fricl: at 0.043 and 0.064 that law chattered, logged
#   omega 0.039 rad/s at full tau while the angle crept 0.3 deg/s)
HINGE_W0 = 0.02  # rad/s


# -- mechanics: the locks' over-centre toggle, the wheels they hold, the footrests' latches -----------------------------
_mech = {}
WHEEL_HOLD = 400.0  # N m/rad on a locked rear wheel


@on_reset
def _reset(env):
    _mech.clear()
    _mech.update(locked={"left": False, "right": False}, hold={}, latched={"left": True, "right": True},
                 start=None, hist=[], clear_hist=[], far=False, still=False, viol=set(), n=0)
    _stage_from.clear()
    _seen.update(left=None, right=None)
    if os.environ.get("WC_DEBUG") and not _mech.get("_audited"):  # which chair shapes the stage actually gives colliders
        _mech["_audited"] = True
        try:
            import omni.usd
            from pxr import Usd, UsdPhysics

            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath("/World/envs/env_0/wheelchair")
            rows = {}
            for prim in Usd.PrimRange(root):
                if prim.GetTypeName() in ("Cube", "Cylinder", "Mesh"):
                    body = str(prim.GetPath()).split("/wheelchair/")[-1].split("/")[0]
                    k = rows.setdefault(body, [0, 0])
                    k[0 if prim.HasAPI(UsdPhysics.CollisionAPI) else 1] += 1
            print("[col] collision prims per chair body (collide, drawn only): "
                  + " ".join(f"{b}={c}/{v}" for b, (c, v) in sorted(rows.items())), file=sys.stderr, flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[col] audit failed: {e}", file=sys.stderr, flush=True)
    _track_from.clear()
    _chair_seen[:] = 0.0
    if COUPON:
        _mech["locked"] = {"left": True, "right": True}
        _mech["hold"] = {"left": 0.0, "right": 0.0}
    if HINGE_MU > 0.0:
        asset = env.unwrapped.scene["wheelchair"]
        fids = [list(asset.joint_names).index(_jn(f"footrest_{side}")) for side in ("left", "right")]
        import torch

        mu = torch.full((1, len(fids)), HINGE_MU, dtype=torch.float32, device=asset.device)
        asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=mu, joint_dynamic_friction_coeff=mu.clone(),
                                                            joint_ids=fids)
        print(f"[fric] reset: native joint friction mu={HINGE_MU} on footrest hinges {fids}", file=sys.stderr, flush=True)
    if COUPON == "3":  # calibration: both footrests released at 45 deg, hands parked; a slow torque ramp finds breakaway
        _mech["latched"] = {"left": False, "right": False}
        import torch

        asset = env.unwrapped.scene["wheelchair"]
        fids = [list(asset.joint_names).index(_jn(f"footrest_{side}")) for side in ("left", "right")]
        cal = math.radians(CAL_DEG)
        pos = torch.tensor([[-cal, cal]], dtype=torch.float32, device=asset.device)
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=fids)
        rb = getattr(asset.data, "joint_friction_coeff", None)
        try:
            rb = labkit._np(rb)[0, fids] if rb is not None else "n/a"
        except Exception as e:  # noqa: BLE001
            rb = f"unreadable ({e})"
        print(f"[cal] reset: footrests written to -{CAL_DEG}/+{CAL_DEG} deg, latched flags {_mech['latched']}, "
              f"mu set {HINGE_MU}, readback {rb}, hands parked (LEFT_PARK/RIGHT_PARK)", file=sys.stderr, flush=True)
    if COUPON == "2":  # the left footrest released and swung: set its joint state directly (no drive target holds it)
        _mech["latched"] = {"left": False, "right": False}
        import torch

        asset = env.unwrapped.scene["wheelchair"]
        ids = [list(asset.joint_names).index(_jn("footrest_left"))]
        pos = torch.tensor([[-COUPON_FOOT]], dtype=torch.float32, device=asset.device)
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
        print(f"[cpn] reset: footrest_left joint state written to {-math.degrees(COUPON_FOOT):.1f} deg", file=sys.stderr,
              flush=True)


def _look_footrests(env):
    """What the demo may see of the footrests: each footplate's yaw against the chair's (body poses, as a camera
    would see them; no joint readings)."""
    asset = env.unwrapped.scene["wheelchair"]
    names = list(asset.body_names)
    q = labkit._np(asset.data.body_link_quat_w)[0]
    ib = names.index("body")
    xyzw = abs(q[ib][3]) >= abs(q[ib][0])  # the chair stands about upright and square: its scalar part is the big one

    def yaw(i):
        x, y, z, w = (float(v) for v in (q[i] if xyzw else np.roll(q[i], -1)))
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    base = yaw(ib)
    for side in ("left", "right"):
        d = yaw(names.index(f"footrest_{side}")) - base
        _seen[side] = math.atan2(math.sin(d), math.cos(d))


def _jn(short):
    return next(j.joint_name for j in CHAIR.joints if j.name == short)


@on_step
def _mechanics(env):
    import torch

    st = labkit._part_state(env, CHAIR)
    _look_footrests(env)
    _chair_seen[:] = np.asarray(st.pos, float) - np.array([CX, 0.0, FLOOR + 0.002])  # where the chair is seen now
    global _chair_yaw_seen
    _chair_yaw_seen = _yaw(st)
    if os.environ.get("WC_NOMECH"):  # diagnostic: no lock, latch or wheel-hold drives
        _gate(env, st)
        return
    j = st.joints
    asset = env.unwrapped.scene["wheelchair"]
    names = list(asset.joint_names)
    tgt, stiff = {}, {}
    for side in ("left", "right"):
        th = j[f"lock_{side}"]
        engaged = th <= LOCK_OVER  # over centre now (codex 0034: reversible, from the actual handle angle)
        if engaged and not _mech["locked"][side]:
            _mech["locked"][side] = True
            _mech["hold"][side] = j[f"wheel_{side}"]
        elif not engaged and _mech["locked"][side]:
            _mech["locked"][side] = False  # pulled back past centre: it springs off and the wheel is free again
        tgt[f"lock_{side}"] = LOCK_END if _mech["locked"][side] else 0.0
        stiff[f"lock_{side}"] = LOCK_K
        if _mech["locked"][side]:
            tgt[f"wheel_{side}"], stiff[f"wheel_{side}"] = _mech["hold"][side], WHEEL_HOLD
        else:
            tgt[f"wheel_{side}"], stiff[f"wheel_{side}"] = 0.0, 0.0
        sw = j[f"footrest_{side}"]
        pressed = j[f"release_{side}"] >= LEVER_TRAVEL * 0.5
        if _mech["latched"][side] and pressed:
            _mech["latched"][side] = False
        if not _mech["latched"][side] and abs(sw) < math.radians(2.0) and not pressed:
            _mech["latched"][side] = True  # swung back home, the latch catches again
        tgt[f"footrest_{side}"], stiff[f"footrest_{side}"] = 0.0, (200.0 if _mech["latched"][side] else 0.0)
        tgt[f"release_{side}"], stiff[f"release_{side}"] = 0.0, 300.0
    ids = [names.index(_jn(n)) for n in tgt]
    asset.set_joint_position_target(torch.tensor([[tgt[n] for n in tgt]], dtype=torch.float32, device=asset.device),
                                    joint_ids=ids)
    asset.write_joint_stiffness_to_sim(torch.tensor([[stiff[n] for n in tgt]], dtype=torch.float32,
                                                    device=asset.device), joint_ids=ids)
    # passive pin/thrust friction on each footrest hinge (a declared MODEL with estimated parameters, see HINGE_TF):
    #   tau = -clip(c * omega, +/-HINGE_TF), both directions, no target angle
    if HINGE_TF > 0.0:
        fids = [names.index(_jn(f"footrest_{side}")) for side in ("left", "right")]
        w = labkit._np(asset.data.joint_vel)[0, fids]
        tau = np.clip(-HINGE_TF / HINGE_W0 * w, -HINGE_TF, HINGE_TF)
        asset.set_joint_effort_target(torch.tensor([tau], dtype=torch.float32, device=asset.device), joint_ids=fids)
        if os.environ.get("WC_DEBUG") and _mech.get("n", 0) % 5 == 0:
            print(f"[fric] t={_sim_time():6.2f}s step={_mech.get('n', 0)} (on_step, 20 Hz; effort held 12 substeps) "
                  f"omega=({w[0]:+.4f},{w[1]:+.4f}) rad/s tau=({tau[0]:+.4f},{tau[1]:+.4f}) N m", file=sys.stderr,
                  flush=True)
    if COUPON == "3" and _sim_time() >= 2.0:
        # 2-4 s: zero torque (baseline); 4-14 s: outward ramp 0 -> 0.10 N m; 14-16 s: zero; 16-26 s: inward ramp 0 -> 0.10
        fids = [names.index(_jn(f"footrest_{side}")) for side in ("left", "right")]
        tt = _sim_time()
        tr = 0.0 if tt < 4.0 else 0.01 * (tt - 4.0) if tt < 14.0 else 0.0 if tt < 16.0 else -0.01 * min(tt - 16.0, 10.0)
        asset.set_joint_effort_target(torch.tensor([[-tr, tr]], dtype=torch.float32, device=asset.device), joint_ids=fids)
        if _mech.get("n", 0) % 5 == 0:
            wv = labkit._np(asset.data.joint_vel)[0, fids]
            print(f"[cal] t={_sim_time():6.2f}s tau={tr:+.4f} N m (+ outward) footL={math.degrees(j['footrest_left']):.3f} "
                  f"footR={math.degrees(j['footrest_right']):.3f} omega=({wv[0]:+.4f},{wv[1]:+.4f}) mu={HINGE_MU} "
                  f"latched={_mech['latched']}", file=sys.stderr, flush=True)
    _gate(env, st)


# -- stillness witness (WC_WITNESS_MM; a test of the scorer, never part of a scored trial) ------------------------------
# From 30 steps before WC_WITNESS_END (default 2400) on, the right footrest's joint state is written a little further in
# each step, so its footplate mark P2 creeps WC_WITNESS_MM mm per second towards forward (still well over 80 degrees
# aside). Locks, hands and chair are untouched, so this isolates the stillness gate: 6 mm must score 0, 3 mm 1.0.
WITNESS_MM = float(os.environ.get("WC_WITNESS_MM", "0") or 0)
WITNESS_END = int(os.environ.get("WC_WITNESS_END", "2400"))


@on_step
def _witness(env):
    if WITNESS_MM <= 0 or _mech.get("n", 0) < WITNESS_END - 30:
        return
    import torch

    asset = env.unwrapped.scene["wheelchair"]
    jid = list(asset.joint_names).index(_jn("footrest_right"))
    r = abs(-TIP_DX + 0.02 - PINTLE_X)  # the P2 mark's radius from the pintle
    a = float(labkit._np(asset.data.joint_pos)[0, jid]) - (WITNESS_MM / 1000.0 / r) / 20.0
    pos = torch.tensor([[a]], dtype=torch.float32, device=asset.device)
    asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=[jid])
    if _mech["n"] % 5 == 0:
        print(f"[witness] n={_mech['n']} footR={math.degrees(a):.3f} deg spread={_mech.get('spread', float('nan')) * 1000:.2f} mm "
              f"still={_mech.get('still')}", file=sys.stderr, flush=True)


# -- release and stillness (every positive score) ---------------------------------------------------------------------
RELEASE = 0.080
STILL_N, STILL_TOL = 21, 0.005  # 10 tape marks, each under 5 mm over the final 1 s (a ruler in a video shows 5 mm)
MOVE_TOL, TURN_TOL = 0.050, math.radians(5.0)  # the chair may end at most 5 cm and 5 degrees from where it stood


def _tips(env):
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q7, q8 = (abs(float(v)) for v in robot.data.joint_pos.torch[0, ids].cpu().numpy())
        out += [mid, mid - q7 * g[:3, 0], mid + q8 * g[:3, 0]]
    return out


MARKS = ("F1 left front upright", "F2 right front upright", "F3 left push-handle tip", "F4 right push-handle tip",
         "W1 left rear tyre", "W2 right rear tyre", "L1 left lock extension tip", "L2 right lock extension tip",
         "P1 left footplate front", "P2 right footplate front")


def _marks(st) -> np.ndarray:
    """The 10 tape marks of the stillness check (world, m), in MARKS order; README draws them.
    F1/F2: a dot on the outer face of each front upright at seat height. F3/F4: the end of each push handle.
    W1/W2: a dot on the outer side wall of each rear tyre, 2 cm in from the tread, at the tyre's front at the start
    (it turns with the wheel). L1/L2: the top end of each wheel-lock extension (it swings with the lock).
    P1/P2: the centre of each footplate's front edge (it swings with the footrest)."""
    j = st.joints
    pts = [st.to_world(p) for p in ((0.0, 0.25, SEAT_Z), (0.0, -0.25, SEAT_Z), (SEAT_D, 0.25, HANDLE_Z),
                                    (SEAT_D, -0.25, HANDLE_Z))]
    rw = WHEEL_R - 0.02
    for s in (1, -1):
        w = j[f"wheel_{_side(s)}"]
        pts.append(st.to_world((AXLE_X - rw * math.cos(w), s * (WHEEL_Y + WHEEL_W / 2), WHEEL_R + rw * math.sin(w))))
    for s in (1, -1):
        kx, kz = _knob_xz(j[f"lock_{_side(s)}"])
        pts.append(st.to_world((kx, s * LOCK_PIVOT[1], kz)))
    for s in (1, -1):
        a = j[f"footrest_{_side(s)}"]
        lx = -TIP_DX + 0.02 - PINTLE_X  # the footplate's front, from the pintle
        pts.append(st.to_world((PINTLE_X + math.cos(a) * lx, s * PINTLE_Y + math.sin(a) * lx, PLATE_Z)))
    return np.asarray(pts, float)


def _still(hist) -> tuple[bool, float]:
    """True when there are STILL_N finite samples and EVERY mark's largest distance between any two of its samples is
    under STILL_TOL. Returns (still, the worst mark's spread in m). NaN or a short window fails closed."""
    h = np.asarray(hist, float)
    if len(hist) != STILL_N or h.ndim != 3 or not np.isfinite(h).all():
        return False, float("inf")
    spread = float(np.linalg.norm(h[:, None] - h[None, :], axis=3).max())
    return spread < STILL_TOL, spread


def _clearance(st, tips) -> float:
    """Smallest distance from a hand point to the chair: its boxes (frame, seat, back, arms), wheels, locks and
    footrests, as axis-aligned boxes around each in the chair's frame (conservative)."""
    try:
        tips = np.asarray(tips, float)
    except (TypeError, ValueError):
        return 0.0
    if tips.ndim != 2 or tips.shape[0] == 0 or tips.shape[1] != 3 or not np.isfinite(tips).all():
        return 0.0  # invalid hand data fails closed (codex 0113: a NaN tip read as 9 m clear)
    best = 9.0
    for t in tips:
        q = np.asarray(st.to_local(t), float)
        if q.shape != (3,) or not np.isfinite(q).all():
            return 0.0
        boxes = [((SEAT_D / 2, 0.0, 0.55), (SEAT_D / 2 + 0.06, FRAME_Y + 0.03, 0.40)),  # frame, seat, back, arms
                 *[((AXLE_X, s * WHEEL_Y, WHEEL_R), (WHEEL_R, 0.035, WHEEL_R)) for s in (1, -1)],
                 *[((LOCK_PIVOT[0], s * LOCK_PIVOT[1], 0.56), (0.11, 0.03, 0.13)) for s in (1, -1)],  # lock + extension
                 *[((CASTER_X + CASTER_TRAIL / 2, s * FRAME_Y, 0.15), (CASTER_TRAIL + CASTER_R, CASTER_R, 0.15))
                   for s in (1, -1)]]  # casters (wheel swept about the stem) and their stems
        for s in (1, -1):
            a = st.joints[f"footrest_{_side(s)}"]
            c, sn = math.cos(a), math.sin(a)
            r = (TIP_DX + PINTLE_X) / 2  # half the hanger's reach forward of its pintle
            fx, fy = PINTLE_X - c * r, s * PINTLE_Y - sn * r
            boxes.append(((fx, fy, 0.28), (r + 0.05, r + 0.05, 0.20)))
        for c0, h in boxes:
            d = np.maximum(np.abs(q - np.asarray(c0)) - np.asarray(h), 0.0)
            best = min(best, float(np.linalg.norm(d)))
    return best


def _coupon_log(env, st):
    """Raw evidence each 0.2 s: footrest joint and seen yaw, paddle travel, jaw openings, TCP and pad-tip positions, the
    paddle's world position and their distance, lock angles and flags, chair displacement."""
    from yam_arena import runtime as _rt

    j = st.joints
    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    parts = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q7, q8 = (float(v) for v in labkit._np(robot.data.joint_pos)[0, ids])
        g = _rt.link_poses(robot)["gripper"]
        tcp = g[:3, :3] @ np.array([0.0, 0.0, 0.134]) + g[:3, 3] - o
        mid = _rt.pad_tips_world(robot) - o
        parts.append(f"{n[:1]} jaw=({q7 * 1000:.1f},{q8 * 1000:.1f})mm tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) "
                     f"tip=({mid[0]:.4f},{mid[1]:.4f},{mid[2]:.4f})")
    a = j["footrest_left"]
    pad = st.to_world((PINTLE_X + (PAD_X - PINTLE_X + j["release_left"]) * math.cos(a),
                       PINTLE_Y + (PAD_X - PINTLE_X + j["release_left"]) * math.sin(a), LEVER_Z))
    ar = j["footrest_right"]
    padr = st.to_world((PINTLE_X + (PAD_X - PINTLE_X + j["release_right"]) * math.cos(ar),
                        -PINTLE_Y + (PAD_X - PINTLE_X + j["release_right"]) * math.sin(ar), LEVER_Z))
    parts.append(f"padR=({padr[0]:.4f},{padr[1]:.4f},{padr[2]:.4f}) relR={j['release_right'] * 1000:.2f}mm "
                 f"latchedR={_mech['latched']['right']} chair_yaw={math.degrees(_yaw(st)):.3f}")
    d, _ = _moved(st)
    print(f"[cpn] t={m_t():6.2f}s footL={math.degrees(a):.2f} seenL={math.degrees(_seen['left'] or 0):.2f} "
          f"footR={math.degrees(j['footrest_right']):.2f} relL={j['release_left'] * 1000:.2f}mm "
          f"latched={_mech['latched']} lockL={math.degrees(j['lock_left']):.1f} lockR={math.degrees(j['lock_right']):.1f} "
          f"locked={_mech['locked']} chair={d * 1000:.1f}mm pad=({pad[0]:.4f},{pad[1]:.4f},{pad[2]:.4f}) "
          + " ".join(parts), file=sys.stderr, flush=True)


def m_t():
    return _mech.get("n", 0) / 20.0


def _gate(env, st):
    m = _mech
    if m["start"] is None:
        m["start"] = (np.asarray(st.pos, float).copy(), _yaw(st))
    m["clear_hist"].append(_clearance(st, _tips(env)))
    del m["clear_hist"][:-STILL_N]
    m["far"] = (len(m["clear_hist"]) == STILL_N and all(math.isfinite(c) for c in m["clear_hist"])
                and min(m["clear_hist"]) >= RELEASE)  # clear over the whole window, finite values only
    m["hist"].append(_marks(st))
    del m["hist"][:-STILL_N]
    m["still"], m["spread"] = _still(m["hist"])  # the 10 tape marks, fail closed on NaN
    if st.tilt_deg() > 10.0:
        m["viol"].add("the chair tipped")
    m["n"] += 1
    if (COUPON or (os.environ.get("WC_DEBUG") and m["n"] >= 560)) and m["n"] % 5 == 0:
        _coupon_log(env, st)  # every 0.25 s from the footrest phase (28 s) on
    if os.environ.get("WC_DEBUG") and m["n"] % 20 == 0:
        d, yaw = _moved(st)
        j = st.joints
        print(f"[wc] t={m['n'] / 20:5.1f}s moved={d * 1000:.1f} mm yaw={math.degrees(yaw):.2f} deg "
              f"locks L={math.degrees(j['lock_left']):.1f} R={math.degrees(j['lock_right']):.1f} "
              f"locked={m['locked']} footrests L={math.degrees(j['footrest_left']):.1f} "
              f"R={math.degrees(j['footrest_right']):.1f} latched={m['latched']} far={m['far']} still={m['still']}",
              file=sys.stderr, flush=True)
        tp = _tips(env)
        for s, side, mid in ((1, "left", tp[0]), (-1, "right", tp[3])):
            kx, kz = _knob_xz(j[f"lock_{side}"])
            kn = st.to_world((kx, s * LOCK_PIVOT[1], kz))
            pad = st.to_world((PINTLE_X - 0.024, s * PINTLE_Y, LEVER_Z))
            print(f"[wc3] t={m['n'] / 20:5.1f}s {side} tip=({mid[0]:.3f},{mid[1]:.3f},{mid[2]:.3f}) "
                  f"knob=({kn[0]:.3f},{kn[1]:.3f},{kn[2]:.3f}) d_knob={np.linalg.norm(mid - kn) * 1000:.0f} mm "
                  f"d_paddle={np.linalg.norm(mid - pad) * 1000:.0f} mm", file=sys.stderr, flush=True)
        p = np.asarray(st.pos, float)
        print(f"[wc2] t={m['n'] / 20:5.1f}s pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) tilt={st.tilt_deg():.2f} "
              + " ".join(f"{k}={j[k]:.2f}" for k in ("wheel_left", "wheel_right", "swivel_left", "swivel_right",
                                                     "caster_left", "caster_right")), file=sys.stderr, flush=True)


def _yaw(st) -> float:
    R = st.rotation()
    return math.atan2(R[1, 0], R[0, 0])


def _moved(st) -> tuple[float, float]:
    p0, y0 = _mech["start"]
    d = np.asarray(st.pos, float) - p0
    return float(math.hypot(d[0], d[1])), float(abs((_yaw(st) - y0 + math.pi) % (2 * math.pi) - math.pi))


# -- scoring ----------------------------------------------------------------------------------------------------------
def score(objects):
    """0 if the chair ever tipped more than 10 degrees; 0 unless, over the last 21 control steps (1 s), every gripper jaw
    and pad point stayed at least 80 mm from the chair and each of the 10 tape marks (MARKS) moved less than 5 mm
    (largest 3-D distance over every pair of its samples), with all values finite; 0 if the chair reference point (the
    midpoint of the two front uprights at floor level) ended more than 5 cm, or its yaw more than 5 degrees, from where
    it started. Otherwise 0.25 for each, as it is at the end: the left wheel lock engaged (handle past centre), the
    right wheel lock engaged, the left footrest at least 80 degrees aside, the right footrest at least 80 degrees
    aside. Possible scores: 0, 0.25, 0.5, 0.75, 1.0."""
    st = objects["wheelchair"]
    m = _mech
    if m.get("start") is None or m["viol"]:
        return 0.0
    if not (m["far"] and m["still"]):
        return 0.0
    d, yaw = _moved(st)
    if d > MOVE_TOL or yaw > TURN_TOL:
        return 0.0
    j = st.joints
    done = [j["lock_left"] <= LOCK_OVER, j["lock_right"] <= LOCK_OVER, j["footrest_left"] <= -SWING_OPEN,
            j["footrest_right"] >= SWING_OPEN]  # each as it is NOW (a reopened lock simply earns nothing)
    return 0.25 * sum(bool(x) for x in done)


SUCCESS_HOLD_S = 2.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return 1.0 - score(objects)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both wheel locks first, each with the other hand bracing the chair: the right hand holds the right armrest's front
# post while the left hand pinches the left lock's extension and pulls it forward along its arc, over centre; then the
# same mirrored. Then each hand pinches its footrest's release paddle from the sides, pushes it in, swings the footrest
# out 90 degrees about its pintle, lets go and lifts off. The task file lets a Waypoint's ``yaw`` carry a full gripper rotation (``Orient``).
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _down_grip(yaw)

    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation

PITCH = math.radians(60.0)  # the brace grip points forward and 30 degrees down: a level grip this low is out
#   of reach (IK: none at 90, 22 to 25 mm off at 75, exact to 60 degrees), and the wrist stays in front of the armrest
FWD = Orient(axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -PITCH) @ _down_grip(0.0))  # fingers closing along y
FWD_DIR = np.array([math.sin(PITCH), 0.0, -math.cos(PITCH)])  # the way the fingertips point


def down(yaw: float) -> Orient:
    return Orient(_down_grip(yaw))


TIP = 0.013  # fingertips this far beyond the TCP along the pointing direction
LEFT_PARK = (0.12, 0.30, 0.95)
RIGHT_PARK = (0.12, -0.30, 0.95)
PAD_OPEN, PAD_PINCH = -0.024, 0.0  # round the 28 mm release paddle's sides; closed on them
BRACE = -0.002  # fingers together, pushing with the fingertips


def W(x, y, z):
    """Chair coordinates (at the chair's start) to world."""
    return (CX + x, y, FLOOR + z)


def _fwd_tcp(tip_world, back: float = 0.0):
    """The TCP for a forward-pitched grip whose fingertips are at ``tip_world``, or ``back`` m short of it."""
    return tuple(np.asarray(tip_world, float) - (TIP + back) * FWD_DIR)


# The lock hand comes in from outboard, over the tyre and under the armrest pad's outer edge, pointing inboard and
# GRIP_DOWN below level, and pinches the extension tube fore and aft (the jaws across it, rolled with it), then pulls it
# forward along its arc past centre. Hooking the stock 22 mm knob from behind (r3 to r8) slid off it at 8 of the
# 24 degrees needed.
GRIP_DOWN = math.radians(float(os.environ.get("WC_GRIP_DOWN", "60")))
GRIP_YAW = math.radians(float(os.environ.get("WC_GRIP_YAW", "15")))  # turned this far to point forward too
GRIP_DY = float(os.environ.get("WC_GRIP_DY", "-0.016"))  # the TCP this far outboard of the tube's centre line: the
#   steep fingers cross the tube's 19 mm width on the way down, so the TCP sits 2 mm inboard of its centre and the
#   fingertips 1 mm short of its inner face (r9: 8 mm outboard, only the fingertips touched it and slid off)
EXT_OPEN, EXT_PINCH = -0.030, 0.0  # round the 28 mm tube; closed on it
JAW_SQ = float(os.environ.get("WC_JAW_SQ", "0.25"))  # 0.5 and up (square to the pull) are out of reach


def _lock_rot(s: int, theta: float) -> tuple[Orient, np.ndarray]:
    """The lock hand's rotation and pointing direction (world) on the extension at lever angle ``theta``."""
    a = LOCK_TILT + theta
    u = np.array([math.sin(a), 0.0, math.cos(a)])  # along the tube, up
    c = math.cos(GRIP_DOWN)
    z = np.array([c * math.sin(GRIP_YAW), -s * c * math.cos(GRIP_YAW), -math.sin(GRIP_DOWN)])  # inboard and down
    x0 = np.cross(u, z)
    x0 /= np.linalg.norm(x0)  # the jaws across the tube, about 30 deg off the pull
    x = (1.0 - JAW_SQ) * x0 + JAW_SQ * s * np.array([u[2], 0.0, -u[0]])  # turned JAW_SQ of the way to square
    x /= np.linalg.norm(x)  # with the pull (r10: 30 deg off it wedged the tube out past the fingertips at 6 deg)
    z = z - float(z @ x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return Orient(np.column_stack([x, y, z])), z


def _grip_tcp(s: int, theta: float, back: float = 0.0):
    """The TCP holding the extension at its grip point at lever angle ``theta``, or ``back`` m short of that along the
    hand's pointing direction (outboard and up)."""
    gx, gz = _knob_xz(theta, GRIP_R)
    d = _lock_rot(s, theta)[1]
    return tuple(np.asarray(W(gx, s * (LOCK_PIVOT[1] + GRIP_DY), gz), float) - back * d)


def _knob_off(s: int):
    """Clear of the locked extension: 8 cm back out the way the hand came in."""
    return _grip_tcp(s, LOCK_END, 0.08)


def _unlock_seq(s: int) -> list[Waypoint]:
    """Control only (WC_CONTROL=unlock): pinch the locked extension and pull it back past centre to off, let go."""
    side = _side(s)
    rE = _lock_rot(s, LOCK_END)[0]
    out = [Waypoint(2.0, _grip_tcp(s, LOCK_END, 0.08), EXT_OPEN, f"outboard of the {side} lock's extension, to unlock",
                    rE),
           Waypoint(1.5, _grip_tcp(s, LOCK_END), EXT_OPEN, "in round the locked extension", rE),
           Waypoint(0.6, _grip_tcp(s, LOCK_END), EXT_PINCH, "pinch it", rE)]
    for k in range(1, 7):
        th = LOCK_END * (1 - k / 6)
        out.append(Waypoint(0.7, _grip_tcp(s, th), EXT_PINCH, "pull it back, off", _lock_rot(s, th)[0]))
    out += [Waypoint(0.5, _grip_tcp(s, 0.0), EXT_OPEN, "let go", _lock_rot(s, 0.0)[0]),
            Waypoint(1.2, _grip_tcp(s, 0.0, 0.08), EXT_OPEN, "back out", _lock_rot(s, 0.0)[0])]
    return out


def _lock_seq(s: int) -> list[Waypoint]:
    """Pinch the extension and pull it forward along its arc past centre to its locked rest, let go and back out."""
    side = _side(s)
    r0, rE = _lock_rot(s, 0.0)[0], _lock_rot(s, LOCK_END)[0]
    out = [Waypoint(2.0, _grip_tcp(s, 0.0, 0.08), EXT_OPEN, f"outboard of the {side} lock's extension", r0),
           Waypoint(1.5, _grip_tcp(s, 0.0), EXT_OPEN, "in round the extension", r0),
           Waypoint(0.6, _grip_tcp(s, 0.0), EXT_PINCH, "pinch it", r0)]
    for k in range(1, 7):
        th = (LOCK_END - 0.04) * k / 6
        out.append(Waypoint(0.7, _grip_tcp(s, th), EXT_PINCH, "pull it forward, over centre", _lock_rot(s, th)[0]))
    out += [Waypoint(0.5, _grip_tcp(s, LOCK_END), EXT_PINCH, "settle", rE),
            Waypoint(0.5, _grip_tcp(s, LOCK_END), EXT_OPEN, "settle, open", rE),
            Waypoint(1.2, _knob_off(s), EXT_OPEN, "back out", rE)]
    return out


POST_X, POST_Z = 0.06, 0.62  # the armrest's front post (chair x) and where on it the bracing hand grips (z)
POST_OPEN, POST_PINCH = -0.022, 0.0  # round the 22 mm tube; closed on it


def _brace(s: int, dur: float) -> list[Waypoint]:
    """Hold the chair still by its armrest's front post for ``dur`` seconds: in with the fingers either side of the
    tube, close, hold, open and back off. (Pushing on the armrest instead rolled the free chair 10 cm away: r1.)"""
    post = W(POST_X, s * FRAME_Y, POST_Z)
    return [Waypoint(2.0, _fwd_tcp(post, 0.05), POST_OPEN, "in front of the armrest's front post", FWD),
            Waypoint(1.2, _fwd_tcp(post), POST_OPEN, "round the post", FWD),
            Waypoint(0.8, _fwd_tcp(post), POST_PINCH, "hold the chair by it", FWD),
            Waypoint(dur, _fwd_tcp(post), POST_PINCH, "hold it", FWD),
            Waypoint(0.6, _fwd_tcp(post), POST_OPEN, "let go", FWD),
            Waypoint(1.0, _fwd_tcp(post, 0.05), POST_OPEN, "off the post", FWD)]


PAD_X = PINTLE_X - 0.024  # the release paddle's centre (chair x)
STAGE_DX = float(os.environ.get("WC_STAGE_DX", "-0.004"))  # regrip stages (no press) pinch near the paddle centre: 13 mm
#   forward, the open fingers came down on the paddle's front face and pushed it in 7.8 mm before closing, so the jaws
#   closed off-centre (rel2: jaws 5.7/33.9 mm) and lost it
PRESS_LOW = float(os.environ.get("WC_PRESS_LOW", "0.0"))  # the release press and first carry: mid-height, as r12 to
#   r15 (both latches let go natively every run)
PAD_LOW = float(os.environ.get("WC_PAD_LOW", "0.016"))  # fingertips this far below the paddle's mid-height (4 mm
#   above its lower edge): the fingers taper, so the part beside the latch housing (just above the paddle) is the one
#   35 mm up, whose jaw faces stand 15.7 mm off centre, clear of the 30 mm housing (coupon1: pinched at mid-height
#   and 11 mm rearward of plan, the press jammed, paddle travel 0)
PAD_PRESS = 0.009  # pushed in this far (r6, r12 to r15: both latches let go; at 7 mm with the pinch 20 mm forward, r16, r17: neither)
PAD_DX = float(os.environ.get("WC_PAD_DX", "-0.013"))  # the TCP this far in front of the paddle's centre: r12 to r19 released
#   with targets in the spawn frame while the chair sat 12 to 16 mm rearward of it ([trk] lines, frice), i.e. the pinch
#   was 13 mm forward of the paddle centre; with chair tracking, centred jammed on the latch housing (frice, fricf).
#   Earlier: (r16, r17: 20 mm forward, the
#   sim hand stopped on the latch housing, 23 to 36 mm off the paddle; the socket hit that moved it was the checker
#   drawing the jaws shut through the paddle)


# The paddle hand points straight down to pinch and push the paddle in (r6, r9: both latches let go; tipped, r7 and r8,
# it never pushed it in), fingers across the paddle's sides, then tips back 15 and leans 20 degrees over the first two
# fifths of the swing: straight down, the forearm met the armrest's front post and a fingertip the seat rail's front end
# from about 60 degrees of swing (offline clash check; r9 slipped off both paddles at 55 to 59); tipped, nothing.
PAD_PITCH, PAD_LEAN = math.radians(15.0), math.radians(20.0)
PINCH_LEAN = math.radians(float(os.environ.get("WC_PINCH_LEAN", "30")))  # leaned this far, the wrist outboard: keeps
#   the forearm off the locked lock extension (CPU: 0 to 20 deg, 2 to 9 mm into it)


def _pad_rot(s: int, swing: float, f: float = 0.0, lean: float | None = None) -> tuple[Orient, np.ndarray]:
    """The paddle hand's rotation and pointing direction, ``f`` (0 to 1) of the way from straight down to tipped,
    leaned ``lean`` (default PINCH_LEAN) wrist-outboard."""
    lean = PINCH_LEAN if lean is None else lean
    R = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -s * (f * PAD_LEAN + lean)) @ axis_angle_matrix(
        np.array([0.0, 1.0, 0.0]), -f * PAD_PITCH) @ _down_grip(swing)
    return Orient(R), R[:, 2].copy()


def _pad_tcp(s: int, swing: float, press: float, lift: float = 0.0, f: float = 0.0, lean: float | None = None,
             low: float | None = None, dx: float | None = None):
    """The TCP with the fingertips at the paddle (fingers across its sides) with the footrest swung ``swing`` rad and
    the paddle pushed in ``press`` m, or ``lift`` m above that, the hand ``f`` of the way to tipped."""
    r = PAD_X - PINTLE_X + press + (PAD_DX if dx is None else dx)  # the TCP's distance in front of the pintle (< 0)
    x = PINTLE_X + r * math.cos(swing)
    y = s * PINTLE_Y + r * math.sin(swing)
    low = PAD_LOW if low is None else low
    return tuple(np.asarray(W(x, y, LEVER_Z - low + lift), float) - TIP * _pad_rot(s, swing, f, lean)[1])


def _footrest_seq(s: int) -> list[Waypoint]:
    side = _side(s)
    # upright to pinch and press (r16: leaned 30 degrees, the tilted jaws held only the paddle's top corners and never
    #   pushed it in), leaning over during the first carry, clear of the locked extension (CPU)
    up = 0.0
    out = [Waypoint(2.0, _pad_tcp(s, 0.0, 0.0, 0.06, lean=up, low=PRESS_LOW), PAD_OPEN, f"above the {side} footrest's release paddle",
                    _pad_rot(s, 0.0, lean=up)[0]),
           Waypoint(1.5, _pad_tcp(s, 0.0, 0.0, lean=up, low=PRESS_LOW), PAD_OPEN, "down round the paddle", _pad_rot(s, 0.0, lean=up)[0]),
           Waypoint(0.8, _pad_tcp(s, 0.0, 0.0, lean=up, low=PRESS_LOW), PAD_PINCH, "pinch it", _pad_rot(s, 0.0, lean=up)[0]),
           Waypoint(1.0, _pad_tcp(s, 0.0, PAD_PRESS, lean=up, low=PRESS_LOW), PAD_PINCH, "push it in: the latch lets go",
                    _pad_rot(s, 0.0, lean=up)[0])]
    # upright, start the swing only past the latch's 2-degree catch (leaning over while holding it, the forearm went
    #   9 mm into the locked extension at 10 to 20 degrees of lean), let go and lift; the stages carry on leaned
    a1 = -s * SWING_PINCH
    out += [Waypoint(0.9, _pad_tcp(s, a1, PAD_PRESS, lean=up, low=PRESS_LOW), PAD_PINCH, "start the swing by the paddle",
                     _pad_rot(s, a1, lean=up)[0]),
            Waypoint(0.6, _pad_tcp(s, a1, PAD_PRESS, lean=up, low=PRESS_LOW), PAD_OPEN, "let go of the paddle",
                     _pad_rot(s, a1, lean=up)[0]),
            Waypoint(1.0, _pad_tcp(s, a1, PAD_PRESS, 0.11, lean=up, low=PRESS_LOW), PAD_OPEN, "lift off the paddle",
                     _pad_rot(s, a1, lean=up)[0])]
    return out


SWING_PINCH = math.radians(float(os.environ.get("WC_FIRST", "18")))  # the first carry, upright (r13 to r15, to 40:
#   it reached 36 to 37)
STAGE = math.radians(30.0)  # each later carry: re-pinch the paddle where the footrest is now, carry it this much further
SWING_END = math.radians(float(os.environ.get("WC_END", "95")))  # the last carry's end (at 95, CPU: the fingertip went 8 to
#   10 mm into the front upright)
CATCH_CLEAR = math.radians(5.0)  # seen swung past this (the latch catches within 2 degrees), the paddle needs no press
N_STAGES = 4
LAG = math.radians(float(os.environ.get("WC_LAG", "4")))  # off the sim (CPU checks): the footrest ends a carry this far
#   short of the hand (r13 to r15: 36 to 37 of 40)


# Then carry the footrest on in stages, each from where it is SEEN to be (r3 to r13: carried by the paddle in one go,
# the pinch slipped at 50 to 73 degrees; r14, r15: a push on the hanger tube by the same hand put its forearm across
# the footrest; A2, the other hand, out of reach). The footrest's angle is the footplate's yaw against the chair's,
# both visible poses; each stage fixes it when it starts and carries from there.
_seen = {"left": None, "right": None}  # the footrests' current angles (rad), from on_step
_stage_from = {}  # (side, k) -> the angle that stage fixed at its start


_chair_yaw_seen = 0.0
_chair_seen = np.zeros(3)  # the chair's seen displacement from its spawn pose (m, world), from its visible pose
_track_from = {}  # waypoint id -> the chair displacement fixed when that waypoint's move began


class _Track:
    """A chair-relative waypoint target that follows where the chair is SEEN: the chair displacement is fixed when the
    move toward this waypoint begins (t0) and added to the nominal target (translation only; the chair's yaw stays
    under 0.3 degrees). Off the sim, or before t0, the current view (zero on the CPU). (codex 8:42 PM: plan in the
    observed settled chair frame; r16 to r19 and coupons 1 to 3: open-loop targets in the spawn frame met the paddle
    and lock extension off-centre after the chair settled.)"""

    def __init__(self, inner, t0, key):
        self.inner, self.t0, self.key = inner, t0, key

    def __array__(self, dtype=None, copy=None):
        off, dyaw = _track_fix(self.key, self.t0)
        if TRACK_YAW:  # the full planar transform: rotate about the chair's seen origin, then translate
            c, sn = math.cos(dyaw), math.sin(dyaw)
            o = np.array([CX, 0.0, FLOOR + 0.002])
            v = np.asarray(self.inner, float) - o
            v = np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1], v[2]])
            return np.asarray(o + v + off, dtype=dtype or float)
        return np.asarray(np.asarray(self.inner, float) + off, dtype=dtype or float)


class _TrackRot(Orient):
    """A hand rotation turned by the chair's seen yaw, fixed with the same grasp key as its position."""

    def __new__(cls, inner, t0, key):
        obj = float.__new__(cls, 0.0)
        obj.inner, obj.t0, obj.key = inner, t0, key
        return obj

    @property
    def rot(self):
        _, dyaw = _track_fix(self.key, self.t0)
        return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), dyaw) @ self.inner.rot


TRACK_YAW = os.environ.get("WC_TRACK_YAW", "1") == "1"  # rel1: the chair turned 1.3 to 1.5 deg during the right lock
#   grasp and 3 to 5 deg during the first footrest carry; translation-only tracking left the jaws 1 to 7 mm off


def _track_fix(key, t0):
    """The chair displacement and yaw fixed for this grasp (fixed on first use at or after t0)."""
    if key not in _track_from and _mech.get("n", 0) > 0 and _sim_time() >= t0:
        _track_from[key] = (_chair_seen.copy(), _chair_yaw_seen)
        if os.environ.get("WC_DEBUG"):
            print(f"[trk] t={_sim_time():6.2f}s {key} fixed chair offset ({_chair_seen[0] * 1000:.1f},"
                  f"{_chair_seen[1] * 1000:.1f},{_chair_seen[2] * 1000:.1f}) mm yaw={math.degrees(_chair_yaw_seen):.3f} deg",
                  file=sys.stderr, flush=True)
    return _track_from.get(key, (_chair_seen, _chair_yaw_seen if TRACK_YAW else 0.0))




_APPROACH = ("outboard of", "in front of", "above the")  # the first move of each grasp


def _tracked(wps, arm):
    """Wrap every chair-relative waypoint (all but the parks) of one arm's plan in _Track. The chair displacement is
    fixed ONCE per grasp, when its approach move begins, and held for every waypoint of that grasp (fricc, fricd: fixed
    per waypoint, each push moved the chair and the next target followed it: the chair ran 80 to 490 mm)."""
    out, t, key, t_key = [], 0.0, None, 0.0
    for i, w in enumerate(wps):
        pos = w.pos
        if any(w.label.startswith(a) for a in _APPROACH) or key is None:
            key, t_key = (arm, i), t
        yaw = w.yaw
        if not any(np.allclose(np.asarray(pos, float), np.asarray(p, float)) for p in (LEFT_PARK, RIGHT_PARK)):
            pos = _Track(pos, t_key, key)
            if TRACK_YAW and isinstance(yaw, Orient):
                yaw = _TrackRot(yaw, t_key, key)
        out.append(Waypoint(w.duration, pos, w.finger, w.label, yaw))
        t += w.duration
    return out


def _sim_time():
    return _mech.get("n", 0) / 20.0


class _Stage:
    """Stage k's start angle for footrest s: fixed at the first look on or after ``t0`` (the stage's start, s); before
    that, or off the sim, the planned value (the previous carry's end less LAG)."""

    def __init__(self, s, k, t0):
        self.s, self.k, self.t0 = s, k, t0

    def nominal(self):
        prev = SWING_PINCH if self.k == 0 else min(SWING_END, _Stage(self.s, self.k - 1, 0).nominal() + LAG + STAGE)
        return prev - LAG

    def start(self) -> float:
        side = _side(self.s)
        key = (side, self.k)
        if key in _stage_from:
            return _stage_from[key]
        seen = _seen[side]
        if seen is not None and _sim_time() >= self.t0:
            _stage_from[key] = abs(seen)
            return _stage_from[key]
        return self.nominal()

    def end(self) -> float:
        return min(SWING_END, self.start() + STAGE) if self.k < N_STAGES - 1 else SWING_END


class _PadAt:
    """A paddle TCP following a stage: at its start angle, or ``frac`` of the way to its end; pushed in ``press``,
    ``lift`` m up."""

    def __init__(self, st, frac=0.0, lift=0.0, off=(0.0, 0.0, 0.0)):
        self.st, self.frac, self.lift, self.off = st, frac, lift, np.asarray(off, float)

    def angle(self):
        a0 = self.st.start()
        return -self.st.s * (a0 + self.frac * (self.st.end() - a0))

    def __array__(self, dtype=None, copy=None):
        press = PAD_PRESS if self.st.start() < CATCH_CLEAR or os.environ.get("WC_STAGE_PRESS") else 0.0  # (CPU: the 9 mm
        #   press offset on every stage put a
        #   fingertip 5 to 6 mm into the hanger socket)
        return np.asarray(np.asarray(_pad_tcp(self.st.s, self.angle(), press, self.lift, dx=STAGE_DX)) + self.off,
                          dtype=dtype or float)


class _PadRot(Orient):
    def __new__(cls, at):
        obj = float.__new__(cls, 0.0)
        obj.at = at
        return obj

    @property
    def rot(self):
        return _pad_rot(self.at.st.s, self.at.angle())[0].rot


def _stages_seq(s: int, t0: float) -> list[Waypoint]:
    """Footrest ``s`` carried on from where it is seen, in N_STAGES re-pinches of the paddle, starting at ``t0``."""
    out, t = [], t0
    for k in range(N_STAGES):
        seq = _stage_seq(s, k, t)
        out += seq
        t += _dur(seq)
    return out


def _stage_seq(s: int, k: int, t0: float) -> list[Waypoint]:
    side = _side(s)
    if True:
        st = _Stage(s, k, t0)
        a, b = _PadAt(st, 0.0, 0.11), _PadAt(st, 0.0)
        m, e, up = _PadAt(st, 0.5), _PadAt(st, 1.0), _PadAt(st, 1.0, 0.11)
        seq = [Waypoint(1.2, a, PAD_OPEN, f"above the {side} paddle, stage {k + 1}", _PadRot(a)),
               Waypoint(1.0, b, PAD_OPEN, f"round the {side} paddle", _PadRot(b)),
               Waypoint(0.6, b, PAD_PINCH, f"pinch the {side} paddle", _PadRot(b)),
               Waypoint(0.9, m, PAD_PINCH, f"carry the {side} footrest on", _PadRot(m)),
               Waypoint(0.9, e, PAD_PINCH, f"carry the {side} footrest on", _PadRot(e)),
               Waypoint(0.6, e, PAD_OPEN, f"let go of the {side} paddle", _PadRot(e)),
               Waypoint(0.8, up, PAD_OPEN, f"up off the {side} paddle", _PadRot(up))]
    if k == N_STAGES - 1:  # out sideways, away from the chair, before parking (r18: from 0.11 m up the park move's
        #   forearm swept both footrests back from 90 to 75 and 83 degrees; straight up meets the locked extension, CPU)
        out1 = _PadAt(st, 1.0, 0.11, (0.0, s * 0.12, 0.0))
        out2 = _PadAt(st, 1.0, 0.25, (-0.05, s * 0.15, 0.0))
        seq += [Waypoint(1.0, out1, PAD_OPEN, f"out sideways from the {side} footrest", _PadRot(out1)),
                Waypoint(1.0, out2, PAD_OPEN, f"out and up from the {side} footrest", _PadRot(out2))]
    return seq


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    if os.environ.get("WC_CONTROL") == "untouched":
        return {"left": [Waypoint(20.0, LEFT_PARK, FINGER_OPEN, "rest")],
                "right": [Waypoint(20.0, RIGHT_PARK, FINGER_OPEN, "rest")]}
    if COUPON == "1":
        left = [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "start")] + _footrest_seq(1)
        left += [Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "retreat"), Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "observe")]
        return {"left": left, "right": [Waypoint(_dur(left), RIGHT_PARK, FINGER_OPEN, "parked")]}
    if COUPON == "3":
        return {"left": [Waypoint(27.0, LEFT_PARK, FINGER_OPEN, "parked")],
                "right": [Waypoint(27.0, RIGHT_PARK, FINGER_OPEN, "parked")]}
    if COUPON == "2":
        left = [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "start")]
        left += _stage_seq(1, N_STAGES - 1, _dur(left))
        left += [Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "retreat"), Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "observe")]
        right = [Waypoint(_dur(left), RIGHT_PARK, FINGER_OPEN, "parked")]
        if os.environ.get("WC_TRACK", "1") == "1":  # targets in the observed chair frame (codex 2042)
            left, right = _tracked(left, "left"), _tracked(right, "right")
        return {"left": left, "right": right}
    left, right = [], []
    # 1. left lock, the right hand bracing
    lk = _lock_seq(1)
    right += _brace(-1, _dur(lk) - 4.6 + 0.5)
    left += [Waypoint(4.0, LEFT_PARK, FINGER_OPEN, "wait for the brace")] + lk
    t = max(_dur(left), _dur(right))
    left.append(Waypoint(max(0.5, t - _dur(left)), _knob_off(1), EXT_OPEN, "wait", _lock_rot(1, LOCK_END)[0]))
    right.append(Waypoint(max(0.5, t - _dur(right)), _fwd_tcp(W(POST_X, -FRAME_Y, POST_Z), 0.05), POST_OPEN, "wait",
                          FWD))
    # 2. right lock, the left hand bracing
    rk = _lock_seq(-1)
    left += _brace(1, _dur(rk) - 4.6 + 0.5)
    right += [Waypoint(4.0, RIGHT_PARK, FINGER_OPEN, "wait for the brace")] + rk
    t = max(_dur(left), _dur(right))
    left.append(Waypoint(max(0.5, t - _dur(left)), LEFT_PARK, FINGER_OPEN, "back"))
    right.append(Waypoint(max(0.5, t - _dur(right)), RIGHT_PARK, FINGER_OPEN, "back"))
    if os.environ.get("WC_CONTROL") == "unlock":  # control: the right lock is pulled back off again (expect 0.75)
        uk = _unlock_seq(-1)
        right += uk + [Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "back")]
        left.append(Waypoint(_dur(uk) + 2.0, LEFT_PARK, FINGER_OPEN, "wait"))
    # 3. both footrests, one hand each (the wheels are locked now): release and start by the paddle, then carry on
    #   in stages from where each is seen
    left += _footrest_seq(1)
    right += _footrest_seq(-1)
    left += _stages_seq(1, _dur(left))
    right += _stages_seq(-1, _dur(right))
    left.append(Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"))
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    t = max(_dur(left), _dur(right)) + float(os.environ.get("WC_HOLD", "2.0"))  # hands off this long at the end
    left.append(Waypoint(t - _dur(left), LEFT_PARK, FINGER_OPEN, "hold"))
    right.append(Waypoint(t - _dur(right), RIGHT_PARK, FINGER_OPEN, "hold"))
    if os.environ.get("WC_TRACK", "1") == "1":
        left, right = _tracked(left, "left"), _tracked(right, "right")
    return {"left": left, "right": right}
