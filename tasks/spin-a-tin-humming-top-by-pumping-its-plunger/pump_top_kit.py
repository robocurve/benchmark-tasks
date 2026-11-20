"""Tin humming top (Schylling Little Tin Tops, MTA) with a pump-down screw drive.

What it models: a free rigid tin shell (rotor) standing on a rounded plastic point, a
captive twisted rod with a knob that slides along the shell's axis, and the one-way
screw clutch between them. Pushing the knob down turns the shell through the rod's
screw; lifting it freewheels. Everything else (gravity, tip contact on the tray,
gyroscopic response, falling over) is left to PhysX.

Published: 127 mm body diameter, 139.7 mm listed height. ESTIMATES (not physical
observations): net mass 110 g split 99 g shell / 3 g bearing carrier / 8 g rod and
knob; shell inertia (1.3e-4, 1.3e-4, 2.2e-4) kg m2 with its centre of mass 70 mm above
the point; rounded point radius 4.8 mm (a retail photo shows the point as a round
button about 26 pixels wide where the 127 mm body is 342, so about 9.7 mm across and
half as tall); knob 28 mm across; housing top 112 mm above the point; rod travel
80 mm; screw lead 25 mm per turn (read from the twist count on a retail photo of the
extended rod); push needed to drive the screw 14 N in total; screw efficiency 0.7;
rod return drag 0.2 N; spin loss 2 rad/s per second; point and tray friction 0.25.

Tuned numbers (all of the above are inputs, none was tuned to make a run pass):
LEAD=.025, ETA=.70, FSCREW=13 N, FBEAR=.8 N, rod joint friction .2 N, DRAG=2 rad/s/s.

Screw clutch, applied before every 1/240 s physics substep (install()):
  u = downward rod speed relative to the shell, w = shell spin relative to the carrier,
  K = 2*pi/LEAD. The screw catches once the rod has come down 2 mm since it last rose
  by 1 mm; then, while u > 10 mm/s and K*u > w, it drives: torque
  tau = min(ETA*FSCREW/K, (K*u - w)*I/dt) on the shell and -tau on the carrier (I is
  the shell inertia when the knob is held still, the pair's reduced inertia when the
  knob turns freely), and an axial force K*tau/ETA + FBEAR pushing the rod up and the
  carrier down. Otherwise nothing (freewheel). Every force and torque is an equal and
  opposite pair between bodies of the top; its power never exceeds ETA times the
  hand's input power. The spin loss is an external torque about the shell's own axis
  only. No joint drive target, pose, velocity, upright constraint, added spin or pivot
  is ever written.

GPU notes: the articulation root must sit on the shell body (spawn_settings() moves
it), else PhysX roots the chain at the carrier and the shell spawns 100 mm low. The
knob must be convex meshes, not primitive cylinders. Without the 2 mm catch the rule
answered any creep of the 8 g rod with its full push-back and threw an unheld knob
77 mm out of the body. With a 2.5 mm point every top leaned over within 6 s of being
let go; with the 4.8 mm point read off the photo a top let go upright near 100 rad/s
spins alone about 14 s and leans past 20 degrees once it has slowed to 70-76 rad/s.
"""
from __future__ import annotations
import math
import numpy as np
from yam_parts import box, cyl, ball, Part
from yam_joints import Link, Hinge, Slide
from yam_mesh import MeshMechanism, revolve

LEAD = .025            # m per full turn, estimate
K = 2 * math.pi / LEAD  # rad per m of rod travel
ETA = .70
FSCREW = 13.           # N carried by the screw at full torque
FBEAR = .8             # N bearing loss while driving (plus the joint's .2 N friction)
TAU_MAX = ETA * FSCREW / K
DT = 1 / 240
I3 = .00022
IC = 5.6e-7            # carrier + rod + knob about the axis, from their shapes and masses
DRAG = 2.              # rad/s per second, estimated spin loss (air, point, bearing)
U_MIN = .001
U_ARM = .010           # m/s: slower creeping of the rod does not catch the screw
TIP = .0048            # m, radius of the rounded point (9.6 mm button in the retail photo)
TILT_DEG = 44.         # natural rest on point and rim for this profile
TOP_XY = (.120, .050)
RED, YELLOW, GREEN, BLUE, ORANGE = (.85, .06, .05), (.98, .80, .05), (.08, .62, .18), (.05, .35, .90), (.98, .45, .05)


def top(pos, name='top', friction=(.25, .25)):
    """The toy, rod collapsed (q=0). Bands are separate convex coloured regions."""
    bands = [  # (r0,z0) -> (r1,z1), colour
        ((TIP, TIP), (.0065, .0085), RED), ((.0065, .0085), (.020, .030), RED),
        ((.020, .030), (.052, .050), BLUE), ((.052, .050), (.0635, .060), BLUE),
        ((.0635, .060), (.0635, .066), GREEN), ((.0635, .066), (.060, .074), GREEN),
        ((.060, .074), (.052, .084), YELLOW), ((.052, .084), (.036, .092), BLUE),
        ((.036, .092), (.019, .096), ORANGE), ((.019, .096), (.019, .112), BLUE)]
    meshes = []
    for i, ((r0, z0), (r1, z1), c) in enumerate(bands):
        meshes.append(revolve([(0, z0), (r0, z0), (r1, z1), (0, z1)], samples=48, name=f'band_{i}',
                              collision='convex', color=c,
                              proxy=(cyl((0, 0, (z0 + z1) / 2), max(r0, r1), z1 - z0),)))
    carrier = Link('carrier', (cyl((0, 0, .104), .004, .012),), Hinge('z', (0, 0, .100), -1e6, 1e6), .003)
    rod = Link('rod', (box((0, 0, .086), (.005, .001, .080), color=(.75, .75, .78)),),
               Slide('z', (0, 0, .112), 0, .080, static_friction=.2, dynamic_friction=.2),
               .008, parent='carrier', friction=(.65, .55))
    # Knob as convex revolved meshes (primitive cylinders make unreliable contacts with the pads in this
    # harness): an 18 -> 28 mm red cone, then a 12 mm tall 28 mm band on top, which is what the pads hold.
    knob = [revolve([(0, .1125), (.009, .1125), (.014, .128), (0, .128)], samples=48, name='knob_cone',
                    collision='convex', color=RED, proxy=(cyl((0, 0, .12025), .014, .0155),)),
            revolve([(0, .128), (.014, .128), (.014, .140), (0, .140)], samples=48, name='knob_band',
                    collision='convex', color=RED, proxy=(cyl((0, 0, .134), .014, .012),))]
    # Point, plus the one ink dot a person adds to count turns on video.
    shapes = (ball((0, 0, TIP), TIP, color=RED), box((.046, 0, .0875), (.010, .010, .004), color=(1, 1, 1)))
    return MeshMechanism(name, pos, shapes, [carrier, rod], mass=.099, contact_offset=.001,
                         friction=friction, meshes={'base': meshes, 'rod': knob})


def tray(z):
    """IKEA TILLGANG tray 370 x 290 mm, 3 mm deck and 12 mm rim (deck and rim estimates)."""
    return Part('tray', (.195, .050, z + .0015), [
        box((0, 0, 0), (.370, .290, .003), color=(.30, .32, .34)),
        box((-.180, 0, .006), (.010, .290, .012), color=(.24, .26, .28)),
        box((.180, 0, .006), (.010, .290, .012), color=(.24, .26, .28)),
        box((0, -.140, .006), (.350, .010, .012), color=(.24, .26, .28)),
        box((0, .140, .006), (.350, .010, .012), color=(.24, .26, .28))], mass=.30,
        contact_offset=.001, friction=(.25, .25))


# Per mechanism name: com = author the estimated centre of mass and inertia; tilt = start leaning on point and rim.
SETTINGS = {'top': dict(com=True, tilt=True)}


def spawn_settings():
    """Spawn-time PhysX settings only (nothing is rewritten while it runs)."""
    import yam_joints as J
    if getattr(J._spawn_mechanism, '_pump', False):
        return
    original = J._spawn_mechanism

    def spawn(path, cfg, *args, **kw):
        prim = original(path, cfg, *args, **kw)
        opts = SETTINGS.get(cfg.mech_name)
        if opts is None:
            return prim
        from pxr import PhysxSchema, UsdPhysics, Gf
        stage = prim.GetStage()
        # Root the articulation on the shell. Rooted on the Xform, PhysX picks the middle link of the chain
        # shell - carrier - rod (the carrier) as the floating root, and the harness then places and reads the
        # top by the carrier's frame, 100 mm off (the fix GPU-proven by the daydot, case and deckbox probes).
        base = stage.GetPrimAtPath(f'{path}/base')
        prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        UsdPhysics.ArticulationRootAPI.Apply(base)
        art = PhysxSchema.PhysxArticulationAPI.Apply(base)
        art.CreateEnabledSelfCollisionsAttr().Set(False)
        art.CreateSleepThresholdAttr().Set(0.)
        art.CreateStabilizationThresholdAttr().Set(0.)
        pos_it, vel_it = opts.get('iters', (32, 8))
        art.CreateSolverPositionIterationCountAttr().Set(pos_it)
        art.CreateSolverVelocityIterationCountAttr().Set(vel_it)
        for name in ('base', 'carrier', 'rod'):
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f'{path}/{name}'))
            api.CreateEnableGyroscopicForcesAttr().Set(bool(opts.get('gyro', True)))
            api.CreateAngularDampingAttr().Set(0.)
            api.CreateLinearDampingAttr().Set(0.)
            api.CreateMaxAngularVelocityAttr().Set(18000.)  # deg/s, above any reachable spin
        if opts.get('com'):
            mass = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(f'{path}/base'))
            mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, .070))
            mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(.00013, .00013, I3))
        for name, vmax in (('carrier', 18000.), ('rod', 3.)):
            j = stage.GetPrimAtPath(f'{path}/joints/{name}_joint')
            PhysxSchema.PhysxJointAPI.Apply(j).CreateMaxJointVelocityAttr().Set(vmax)
        print('PUMP_SPAWN', cfg.mech_name, opts, 'gyro on, free point, lead', LEAD, flush=True)
        return prim
    spawn._pump = True
    J._spawn_mechanism = spawn

    # Start stopped and naturally tilted on point and rim (leaning toward +y).
    original_obj = J._mechanism_object

    def obj(spec):
        result = original_obj(spec)
        tilt = SETTINGS.get(spec.name, {}).get('tilt')
        if tilt:
            from isaaclab_arena.utils.pose import Pose
            from types import MethodType
            h = math.radians(TILT_DEG if tilt is True else float(tilt)) / 2
            q = (-math.sin(h), 0., 0., math.cos(h))
            pose = Pose(position_xyz=spec.pos, rotation_xyzw=q)
            result.initial_pose = pose
            result.get_initial_pose = MethodType(lambda self: pose, result)
            result.object_cfg.init_state.rot = q
            # The reset event was built with the upright pose when the object was made; build it again so a
            # reset without jitter also lays the top on its point and rim.
            result._pose_event_cfg = result._build_reset_event()
        return result
    J._mechanism_object = obj


class Clutch:
    """One-way screw clutch state and its per-stroke log."""
    def __init__(self, name='top'):
        self.name = name
        self.reset()

    def reset(self):
        self.work_in = 0.
        self.work_out = 0.
        self.last_tau = 0.
        self.last_force = 0.
        self.q_hi = None
        self.q_lo = None
        self.peak_force = 0.
        self.down = False
        self.stroke = None
        self.strokes = []

    def step(self, u, w, held, spin, q=None):
        inertia = I3 if held else 1 / (1 / I3 + 1 / IC)
        # The screw catches only once the rod has come down 2 mm since it last rose by a millimetre, and only above
        # 10 mm/s: a rod that creeps or rattles on its stop is left alone, so an unheld knob is never thrown.
        if q is not None:
            if self.q_hi is None:
                self.q_hi = self.q_lo = q
            self.q_lo = min(self.q_lo, q)
            if q > self.q_lo + .001:
                self.q_hi = self.q_lo = q
        armed = q is None or self.q_hi - q >= .002
        drive = armed and u > U_ARM and K * u > w
        tau = min(TAU_MAX, max(0., (K * u - w) * inertia / DT)) if drive else 0.
        force = K * tau / ETA + FBEAR if drive else 0.
        self.work_in += force * max(u, 0.) * DT
        self.work_out += tau * max(w, 0.) * DT
        self.last_tau, self.last_force = tau, force
        self.peak_force = max(self.peak_force, force)
        # Stroke log: a down stroke runs from u > 2 cm/s to u < -2 cm/s.
        if not self.down and u > .02:
            self.down = True
            self.stroke = dict(n=len(self.strokes) + 1, spin0=spin, umax=0., engaged=0., fmax=0., win=self.work_in,
                               travel=0.)
        if self.down:
            s = self.stroke
            s['umax'] = max(s['umax'], u)
            s['engaged'] += DT if drive else 0.
            s['fmax'] = max(s['fmax'], force)
            s['travel'] += max(u, 0.) * DT
            s['spin1'] = spin
            if u < -.02:
                self.down = False
                s['work'] = self.work_in - s['win']
                self.strokes.append(s)
                if s['travel'] > .01:
                    print(f"PUMP_STROKE n={s['n']} spin {s['spin0']:.2f} -> {s['spin1']:.2f} rad/s "
                          f"travel={s['travel'] * 1000:.1f} mm peak_u={s['umax']:.3f} m/s engaged={s['engaged']:.3f} s "
                          f"fmax={s['fmax']:.2f} N work={s['work']:.3f} J total_in={self.work_in:.3f} "
                          f"out={self.work_out:.3f}", flush=True)
        return tau, force


CLUTCHES = {}


def clutch(name='top'):
    if name not in CLUTCHES:
        CLUTCHES[name] = Clutch(name)
    return CLUTCHES[name]


CLUTCH = clutch('top')


def array(x):
    x = getattr(x, 'torch', x)
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def _axis(quat_xyzw):
    x, y, z, w = quat_xyzw
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def _composer_set(a, forces, torques, ids):
    comp = getattr(a, 'permanent_wrench_composer', None)
    if comp is not None:
        comp.set_forces_and_torques_index(forces=forces, torques=torques, body_ids=ids, is_global=True)
    else:
        a.set_external_force_and_torque(forces, torques, body_ids=ids, is_global=True)


def clear(env):
    """At a reset: no wrench left over from the previous episode."""
    for name in SETTINGS:
        clutch(name).reset()
        try:
            a = env.unwrapped.scene[name]
            comp = getattr(a, 'permanent_wrench_composer', None)
            if comp is not None:
                comp.reset()
        except Exception as e:  # noqa: BLE001
            print('PUMP_RESET', name, repr(e), flush=True)


def install(env):
    """Wrap each top's write_data_to_sim so its clutch acts before every physics substep."""
    for name in SETTINGS:
        _install_one(env, name)


def _install_one(env, name):
    a = env.unwrapped.scene[name]
    if getattr(a, '_pump_hook', False):
        return
    import torch
    names = list(a.data.joint_names)
    h, s = names.index('carrier_joint'), names.index('rod_joint')
    bodies = list(a.data.body_names)
    ids = [bodies.index('base'), bodies.index('carrier'), bodies.index('rod')]
    orig = a.write_data_to_sim
    state = {'n': 0, 'err': None}
    c = clutch(name)

    def write(*args, **kwargs):
        try:
            v = array(a.data.joint_vel)[0]
            axis = _axis(array(a.data.root_pose_w)[0, 3:7])
            spin = float(array(a.data.root_ang_vel_w)[0] @ axis)
            u = -float(v[s])           # rod moving down (toward the shell) is positive
            w = -float(v[h])           # shell spin relative to the carrier, drive sense
            carrier_spin = spin + float(v[h])
            tau, force = c.step(u, w, abs(carrier_spin) < 3., spin, float(array(a.data.joint_pos)[0][s]))
            drag = -I3 * DRAG * float(np.clip(spin / .5, -1, 1))
            F = np.zeros((1, 3, 3))
            T = np.zeros((1, 3, 3))
            T[0, 0] = (tau + drag) * axis
            T[0, 1] = -tau * axis
            F[0, 2] = force * axis
            F[0, 1] = -force * axis
            _composer_set(a, torch.tensor(F, dtype=torch.float32, device=a.device),
                          torch.tensor(T, dtype=torch.float32, device=a.device), ids)
            if state['n'] < 2:
                state['n'] += 1
                print(f'PUMP_HOOK ok {name} u={u:.4f} w={w:.3f} spin={spin:.3f}', flush=True)
        except Exception as e:  # noqa: BLE001
            if state['err'] is None:
                state['err'] = repr(e)
                print('PUMP_HOOK error', name, repr(e), flush=True)
        return orig(*args, **kwargs)
    a.write_data_to_sim = write
    a._pump_hook = True
    print('PUMP_HOOK installed:', name, 'internal wrench pairs per substep, no targets written', flush=True)
