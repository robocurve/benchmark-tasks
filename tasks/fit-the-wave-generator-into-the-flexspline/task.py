"""A robot-joint assembly bench. A black printed assembly cradle holds a housed size-14 strain-wave gearbox
(StepperOnline HSS-14-50-I-D8, 73 mm flange) input side up, its flange bolted to the top of the cradle's riser. In
the gearbox's open input bore the dark steel circular spline and the bright steel flexspline cup are already mounted;
the flexspline's thin toothed rim is round and empty. The wave generator that belongs in it stands on a silver parts
tray beside the cradle on its 14 mm hub: a 30.4 mm steel plug (an elliptical cam inside a thin bearing), fixed by its
set screws on a short 8 mm steel shaft (the stand-in for the motor shaft) that carries a black printed handle and a
20 mm square grip on top. At the back of the bench are the joint's stepper motor on its printed adapter plate and the
joint's printed cover.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import parts  # noqa: E402  (before lots: its reset hooks run before the lot moves)
import lots  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
MM = 0.001
EPS = 1e-6
SUCCESS_HOLD_S = 0.0  # the one-second still hold is measured inside score()

# ---- the gearbox on its cradle (housing frame: origin on the gear axis at bench level, mm) -----------------------
# Maker's drawing (HSS-14-50-I-D8 dimension drawing): 73 flange, 34 from the output face to the input face, 36
# overall, 38 h7 input register, 56 h7 output pilot, 8 x 4.5 holes on a 65 circle, output 6 x M4 on a 23 circle with an
# 11 H7 bore; the drawing's 8.5 +/- 0.2 dimension (generator A) is read as the generator's inner (hub) end below the input
# face, 4.1 clear of the face 21.4 above the output face. Values marked "estimate" are not on the drawing.
Z_TOP = 60.0  # the gearbox's input face (top of its flange) above the bench
UNIT_L = 34.0  # input face to output face (drawing)
FLANGE_R, FLANGE_H = 36.5, 7.0  # 73 flange (drawing); thickness read off the drawing, which is not to scale: estimate
BODY_R = 28.0  # 56 h7 output pilot (drawing)
SPIGOT_R, SPIGOT_H = (16.5, 19.0), 2.0  # 38 h7 register round the input bore; bore and height estimates
INNER = Z_TOP - UNIT_L + 21.4  # 47.4: the face inside the flexspline cup that the generator's hub end clears (drawing)
CS_R = (17.8, 25.0)  # circular spline: tooth tips to the housing (estimates)
CS_Z = (52.0, 60.0)
FS_BORE_PHONE = 31.0  # mm: obtained by phone from a technician who works with this product (Nov 2026)
FS_R = (FS_BORE_PHONE / 2, 17.7)  # bore from phone figure; rim outside estimate
FS_Z = (INNER, 60.0)  # cup open end level with the input face (estimate)
CHAMFER = 1.0  # mm lead-in at the flexspline's open end (estimate)
CRADLE = 10.0  # mm base plate of the printed cradle
RISER_R = (28.25, 42.0)  # printed riser: 56.5 bore centres the output pilot, 84 outside; flange bolted on top
RISER_TOP = Z_TOP - FLANGE_H

# ---- the declared mesh rule (fidelity text) ------------------------------------------------------------------------
INSTALL = 8.5  # mm: drawing (+/- 0.2 for A), also obtained by phone from a technician who works with this product (Nov 2026)
INSTALL_TOL = 0.2  # mm (drawing)
RUNOUT_CUTOFF = 0.3  # mm TIR: unvalidated physical diagnostic estimate; never inferred from a successful demo
RUNOUT_BASELINE = 0.1  # mm TIR: proposed correct-seat enrolment maximum, not a measured baseline
FS_TEETH_ASSUMED = 100  # 2 * 50 ratio, assuming a two-tooth difference; tooth count not specified in supplied drawing
PITCH_DIAMETER_EST = 35.0  # mm: estimate within the drawing's 38 mm input register
ONE_TOOTH_TIR_EST = 4 * PITCH_DIAMETER_EST / FS_TEETH_ASSUMED  # 1.4 mm hypothetical TIR, not diagnostic validation
MOUTH = 1.5  # mm: the plug's chamfer and the rim's lead-in go in freely; the teeth zone starts here
FEED = 4.0  # mm of insertion per full turn of the generator (either way) while it presses on the teeth zone
LOAD_MIN = 10.0  # N: lower end of 10-30 N insertion load obtained by phone from a technician who works with this product (Nov 2026)
F_DED = 20.0  # N: unvalidated benchmark proxy trigger for a 0.25 s straight press; not a measured tooth-jump force
F_TILT = 8.0  # N: unvalidated tilted-press proxy trigger, not a measured physical failure force
TILT_MAX = 3.0  # degrees: benchmark alignment limit, not a measured HSS binding angle
W_TURN = 0.3  # rad/s: slower than this is not turning
LOAD_MAX = 30.0  # N, 0.25 s mean: benchmark force limit, not a bearing-damage rating
E_MAX = 1.0  # mm: plug centre off the gear axis (the rim allows 0.3 mm)
T_DRAG = 0.041  # N m: turning drag on the generator in the mesh, equal and opposite on the gearbox. Harmonic Drive's
#   maximum no-load starting torque for a size-14, ratio-50 CSF housed unit (CSF/CSG catalogue p. 46, Table 38, 4.1 N cm);
#   StepperOnline publishes no figure for the HSS-14, so this sibling value is used
CONTACT_LOAD_MIN = 0.05  # N: numerical contact margin for drag, below the resting generator weight (estimate)
MESH_MIN_R = 11.0  # mm: innermost contact radius of the mesh stop
MESH_MU = 0.4  # artificial brake-face coefficient: >= T_DRAG / (LOAD_MIN * MESH_MIN_R * MM) = 0.373.
# A contact-friction proxy, not oiled-steel friction: its minimum resisting capacity at 10 N is 0.044 N m.
# PhysX solves friction impulses passively, including stopping after release; no held external torque is applied.
W_SLIP = 0.2  # rad/s: diagnostic moment sign smoothing only (not a runtime force)
CONTACT_SEPARATION = 0.05 * MM  # m: numerical touch tolerance, 1/6 of the 0.3 mm nominal radial clearance
COLLISION_OFFSET = 0.20 * MM  # m: narrow speculative margin; credit still requires <=0.05 mm reported separation
CONTACT_WINDOW = 0.25  # s: time-weighted normal-force window, sampled every completed physics step
CONTACT_CAPACITY = 2048  # detailed contact points for one generator against the stop and the housing
K_STOP, M_STOP = 10000.0, 0.05
PARK = 8.0  # mm: stop position limit (travel)

STEEL, BRIGHT, DARK, BLACK = (0.62, 0.64, 0.67), (0.80, 0.81, 0.83), (0.24, 0.25, 0.27), (0.08, 0.08, 0.09)
ALU_C, CRADLE_C, ALU = (0.70, 0.71, 0.73), (0.12, 0.12, 0.13), (0.74, 0.75, 0.77)
QZ = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))  # turns a prism's x axis to z


def ring(r_in, r_out, z0, z1, color, n=16, **kw):
    """n boxes whose inner faces make a regular n-gon of inradius r_in (mm) about the axis, z0..z1 (mm); m out."""
    thick = r_out - r_in
    w = 2 * r_out * math.tan(math.pi / n) * 1.02
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = r_in + thick / 2
        out.append(parts.box((thick * MM, w * MM, (z1 - z0) * MM),
                             (rc * MM * math.cos(a), rc * MM * math.sin(a), (z0 + z1) / 2 * MM), color,
                             quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)), **kw))
    return out


def rim_segments(n=16):
    """The flexspline's top 10 mm as n convex wedges with a 1 mm lead-in chamfer at the open end."""
    prof = [(FS_R[0], 50.0), (FS_R[1], 50.0), (FS_R[1], Z_TOP), (FS_R[0] + CHAMFER, Z_TOP), (FS_R[0], Z_TOP - CHAMFER)]
    out = []
    half = math.pi / n
    for i in range(n):
        a = 2 * math.pi * i / n
        pts = []
        for b in (a - half * 1.02, a + half * 1.02):
            c, s = math.cos(b), math.sin(b)
            for r, z in prof:
                rr = r / math.cos(half)  # chord at inradius r
                pts.append((rr * c * MM, rr * s * MM, z * MM))
        m = len(prof)
        faces = [tuple(range(m))[::-1], tuple(range(m, 2 * m))] + [(k, (k + 1) % m, m + (k + 1) % m, m + k)
                                                                     for k in range(m)]
        out.append(parts.hull(pts, faces, BRIGHT, friction=(0.15, 0.12)))  # oiled steel on steel, estimate
    return out


STOP_SHAPES = tuple(ring(11.0, 15.0, Z_TOP - MOUTH - 0.6, Z_TOP - MOUTH, BRIGHT, hidden=True,
                         friction=(MESH_MU, MESH_MU)))
STOP = parts.Slider('stop', STOP_SHAPES, axis=(0.0, 0.0, -1.0), travel=PARK * MM, mass=M_STOP, spring=K_STOP,
                    damping=40.0, max_effort=300.0)
OUT_Z = Z_TOP - UNIT_L  # the gearbox's output face, inside the riser's bore
JOINT_SHAPES = [
    parts.box((0.120, 0.120, CRADLE * MM), (0.0, 0.0, CRADLE / 2 * MM), CRADLE_C),  # cradle base plate
    *ring(RISER_R[0], RISER_R[1], CRADLE, RISER_TOP, CRADLE_C, n=20),  # cradle riser; the flange is bolted on top
    *ring(CS_R[1], FLANGE_R, RISER_TOP, Z_TOP, ALU_C, n=20),  # gearbox flange (73), input face on top
    *ring(CS_R[1], BODY_R, OUT_Z, RISER_TOP, ALU_C),  # gearbox body down to the output face (56 pilot)
    *ring(SPIGOT_R[0], SPIGOT_R[1], Z_TOP, Z_TOP + SPIGOT_H, ALU_C),  # 38 h7 input register round the bore
    *ring(CS_R[0], CS_R[1], CS_Z[0], CS_Z[1], DARK),  # circular spline, fixed in the gearbox housing
    *ring(FS_R[0], FS_R[1], FS_Z[0], 50.0, BRIGHT, friction=(0.15, 0.12)),  # flexspline cup below the rim
    *rim_segments(),
    parts.cylinder(FS_R[1] * MM, (INNER - OUT_Z) * MM, (0.0, 0.0, (INNER + OUT_Z) / 2 * MM), BRIGHT),  # cup floor
    parts.cylinder(CS_R[1] * MM, 1.0 * MM, (0.0, 0.0, (OUT_Z + 0.5) * MM), DARK),  # output flange face (23 circle)
    *[parts.cylinder(3.5 * MM, 0.8 * MM, (32.5 * MM * math.cos(a), 32.5 * MM * math.sin(a), (Z_TOP + 0.4) * MM),
                     BLACK, collide=False) for a in (np.arange(8) + 0.5) * math.pi / 4],  # 8 x M4 heads, 65 circle
]
JOINT_A = (0.290, -0.090, TOP)
GEARBOX_PUBLISHED_MASS = 0.51  # kg: HSS-14-50-I-D8 full datasheet, whole unit including generator
GENERATOR_MASS_EST = 0.035  # kg: unmeasured removable generator
CRADLE_MASS_EST = 0.300  # kg: conservative estimate, 248.5 cm3 solid envelope
JOINT_MASS = GEARBOX_PUBLISHED_MASS - GENERATOR_MASS_EST + CRADLE_MASS_EST  # 0.775 kg, generator separate
JOINT = parts.Part('joint', pos=JOINT_A, mass=JOINT_MASS, shapes=JOINT_SHAPES, joints=(STOP,), friction=(0.8, 0.7),
                   link_friction=(('stop', (0.6, 0.5)),), contact_offset=COLLISION_OFFSET)

# ---- the wave generator on its shaft (its frame: origin at the plug's bottom face centre, axis up, mm) ------------
# Drawing: generator A takes the 8 mm keyed input (3 mm key). The 14 hub, 8 H7 bore and set screws are dimensioned in the
# generator detail labelled B and assumed the same for A. B is 12 long and sits 2 deeper (10.5 vs 8.5) with the same outer
# end, so A is about 10 long (inferred). Plug size and height: estimates.
PLUG_R, PLUG_H = 15.2, 8.0  # plug (cam in its thin bearing): estimates
LEAD = 1.2  # plug bottom chamfer (estimate)
HUB_R, HUB_H = 7.0, 2.0  # 14 hub (generator detail), 2 mm below the plug (inferred)
SHAFT_R, SHAFT_Z = 4.0, (PLUG_H, 30.0)  # 8 mm stainless shaft, the motor-shaft stand-in
DISC_R, DISC_Z = 15.0, (30.0, 33.0)  # printed handle: finger stop disc
KNOB_WIDTH, KNOB_Z = 20.0, (33.0, 49.0)  # square printed grip spans the pad height
HOME = INSTALL - HUB_H  # 6.5 mm: plug bottom below the input face when installed
_N = 24
_lead_pts = [((PLUG_R - LEAD) * math.cos(a) * MM, (PLUG_R - LEAD) * math.sin(a) * MM, 0.0)
             for a in np.arange(_N) * 2 * math.pi / _N] + \
            [(PLUG_R * math.cos(a) * MM, PLUG_R * math.sin(a) * MM, LEAD * MM) for a in np.arange(_N) * 2 * math.pi / _N]
_lead_faces = [tuple(range(_N))[::-1], tuple(range(_N, 2 * _N))] + \
              [(k, (k + 1) % _N, _N + (k + 1) % _N, _N + k) for k in range(_N)]


def _zprism(r, z, color):
    return parts.prism(r * MM, (z[1] - z[0]) * MM, _N, (0.0, 0.0, (z[0] + z[1]) / 2 * MM), color, quat=QZ)


WG_SHAPES = [
    _zprism(HUB_R, (-HUB_H, 0.0), STEEL),
    parts.hull(_lead_pts, _lead_faces, STEEL, friction=(MESH_MU, MESH_MU)),  # passive mesh-brake face
    parts.prism(PLUG_R * MM, (PLUG_H - LEAD) * MM, _N, (0.0, 0.0, (PLUG_H + LEAD) / 2 * MM), STEEL, quat=QZ),
    _zprism(SHAFT_R, SHAFT_Z, BRIGHT),
    _zprism(DISC_R, DISC_Z, BLACK),
    parts.box((KNOB_WIDTH * MM, KNOB_WIDTH * MM, (KNOB_Z[1] - KNOB_Z[0]) * MM),
              (0.0, 0.0, (KNOB_Z[0] + KNOB_Z[1]) / 2 * MM), BLACK),
    parts.box((2.0 * MM, 6.0 * MM, 0.3 * MM), (0.0, 11.0 * MM, (PLUG_H + 0.05) * MM), (0.85, 0.15, 0.12),
              collide=False),  # red index mark on the plug
]
WG_MASS = 0.064  # kg: assembly estimate, not weighed; generator .035 + 51 mm steel shaft .020 + printed handle .009
TRAY_FLOOR = 4.0
TRAY_A = (0.195, -0.225, TOP)
WG = parts.Part('wg', pos=(TRAY_A[0], TRAY_A[1], TOP + (TRAY_FLOOR + HUB_H) * MM), mass=WG_MASS, shapes=WG_SHAPES,
                report=('base',), contact_offset=COLLISION_OFFSET)
TRAY = parts.Part('tray', pos=TRAY_A, mass=0.40, friction=(0.6, 0.5), shapes=[
    parts.box((0.140, 0.100, TRAY_FLOOR * MM), (0.0, 0.0, TRAY_FLOOR / 2 * MM), ALU),
    parts.box((0.140, 0.004, 0.015), (0.0, 0.048, 0.0075), ALU),
    parts.box((0.140, 0.004, 0.015), (0.0, -0.048, 0.0075), ALU),
    parts.box((0.004, 0.092, 0.015), (0.068, 0.0, 0.0075), ALU),
    parts.box((0.004, 0.092, 0.015), (-0.068, 0.0, 0.0075), ALU),
])
# ---- the rest of the bench (fixed): the joint's stepper motor on its adapter plate, the joint's cover ------------
BENCH = parts.Part('bench', pos=(0.540, 0.020, TOP), shapes=[
    parts.box((0.070, 0.070, 0.008), (0.0, 0.0, 0.004), CRADLE_C),  # printed motor adapter plate
    parts.box((0.057, 0.057, 0.056), (0.0, 0.0, 0.036), BLACK),  # NEMA 23 stepper, 57 x 57 x 56 mm
    parts.cylinder(0.004, 0.020, (0.0, 0.0, 0.074), BRIGHT),  # 8 mm shaft
    parts.cylinder(0.040, 0.020, (0.0, -0.110, 0.010), CRADLE_C),  # printed joint cover
])

OBJECTS = [JOINT, WG, TRAY, BENCH]
# The housing (an articulated part: the mesh stop is a joint) is never moved between lots; lot B moves the tray.
TRAY_B = (0.385, -0.215, TOP)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot('A', place={'tray': TRAY_A, 'wg': WG.pos}),  # zero move: settles like lot B
    lots.Lot('B', place={'tray': TRAY_B, 'wg': (TRAY_B[0], TRAY_B[1], TOP + (TRAY_FLOOR + HUB_H) * MM)},
             yaw={'wg': math.pi / 2}),
)
DEMO_TOUCHES = {'wg', 'tray'}

# ---- geometry helpers --------------------------------------------------------------------------------------------


def rot(st):
    return parts._quat_to_matrix(st.quat_xyzw)


def wg_dist(p):
    """Distance (m) from points in the generator frame to the generator (hub, plug, shaft, handle and knob)."""
    p = np.asarray(p, float)
    if not len(p):
        return np.inf
    r = np.hypot(p[:, 0], p[:, 1])
    out = np.full(len(p), np.inf)
    for z0, z1, rad in ((-HUB_H, 0.0, HUB_R), (0.0, PLUG_H, PLUG_R), SHAFT_Z + (SHAFT_R,), DISC_Z + (DISC_R,)):
        dz = np.maximum(np.maximum(z0 * MM - p[:, 2], p[:, 2] - z1 * MM), 0.0)
        out = np.minimum(out, np.hypot(np.maximum(r - rad * MM, 0.0), dz))
    # Exact signed-outside distance to the square grip, rather than a cylindrical proxy.
    half = np.array([KNOB_WIDTH / 2, KNOB_WIDTH / 2, (KNOB_Z[1] - KNOB_Z[0]) / 2]) * MM
    q = np.abs(p - np.array([0.0, 0.0, (KNOB_Z[0] + KNOB_Z[1]) / 2 * MM])) - half
    out = np.minimum(out, np.linalg.norm(np.maximum(q, 0.0), axis=1))
    return float(out.min())


def fresh():
    return dict(known=False, fail='', ded=False, front=MOUTH, phi=None, loads=[], load=0.0, peak=0.0, e=0.0,
                tilt=0.0, d=-99.0, prev_t=None, hold=0.0, anchor=None, released=False, turned=0.0, joint0=None,
                spin=0.0, why='', slow=0.0, events=0, mesh_t=None, contact=False,
                total_load=0.0, friction_load=0.0, normal_load=0.0, friction_torque=0.0, contact_count=0, separation=None, contact_samples=0, drive_target=0.0)


REC = fresh()


def geometry(objects):
    r = REC
    js, ws = objects['joint'], objects['wg']
    if not all(np.isfinite(np.asarray(v)).all() for st in (js, ws)
               for v in (st.pos, st.quat_xyzw, st.lin_vel, st.ang_vel)):
        raise ValueError('nonfinite object state')
    Rj, pj, Rw, pw = rot(js), np.asarray(js.pos), rot(ws), np.asarray(ws.pos)
    if r['joint0'] is None:
        r['joint0'] = (pj.copy(), Rj.copy())
    twist = (Rj @ r['joint0'][1].T)
    if np.linalg.norm(pj - r['joint0'][0]) > 0.005 or Rj[2, 2] < math.cos(math.radians(5)) \
            or abs(math.atan2(twist[1, 0], twist[0, 0])) > math.radians(5.0):
        r['fail'] = r['fail'] or 'joint housing knocked out of its cradle position'
    to_j = Rj.T @ Rw
    axis = to_j[:, 2]
    base = Rj.T @ (pw - pj)  # plug bottom centre, housing frame (m)
    r['tilt'] = float(math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))))
    r['e'] = float(np.hypot(base[0], base[1]) / MM)
    r['d'] = float(Z_TOP - base[2] / MM)  # plug bottom depth below the input face (mm)
    return js, ws, Rj, pj, Rw, pw


def force_window(history, value, dt):
    """A complete 0.25 s rolling average; missing initial history is zero, never a shorter window."""
    history.append((float(dt), max(0.0, float(value))))
    excess = sum(d for d, _ in history) - CONTACT_WINDOW
    while excess > EPS and history:
        d, f = history[0]
        if d <= excess + EPS:
            history.pop(0)
            excess -= d
        else:
            history[0] = (d - excess, f)
            break
    return sum(d * f for d, f in history) / CONTACT_WINDOW


def mesh_observe(objects, t, q_stop, contact):
    """Consume one completed physics step, with independently measured contact pairs/normal impulses.

    ``contact`` has mesh axial force only from touching wg/stop points, plus axial normal/friction component magnitudes on all wg/gear contact paths.
    Stop displacement is diagnostic geometry only. It never supplies a force measurement.
    """
    r = REC
    if r['mesh_t'] is not None and t <= r['mesh_t'] + EPS:
        return
    dt = 0.05 if r['mesh_t'] is None else t - r['mesh_t']
    r['mesh_t'] = float(t)
    if contact is None or q_stop is None or not all(k in objects for k in ('joint', 'wg')):
        r['known'] = False
        r['fail'] = r['fail'] or 'contact or object state unavailable'
        return
    js, ws, Rj, pj, Rw, pw = geometry(objects)
    phi = math.atan2((Rj.T @ Rw)[1, 0], (Rj.T @ Rw)[0, 0])
    turn = 0.0 if r['phi'] is None else abs((phi - r['phi'] + math.pi) % (2 * math.pi) - math.pi)
    r['phi'] = phi
    r['spin'] = float((np.asarray(ws.ang_vel) - np.asarray(js.ang_vel)) @ Rw[:, 2])
    # A detailed solver contact is necessary, even if the stop was already deflected by an impact or another body.
    r['contact_count'] = int(contact['count'])
    r['separation'] = contact['separation']
    r['contact'] = bool(contact['count'] > 0 and contact['separation'] is not None
                        and contact['separation'] <= CONTACT_SEPARATION + EPS)
    r['load'] = max(0.0, float(contact['mesh_axial'])) if r['contact'] else 0.0
    r['normal_load'] = max(0.0, float(contact['gear_normal']))
    r['friction_load'] = max(0.0, float(contact.get('gear_friction_axial', 0.0)))
    r['total_load'] = r['normal_load'] + r['friction_load']
    r['contact_samples'] += 1
    # Whole-trial limit includes wg against either gear body, at every offset, tilt, and insertion depth.
    mean = force_window(r['loads'], r['total_load'], dt)
    r['peak'] = max(r['peak'], mean)
    if mean > LOAD_MAX + EPS:
        r['fail'] = r['fail'] or 'benchmark generator/gear contact-force limit exceeded (30 N mean)'
    engaged = (r['contact'] and r['load'] > CONTACT_LOAD_MIN and r['tilt'] < 30.0
               and r['e'] <= E_MAX + EPS and r['d'] > MOUTH - 0.5)
    if r['d'] < 0.5 - EPS:
        r['front'], r['ded'] = MOUTH, False
        r['slow'] = 0.0
        engaged = False
    on_teeth = engaged and r['front'] < HOME - EPS and not r['ded']
    # These are benchmark proxy transitions, not measured HSS damage forces. They remain pending maker validation.
    slow_press = on_teeth and r['load'] >= F_DED - EPS and abs(r['spin']) < W_TURN
    r['slow'] = r['slow'] + dt if slow_press else 0.0
    if on_teeth:
        if r['slow'] >= 0.25 - EPS or (r['tilt'] > TILT_MAX + EPS and r['load'] >= F_TILT - EPS):
            r['ded'], r['front'], r['slow'] = True, HOME, 0.0
            r['events'] += 1
        elif turn > 0 and r['load'] >= LOAD_MIN - EPS and r['tilt'] <= TILT_MAX + EPS:
            step = FEED * turn / (2 * math.pi)
            r['front'] = float(min(HOME, r['front'] + step, max(r['front'], r['d'] + 0.2)))
            r['turned'] += turn


def observe(objects, hands, t, q_stop, contact=None, *, physics_sample=True):
    """Hand/release/hold observations at control rate; unit controls may also supply a measured contact sample."""
    r = REC
    if r['prev_t'] is not None and t <= r['prev_t'] + EPS:
        return
    dt = 0.05 if r['prev_t'] is None else min(0.1, max(0.0, t - r['prev_t']))
    r['prev_t'] = float(t)
    if physics_sample:
        mesh_observe(objects, t, q_stop, contact)
    if hands is None or q_stop is None or not all(k in objects for k in ('joint', 'wg')):
        r['known'] = False
        r['fail'] = r['fail'] or 'hand or object state unavailable'
        r['hold'] = 0.0
        return
    r['known'] = r['contact_samples'] > 0
    js, ws, Rj, pj, Rw, pw = geometry(objects)
    # Hands: never on the gear teeth below the input face; released and clear for the final hold.
    hand_wg = np.inf
    for pts in hands.values():
        if not len(pts):
            continue
        pts = np.asarray(pts)
        hj = (pts - pj) @ Rj
        rr = np.hypot(hj[:, 0], hj[:, 1])
        if np.any((rr < FS_R[1] * MM) & (hj[:, 2] < (Z_TOP - 0.5) * MM) & (hj[:, 2] > FS_Z[0] * MM)):
            r['fail'] = r['fail'] or 'gripper on the gear teeth inside the flexspline'
        hw = (pts - pw) @ Rw
        near = hw[np.linalg.norm(hw, axis=1) < 0.2]
        hand_wg = min(hand_wg, wg_dist(near))
    r['released'] = bool(hand_wg >= 0.020 - EPS)
    # Dropped: falling faster than 0.4 m/s (about an 8 mm free fall) with no gripper on it, anywhere but inside the gear
    # (the eccentric mesh lets a pressed plug drop home inside the rim; that is scored by its own rule).
    out_of_gear = r['e'] > E_MAX + EPS or r['d'] < MOUTH - 0.5
    if float(np.asarray(ws.lin_vel)[2]) < -0.40 and hand_wg >= 0.005 - EPS and out_of_gear:
        r['fail'] = r['fail'] or 'wave generator dropped'
    home = (not r['ded']) and r['front'] >= HOME - EPS and abs(r['d'] - HOME) <= INSTALL_TOL + EPS and r['e'] <= E_MAX + EPS \
        and r['tilt'] <= TILT_MAX + EPS
    marks = np.array([pw, pw + Rw @ np.array([0.0, 0.0, KNOB_Z[1] * MM]), pw + Rw @ np.array([PLUG_R * MM, 0.0, 0.004]), pj])
    still = objects['wg'].resting(0.02, 0.5) and objects['joint'].resting(0.02, 0.5)
    if not (home and not r['fail'] and r['released'] and still):
        r['anchor'], r['hold'] = None, 0.0
    elif r['anchor'] is None or np.linalg.norm(marks - r['anchor'], axis=1).max() > 0.001 + EPS:
        r['anchor'], r['hold'] = marks.copy(), 0.0
    else:
        r['hold'] += dt
    r['home'] = bool(home)


def score(objects):
    r = REC
    if not r['known'] or r['fail'] or r['ded'] or r['d'] < MOUTH - 0.5 or r['e'] > E_MAX + EPS or r['tilt'] > 30.0:
        return 0.0
    if r.get('home'):
        return 1.0 if r['hold'] >= 1.0 - EPS else 0.75
    depth = min(r['front'], r['d'] + 0.5)  # where the gear holds the plug, if the plug is there
    if depth >= MOUTH + (HOME - MOUTH) / 2 - EPS:
        return 0.5
    if depth >= MOUTH + 1.0 - EPS:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# ---- hands: face-sampled collision hulls ------------------------------------------------------------------------
_FACE_CACHE = {}


def face_points(name):
    if name in _FACE_CACHE:
        return _FACE_CACHE[name]
    from yam_arena import analysis
    pts = []
    for h in analysis.link_hulls().get(name, []):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes, 7), axis=0):
            n, d = plane[:3], plane[3]
            on = h.vertices[np.abs(h.vertices @ n - d) < 1e-6]
            if len(on) < 3:
                continue
            c = on.mean(axis=0)
            u = on[0] - c
            u /= np.linalg.norm(u)
            v = np.cross(n, u)
            ring_ = on[np.argsort(np.arctan2((on - c) @ v, (on - c) @ u))]
            for a, b in zip(ring_, np.roll(ring_, -1, axis=0)):
                count = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / .003)))
                ii, jj = np.meshgrid(np.arange(count + 1), np.arange(count + 1))
                valid = ii + jj <= count
                pts.append(c + (a - c) * ii[valid, None] / count + (b - c) * jj[valid, None] / count)
    if not pts:
        raise ValueError('missing hand hull ' + name)
    p = np.concatenate(pts)
    _, ix = np.unique(np.round(p / .003).astype(int), axis=0, return_index=True)
    _FACE_CACHE[name] = p[np.sort(ix)]
    return _FACE_CACHE[name]


def hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, tcp = {}, {}
    for arm in ('left', 'right'):
        asset = base.scene[arm + '_robot']
        poses = parts._np(asset.data.body_link_pose_w)[0]
        pts = []
        for n in ('gripper', 'tip_left', 'tip_right'):
            i = list(asset.body_names).index(n)
            pose = poses[i]
            R = parts._quat_to_matrix(pose[3:])
            pts.append(face_points(n) @ R.T + pose[:3] - origin)
            if n == 'gripper':
                tcp[arm] = (R @ np.array([0.0, 0.0, 0.134]) + pose[:3] - origin).tolist()
        out[arm] = np.concatenate(pts)
    if not all(np.isfinite(p).all() for p in out.values()):
        raise ValueError('nonfinite hands')
    return out, tcp


def drag(rec):
    """Diagnostic lower bound on sliding brake moment (N m); never applies a force.

    Contact geometry guarantees radius >= MESH_MIN_R. Actual contact impulses are solved by PhysX, with equal
    reaction on the stop; static friction can take any value up to its capacity.
    """
    if (not rec['known'] or not rec['contact'] or rec['tilt'] >= 30.0 or rec['e'] > E_MAX + EPS
            or rec['d'] < rec['front'] - 0.5 or rec['d'] <= MOUTH - 0.5
            or rec['load'] < CONTACT_LOAD_MIN):
        return 0.0
    return -MESH_MU * max(0.0, rec['load']) * MESH_MIN_R * MM * max(-1.0, min(1.0, rec['spin'] / W_SLIP))


class ContactReader:
    """Task-owned PhysX views, sampled after every completed physics step.

    Uses the installed PhysX post-step event directly. Tensor transforms/velocities avoid Isaac Lab's
    control-rate cached asset data. No simulator or harness source is changed.
    """
    def __init__(self, sim):
        import omni.physics.tensors as tensors
        self.base = sim.env.unwrapped
        self.dt = float(self.base.physics_dt)
        self.origin = parts._np(self.base.scene.env_origins[0])
        self.start = int(self.base.sim.get_physics_step_count())
        self.last = self.start
        self.error = None
        self.view = tensors.create_simulation_view('torch')
        self.view.set_subspace_roots('/')
        wp, jp = parts._PATHS['wg'], parts._PATHS['joint']
        self.wg = self.view.create_rigid_body_view(wp)
        self.joint = self.view.create_rigid_body_view(jp + '/base')
        self.stop = self.view.create_rigid_body_view(jp + '/stop')
        self.joint_com = parts._np(self.joint.get_coms())[0, :3]
        self.contacts = self.view.create_rigid_contact_view(
            wp, filter_patterns=[jp + '/stop', jp + '/base'], max_contact_data_count=CONTACT_CAPACITY)
        if (self.wg.count, self.joint.count, self.stop.count, self.contacts.sensor_count,
                self.contacts.filter_count) != (1, 1, 1, 1, 2):
            raise RuntimeError('contact views do not resolve exactly one generator and two gear links')
        import omni.physx
        self.subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
            self.after_step, pre_step=False, order=0)

    def close(self):
        self.subscription.unsubscribe()

    def after_step(self, dt):
        try:
            self.sample()
        except Exception as exc:
            REC['known'] = False
            REC['fail'] = 'physics contact reader error: ' + repr(exc)
            self.error = exc  # re-raised by the control callback; never silently pass a missing measurement

    @staticmethod
    def snapshot(*buffers):
        """Batch GPU-to-host copies once per pose/contact group; preserves each tensor's original shape."""
        import torch
        values = [a.torch if hasattr(a, 'torch') else a for a in buffers]
        sizes = [a.numel() for a in values]
        raw = parts._np(torch.cat([a.reshape(-1) for a in values]))
        return [a.reshape(tuple(v.shape)) for a, v in zip(np.split(raw, np.cumsum(sizes)[:-1]), values)]

    def sample(self):
        index = int(self.base.sim.get_physics_step_count())
        if index != self.last + 1:
            raise RuntimeError('missed or duplicate physics contact sample: ' + str((self.last, index)))
        self.last = index
        pose_w, pose_j, pose_stop, vel_w, vel_j = self.snapshot(
            self.wg.get_transforms(), self.joint.get_transforms(), self.stop.get_transforms(),
            self.wg.get_velocities(), self.joint.get_velocities())
        w, j, st, wv, jv = pose_w[0], pose_j[0], pose_stop[0], vel_w[0], vel_j[0]
        Rj = parts._quat_to_matrix(j[3:])
        q = float(STOP.origin[2] - (Rj.T @ (st[:3] - j[:3]))[2])
        from types import SimpleNamespace
        objects = {
            'wg': SimpleNamespace(pos=w[:3] - self.origin, quat_xyzw=w[3:], lin_vel=wv[:3], ang_vel=wv[3:]),
            'joint': SimpleNamespace(pos=j[:3] - self.origin, quat_xyzw=j[3:], lin_vel=jv[:3], ang_vel=jv[3:]),
        }
        matrix_buffer = self.contacts.get_contact_force_matrix(dt=self.dt)
        force, points, normal, separation, counts, starts = self.contacts.get_contact_data(dt=self.dt)
        pair_matrix, force, normal, separation, counts, starts = self.snapshot(
            matrix_buffer, force, normal, separation, counts, starts)
        pair_matrix = pair_matrix.reshape(2, 3)
        force, normal, separation = force.reshape(-1), normal.reshape(-1, 3), separation.reshape(-1)
        counts, starts = counts.astype(int).reshape(-1), starts.astype(int).reshape(-1)
        if not np.isfinite(pair_matrix).all():
            raise ValueError('nonfinite contact matrix')
        if any(n < 0 or (n > 0 and (start < 0 or start + n > len(force))) for n, start in zip(counts, starts)):
            raise ValueError('invalid detailed contact buffer range')
        if counts.sum() >= CONTACT_CAPACITY:
            raise RuntimeError('contact buffer capacity reached')
        total = 0.0
        axial = 0.0
        touching = 0
        min_sep = None
        for k, (n, start) in enumerate(zip(counts, starts)):
            if not n:
                continue
            if not all(np.isfinite(a).all() for a in (force[start:start + n], normal[start:start + n],
                                                       separation[start:start + n])):
                raise ValueError('nonfinite active contact measurement')
            f = np.abs(force[start:start + n])
            total += float(np.sum(f * np.abs(normal[start:start + n] @ Rj[:, 2])))
            if k == 0 and n:
                sep = separation[start:start + n]
                ok = (sep <= CONTACT_SEPARATION) & (f > EPS)
                touching = int(ok.sum())
                if touching:
                    min_sep = float(sep[ok].min())
                    axial = float(np.sum(f[ok] * np.abs(normal[start:start + n][ok] @ Rj[:, 2])))
        # Friction has separate buffers in the installed PhysX API. Copy normal buffers first because
        # get_friction_data shares/overwrites its count and start-index buffers with get_contact_data.
        friction, friction_points, friction_counts, friction_starts = self.contacts.get_friction_data(dt=self.dt)
        friction, friction_points, friction_counts, friction_starts = self.snapshot(
            friction, friction_points, friction_counts, friction_starts)
        friction = friction.reshape(-1, 3)
        friction_points = friction_points.reshape(-1, 3)
        friction_counts = friction_counts.astype(int).reshape(-1)
        friction_starts = friction_starts.astype(int).reshape(-1)
        if friction_counts.sum() >= CONTACT_CAPACITY:
            raise RuntimeError('friction contact buffer capacity reached')
        friction_axial = 0.0
        friction_torque = 0.0
        wg_axis = parts._quat_to_matrix(w[3:])[:, 2]
        for n, start in zip(friction_counts, friction_starts):
            if not n:
                continue
            if n < 0 or start < 0 or start + n > len(friction):
                raise ValueError('invalid friction contact buffer range')
            f = friction[start:start + n]
            if not np.isfinite(f).all():
                raise ValueError('nonfinite friction contact measurement')
            friction_axial += float(np.sum(np.abs(f @ Rj[:, 2])))
            if os.environ.get('TRACE_OUT'):
                lever = friction_points[start:start + n] - w[:3]
                if not np.isfinite(lever).all():
                    raise ValueError('nonfinite friction contact point')
                friction_torque += float(np.sum(np.cross(lever, f) @ wg_axis))
        contact = dict(mesh_axial=axial, gear_normal=total, gear_friction_axial=friction_axial,
                       count=touching, separation=min_sep)
        # The first real sample covers exactly one dt, even when dt is not the control period.
        if REC['mesh_t'] is None:
            REC['mesh_t'] = 0.0
        t = (index - self.start) * self.dt
        target_used = REC['drive_target']
        REC['friction_torque'] = friction_torque
        mesh_observe(objects, t, q, contact)
        if os.environ.get('TRACE_OUT'):
            sv = parts._np(self.stop.get_velocities())[0]
            qv = float((jv[:3] + np.cross(jv[3:], st[:3] - j[:3] - Rj @ self.joint_com) - sv[:3]) @ Rj[:, 2])
            qa = 0.0 if not hasattr(self, 'last_qv') else (qv - self.last_qv) / self.dt
            self.last_qv = qv
            row = dict(t=t, dt=self.dt, q=q, depth=REC['d'], front=REC['front'], mesh_axial=axial,
                       gear_normal=total, gear_friction_axial=friction_axial, gear_total_axial=total+friction_axial, friction_torque_Nm=friction_torque,
                       count=touching, separation=min_sep, peak=REC['peak'],
                       stop_v=sv.tolist(), wg_v=wv.tolist(), target=target_used, qv=qv, qa=qa,
                       pair_matrix=pair_matrix.tolist(),
                       static_estimate=K_STOP * (q - target_used) - M_STOP * 9.81,
                       dynamic_balance=K_STOP * (q - target_used) + 40.0 * qv + M_STOP * qa - M_STOP * 9.81)
            with Path(os.environ['TRACE_OUT']).with_name('contact.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')


_CONTACT_READER = None


@parts.on_reset
def _reset(sim):
    global REC, _CONTACT_READER
    if _CONTACT_READER is not None:
        _CONTACT_READER.close()
    REC = fresh()
    sim.set_target('joint', 'stop', 0.0)
    _CONTACT_READER = ContactReader(sim)


@parts.on_step
def _step(sim):
    try:
        objects = sim.states()
        hands, tcp = hand_clouds(sim)
        q = objects['joint'].joints.get('stop') if hasattr(objects['joint'], 'joints') else None
        if _CONTACT_READER.error is not None:
            raise _CONTACT_READER.error
        observe(objects, hands, float(sim.t), None if q is None else float(q), physics_sample=False)
        REC['drive_target'] = (REC['front'] - MOUTH) * MM
        sim.set_target('joint', 'stop', REC['drive_target'])
        REC['drag_est'] = drag(REC)  # diagnostic capacity only; contact solver applies all friction
        if os.environ.get('TRACE_OUT') and round(sim.t * 20) % 2 == 0:
            ws = objects['wg']
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, front=round(REC['front'], 3),
                       d=round(REC['d'], 3), load=round(REC['load'], 3), q=q, e=round(REC['e'], 3),
                       tilt=round(REC['tilt'], 3), ded=REC['ded'], fail=REC['fail'], hold=REC['hold'],
                       score=score(objects), released=REC['released'], turned=round(REC['turned'], 3),
                       wg=np.round(ws.pos, 5).tolist(), wgq=np.round(ws.quat_xyzw, 5).tolist(),
                       joint=np.round(objects['joint'].pos, 5).tolist(), tcp=tcp, phi=REC['phi'],
                       spin=round(REC.get('spin', 0.0), 3), drag_est=round(REC.get('drag_est', 0.0), 4),
                       peak=round(REC['peak'], 3), total_load=round(REC['total_load'], 3),
                       contacts=REC['contact_count'], separation=REC['separation'],
                       physics_samples=REC['contact_samples'], physics_dt=_CONTACT_READER.dt,
                       normal_load=REC['normal_load'], friction_load=REC['friction_load'],
                       friction_torque_Nm=REC['friction_torque'])
            with Path(os.environ['TRACE_OUT']).with_name('wave.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
    except Exception as e:  # fail closed
        REC['known'] = False
        REC['fail'] = REC['fail'] or 'scorer error'
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('wave-errors.txt').open('a') as f:
                f.write(repr(e) + '\n')
        raise


# ---- demo: open loop from the visible layout ---------------------------------------------------------------------
CAL = json.loads((HERE / 'fixture' / 'calibration.json').read_text())
Y_HI, Y_LO = math.radians(60.0), math.radians(-60.0)
GRIP = -0.006  # 12 mm target gap on 20 mm square grip: 4 mm closure allowance per pad
OPEN = -0.020  # fingers 40 mm apart: clear of the square grip and 30 mm disc
TCP_ABOVE_PLUG = 47.0 * MM  # tips 1 mm above the disc; long square grip spans the pad height
PRELOAD = 28.0  # mm command: trace13 stop load8.8-9.8 N with24 mm, total gear~13 N; +4 mm targets >=10 N on stop
DESCENT = 0.50  # mm commanded per 120-degree stroke; below nominal mesh feed, to avoid outrunning engagement
N_STROKES = 14


def demo():
    lot = lots.current()
    jp = np.asarray(lot.pos('joint'))
    wp = np.asarray(lot.pos('wg'))
    arm = 'right'
    other = 'left'
    cal = CAL[arm]
    g = np.asarray(cal['grasp'])
    b = np.asarray(cal['bore'])

    def W(d, p, f, label, yaw=0.0):
        return Waypoint(float(d), tuple(float(v) for v in p), float(f), label, yaw=float(yaw))

    def tcp_at(depth_mm):
        """TCP over the gear axis with the plug bottom ``depth_mm`` below the input face."""
        return np.array([jp[0], jp[1], jp[2] + (Z_TOP - depth_mm) * MM + TCP_ABOVE_PLUG]) + b

    at = wp + np.array([0.0, 0.0, TCP_ABOVE_PLUG]) + g
    above = at + np.array([0.0, 0.0, 0.030])
    travel = jp[2] + (Z_TOP + SPIGOT_H + HUB_H + 12.0) * MM + TCP_ABOVE_PLUG
    seq = [W(2.5, above, OPEN, 'over the wave generator'),
           W(2.0, at, OPEN, 'fingers round the knob'),
           W(1.0, at, GRIP, 'grip the knob'),
           W(1.5, (at[0], at[1], travel), GRIP, 'lift the generator'),
           W(2.5, tcp_at(-12.0), GRIP, 'over the flexspline'),
           W(1.5, tcp_at(-3.0), GRIP, 'lower to the open end'),
           W(1.5, tcp_at(-3.0), GRIP, 'square over the rim'),
           W(1.5, tcp_at(MOUTH - 0.3), GRIP, 'plug into the lead-in'),
           W(1.5, tcp_at(MOUTH + PRELOAD), GRIP, 'press on the teeth')]
    depth = MOUTH + PRELOAD
    yaw = 0.0
    targets = [Y_LO, Y_HI]
    if os.environ.get('DEMO_ROUTE') == 'straight':  # the naive route, for the bypass table: push, never turn
        seq += [W(4.0, tcp_at(HOME + 3 * PRELOAD), GRIP, 'push straight down, no turning'),
                W(4.0, tcp_at(HOME + 3 * PRELOAD), GRIP, 'keep pushing')]
        N = 0
    else:
        N = N_STROKES
    for k in range(N):
        depth = min(HOME + PRELOAD, depth + DESCENT)
        yaw = targets[k % 2]
        seq.append(W(1.8 if k else 1.2, tcp_at(depth), GRIP, f'turn it in, stroke {k + 1}', yaw))
    seq += [W(1.0, tcp_at(HOME + 0.3), GRIP, 'ease the press off at home', yaw),
            W(1.0, tcp_at(HOME + 0.3), OPEN, 'release the knob', yaw),
            W(1.5, tcp_at(-15.0), OPEN, 'lift clear', yaw),
            W(2.0, tcp_at(-15.0) + np.array([-0.080, -0.040, 0.020]), FINGER_OPEN, 'stand clear', 0.0),
            W(2.5, tcp_at(-15.0) + np.array([-0.080, -0.040, 0.020]), FINGER_OPEN, 'hold still', 0.0)]
    total = sum(w.duration for w in seq)
    rest = [W(total, (0.20, 0.30, TOP + 0.16), FINGER_OPEN, 'stay clear', 0.0)]
    return {arm: seq, other: rest}
