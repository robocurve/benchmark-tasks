"""Synthetic checks of the scorer (no simulator): python check_scoring.py [out.json]. Exit code 1 on any failure."""
from __future__ import annotations

import json
import math
import sys

import numpy as np

import layout as L
from scoring import (HOLD_STEPS, HOLE_AXIS_B_M, Scorer, _square, boxes_distance, measure, wrap_deg)

R_S, O_S = L.R_S, L.O_S
CHECKS = []


def check(name, cond, info=None):
    CHECKS.append(dict(name=name, ok=bool(cond), info=info))


def rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


HA = np.array([HOLE_AXIS_B_M[0], HOLE_AXIS_B_M[1], 0.])


def pose(dx_mm=0., dy_mm=0., dz_mm=0., yaw=0., tilt=0.):
    """Bracket pose: home at yaw with its hole-pattern axis on the servo axis, then displaced in the bracket frame and
    tilted about its x axis."""
    p, r = L.bracket_home(yaw)
    p = p - r @ HA
    r = r @ rx(tilt)
    return p + r @ (np.array([dx_mm, dy_mm, dz_mm]) / 1000.), r


def horn(deg):
    return L.horn_rot(deg)


def m_at(**kw):
    h = kw.pop('horn', 0.)
    p, r = pose(**kw)
    return measure(p, r, horn(h), O_S, R_S)


def face_plane_mm(**kw):
    """The earlier measure, for comparison: each top hole's axis taken only where it crosses the horn's face plane."""
    h = kw.pop('horn', 0.)
    p_b, r_b = pose(**kw)
    d = R_S.T @ r_b[:, 2]
    q = []
    for c in _square(HOLE_AXIS_B_M, 0.):
        p = R_S.T @ (p_b + r_b @ np.array([c[0], c[1], 0.]) - O_S)
        q.append((p - p[2] / d[2] * d)[:2])
    hs = _square(np.zeros(2), h)
    return 1e3 * min(max(float(np.linalg.norm(q[i] - hs[(i + k) % 4])) for i in range(4)) for k in range(4))


def main(out=None):
    # geometry of the measure
    m = m_at(yaw=25., horn=25.)
    check('home registered at matching yaw', m.home and m.registered and m.on, m.as_dict())
    m = m_at(yaw=25. + 90., horn=25.)
    check('any of the four 90-degree hole positions registers', m.registered, m.as_dict())
    m = m_at(yaw=0., horn=25.)
    check('home but 25 deg off is not registered', m.home and not m.registered and abs(m.register_deg + 25) < 1e-6,
          m.as_dict())
    m = m_at(yaw=25., horn=25.)
    check('centred, square to the horn: hole offset 0', abs(m.hole_offset_mm) < 1e-6, m.as_dict())
    m = measure(*L.bracket_home(25.), horn(25.), O_S, R_S)
    check('joint anchor (0.20 mm from the hole axis) on the servo axis: home, holes 0.20 mm off, not registered',
          m.home and not m.registered and abs(m.hole_offset_mm - .202) < .002, m.as_dict())
    # the old defect: 4 deg and 0.5 mm each passed the old separate checks (yaw within 4 degrees, home radial within 0.5 mm)
    m = m_at(yaw=3.9, horn=0., dx_mm=.45)
    check('3.9 deg and 0.45 mm off (old rule: registered): home, not registered',
          m.home and not m.registered and abs(m.register_deg) <= 4. and m.hole_offset_mm > .8, m.as_dict())
    m = m_at(yaw=4.0, horn=0.)
    check('4 deg alone (about half a millimetre at the holes): not registered', m.home and not m.registered, m.as_dict())
    m = m_at(yaw=2.5, horn=0.)
    check('2.5 deg alone (0.31 mm at the holes): not registered', m.home and not m.registered, m.as_dict())
    m = m_at(dy_mm=.4)
    check('0.4 mm sideways, square: home, not registered', m.home and not m.registered, m.as_dict())
    m = m_at(yaw=1.2, horn=0.)
    check('1.2 deg alone (0.148 mm): registered', m.registered and .14 < m.hole_offset_mm < .15, m.as_dict())
    m = m_at(dx_mm=.149)
    check('0.149 mm along the slot, square: registered', m.registered, m.as_dict())
    m = m_at(dx_mm=.151)
    check('0.151 mm along the slot, square: not registered', m.home and not m.registered, m.as_dict())
    m = m_at(dx_mm=.08, yaw=.5, horn=0.)
    check('combined 0.08 mm + 0.5 deg (0.13 mm): registered', m.registered and m.hole_offset_mm < .15, m.as_dict())
    m = m_at(dx_mm=.12, yaw=.5, horn=0.)
    check('combined 0.12 mm + 0.5 deg (0.17 mm): not registered', m.home and not m.registered and
          m.hole_offset_mm > .15, m.as_dict())
    m = m_at(dx_mm=.05, dy_mm=-.05, yaw=25.4, horn=25., tilt=.5)
    check('small combined error at the horn 25 deg, 0.5 deg tilt: registered', m.registered, m.as_dict())
    # tilt: the bore axis is checked at both ends of the 3.5 mm bore, not only in the horn's face plane
    kw = dict(dx_mm=-.01, dy_mm=-.145, dz_mm=.24, tilt=1.98)
    f, m = face_plane_mm(**kw), m_at(**kw)
    check('tilt 1.98 deg, 0.148 mm in the face plane (the review example): 0.28 mm at the bore top, not registered',
          m.home and not m.registered and abs(f - .148) < .001 and abs(m.hole_offset_mm - .277) < .002,
          dict(face_plane_mm=f, **m.as_dict()))
    kw = dict(dx_mm=.1, dy_mm=-.05, dz_mm=.24, yaw=.3)
    f, m = face_plane_mm(**kw), m_at(**kw)
    check('no tilt: bore-end offset equals the face-plane offset (0.147 mm), registered',
          m.registered and abs(f - m.hole_offset_mm) < 1e-6 and .14 < f < .15, dict(face_plane_mm=f, **m.as_dict()))
    m = m_at(dx_mm=5.)  # bracket backed off 5 mm toward its web side: horn axis 5 mm from the hole axis
    check('backed off 5 mm: on, not home', m.on and not m.home and abs(m.px_mm - HA[0] * 1e3 + 5) < 1e-6, m.as_dict())
    m = m_at(dx_mm=8.8)
    check('backed off 8.8 mm (3.2 mm in the slot): on', m.on)
    m = m_at(dx_mm=9.2)
    check('backed off 9.2 mm (2.8 mm in the slot): not on', not m.on)
    m = m_at(dx_mm=.4)
    check('0.4 mm from home: home', m.home)
    m = m_at(dx_mm=.6)
    check('0.6 mm from home: not home', not m.home and m.on)
    # seated, leaning 0.8 deg back on the horns about the lower cheek (the modelled face play allows this): its top
    # cheek is 0.51 mm off the axis, the middle 0.25 mm: home
    th = math.radians(.8)
    ry = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
    p0, r0 = L.bracket_home(0.)
    low = np.array([0., 0., -.0364])
    r1 = r0 @ ry
    p1 = p0 + r0 @ low - r1 @ low
    m = measure(p1, r1, horn(0.), O_S, R_S)
    check('seated, leaning 0.8 deg about the lower cheek: home', m.home and abs(abs(m.px_mm) - .508) < .01 and
          m.radial_mid_mm < .3, m.as_dict())
    m = m_at(dz_mm=.8, dx_mm=3.)
    check('0.8 mm high, 3 mm out: on', m.on)
    m = m_at(dz_mm=1.5, dx_mm=3.)
    check('1.5 mm high: not on (cheeks not straddling)', not m.on)
    m = m_at(dz_mm=-36.4)  # the bottom cheek level with the top horn: wrong height by a whole cheek gap
    check('one cheek gap low: nothing', not m.on and not m.home)
    m = m_at(tilt=2.5, dx_mm=3.)
    check('2.5 deg tilt: on but not home', m.on and not m.home)
    m = m_at(tilt=180.)
    check('upside down: nothing', not m.on and not m.home)
    m = m_at(dy_mm=1.5, dx_mm=3.)
    check('1.5 mm off the slot line: not on', not m.on)
    p, r = L.bracket_start_rot(), None
    m = measure(L.BRACKET_START, L.bracket_start_rot(), horn(L.HORN_TOP_START_DEG), O_S, R_S)
    check('start pose scores nothing', not m.on and not m.home, m.as_dict())
    check('wrap', abs(wrap_deg(44., 90.) - 44.) < 1e-9 and abs(wrap_deg(46., 90.) + 44.) < 1e-9)

    # ladder and hold
    s = Scorer()
    home_reg = m_at(yaw=25., horn=25.)
    lv = [s.step(home_reg, .05, 0., 0.) for _ in range(HOLD_STEPS - 1)]
    check('registered, released, still: 0.75 until held 1 s', all(v == .75 for v in lv), lv[-3:])
    check('then 1.0', s.step(home_reg, .05, 0., 0.) == 1.0)
    check('hand back on it: 0.75', s.step(home_reg, .01, 0., 0.) == .75)
    s = Scorer()
    check('moving bracket never reaches 1.0', max(s.step(home_reg, .05, .01, 0.) for _ in range(40)) == .75)
    s = Scorer()
    check('held (gripper on it) never reaches 1.0', max(s.step(home_reg, .005, 0., 0.) for _ in range(40)) == .75)
    s = Scorer()
    check('home not registered: 0.5', s.step(m_at(yaw=0., horn=25.), .05, 0., 0.) == .5)
    check('on: 0.25', Scorer().step(m_at(dx_mm=5.), .05, 0., 0.) == .25)
    check('nothing: 0', Scorer().step(m_at(dx_mm=30.), .05, 0., 0.) == 0.)
    s = Scorer()
    for _ in range(HOLD_STEPS + 2):
        s.step(home_reg, .05, 0., 0.)
    check('pulled off after full score: final state counts (0)', s.step(m_at(dx_mm=30.), .05, 0., 0.) == 0.)

    # release distance helper
    p, r = L.bracket_home(0.)
    boxes = [(np.array([-.01, -.01, -.01]), np.array([.01, .01, .01]))]
    check('box distance outside', abs(boxes_distance(p + r @ np.array([.03, 0, 0]), boxes, p, r) - .02) < 1e-9)
    check('box distance inside', boxes_distance(p, boxes, p, r) == 0.)
    ok = all(c['ok'] for c in CHECKS)
    res = dict(passed=ok, n=len(CHECKS), failed=[c for c in CHECKS if not c['ok']], checks=CHECKS)
    if out:
        with open(out, 'w') as f:
            json.dump(res, f, indent=1, default=float)
    print(json.dumps(dict(passed=ok, n=len(CHECKS), failed=[c['name'] for c in CHECKS if not c['ok']])))
    return ok


if __name__ == '__main__':
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else None) else 1)
