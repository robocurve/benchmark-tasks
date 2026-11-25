"""Near misses and unintended strategies for the radiator-cover clip scorer. The scorer is fail-closed: with no
observed trial history (unknown hands or objects) every state scores 0, so CASES pins end-state shortcuts that a
policy could leave without any recorded trial; run() exercises the whole-trial rules and the score ladder with a
recorded history (python near_misses.py)."""
import numpy as np

CASES = [
    ('start, nothing done', {}, 0., False),
    ('cover slid 5 mm off its place', {'cover': ((.360, 0., .7900), None, None)}, 0., False),
    ('lifters moved onto the tray', {'lifter_left': ((.095, .05, .80), None, None)}, 0., False),
]


def run():
    import task as t
    out = []

    def check(label, cond):
        out.append((label, bool(cond)))
        print(('ok   ' if cond else 'FAIL ') + label)

    base = {p.name: p.state() for p in t.OBJECTS}
    tray = base['tray']
    far = np.array([[.20, .60, t.TOP + .30], [.20, -.60, t.TOP + .30]])
    trz = t.Z_MAT + t.TRAY_W

    def clip_in(n, dy, pin=t.PIN_TRAVEL, z=0.):
        return t.SPECS[n].state(pos=(float(tray.pos[0]), float(tray.pos[1]) + dy, trz + t.SHANK_L + z),
                                joints={'pin': pin})

    in3 = {'clip_a': clip_in('clip_a', .06), 'clip_b': clip_in('clip_b', 0.), 'clip_c': clip_in('clip_c', -.06)}
    cov_off = t.COVER.state(pos=(.205, 0., t.Z_MAT + .009))
    cov_lift = t.COVER.state(pos=(t.COVER.pos[0], 0., t.Z_S + .040))

    def ladder(objs, hands=far, unlocked=True, clear=True, hold=1., cover_free=True, **rec):
        t.REC = t.fresh()
        t.REC.update(known=True, clear=clear, hold=hold, cover_free=cover_free)
        t.REC['unlocked'] = {n: unlocked for n in t.CLIP_NAMES}
        t.REC.update(rec)
        st = dict(base, **objs)
        t.REC['n_in'] = t.released_clips(st, hands)
        return t.score(st)

    check('three clips in the tray, cover on the fender cover, hands clear, 1 s still: 1.0',
          ladder(dict(in3, cover=cov_off)) == 1.)
    check('same, hold 0.9 s: 0.8', ladder(dict(in3, cover=cov_off), hold=.9) == .8)
    check('same, hands not 30 mm clear: 0.8', ladder(dict(in3, cover=cov_off), clear=False) == .8)
    check('three clips in, cover still on the support: 0.6', ladder(dict(in3)) == .6)
    check('three clips in, cover on the fender cover but a gripper still on it: 0.6',
          ladder(dict(in3, cover=cov_off), cover_free=False, clear=False) == .6)
    lying = dict(in3); lying['clip_b'] = t.SPECS['clip_b'].state(pos=(float(tray.pos[0]), float(tray.pos[1]), trz + .010),
                                                                 quat_xyzw=(.7071, 0., 0., .7071), joints={'pin': t.PIN_TRAVEL})
    check('a clip lying on its side on the tray floor counts: 1.0', ladder(dict(lying, cover=cov_off)) == 1.)
    check('three clips in, cover held up off the support: 0.6', ladder(dict(in3, cover=cov_lift)) == .6)
    two = dict(in3); two['clip_c'] = t.SPECS['clip_c'].state(joints={'pin': t.PIN_TRAVEL})
    check('two clips in, one still fitted (pin up): 0.4', ladder(two) == .4)
    one_out = dict(in3); one_out['clip_b'] = t.SPECS['clip_b'].state(pos=(.20, .20, t.Z_MAT + t.SHANK_L))
    one_out['clip_c'] = t.SPECS['clip_c'].state()
    check('one clip in, one dropped on the fender cover, one fitted: 0.2', ladder(one_out) == .2)
    hover = dict(in3); hover['clip_a'] = clip_in('clip_a', .06, z=.030)
    check('a clip held 30 mm above the tray floor (still on the fork) does not count: 0.4', ladder(hover) == .4)
    rim = dict(in3); rim['clip_a'] = t.SPECS['clip_a'].state(pos=(float(tray.pos[0]), float(tray.pos[1]) + .125,
                                                                   trz + t.SHANK_L))
    check('a clip outside the tray walls does not count: 0.4', ladder(rim) == .4)
    # a gripper finger resting on a clip in the tray: not released
    touch = np.array([[float(tray.pos[0]), float(tray.pos[1]) + .06, trz + t.SHANK_L + t.HEAD_H + .002]])
    check('a gripper 2 mm from a clip in the tray: that clip does not count (0.4)', ladder(dict(in3), hands=touch) == .4)
    # a lifter's fork still round a clip's neck in the tray: not released
    lif = t.LIFTER_L.state(pos=(float(tray.pos[0]), float(tray.pos[1]) + .06 + t.TIP_DX - .007,
                                trz + t.SHANK_L + t.HEAD_H + .0009 + .006 - t.TIP_DZ - .004),
                           quat_xyzw=t._qz(-np.pi / 2))
    check('a lifter fork still under a clip in the tray: that clip does not count (0.4)',
          ladder(dict(in3, lifter_left=lif)) == .4)
    for label, upd in (('a clip broken (pulled with its pin down)', dict(bad=True)),
                       ('unknown state', dict(known=False))):
        t.REC = t.fresh(); t.REC.update(known=True, clear=True, hold=1.); t.REC.update(upd)
        st = dict(base, cover=cov_off, **in3)
        t.REC['n_in'] = 3
        check(label + ' scores 0', t.score(st) == 0.)
    # Whole-trial clip rules on states.
    t.REC = t.fresh()
    pulled = dict(base); pulled['clip_b'] = t.SPECS['clip_b'].state(pos=(t.CLIP_X, t.CLIP_Y['clip_b'], t.Z_C + .0025))
    check('clip body up 2.5 mm with its pin down: broken', 'legs snapped' in t.clip_check(pulled))
    t.REC = t.fresh()
    pulled2 = dict(base); pulled2['clip_b'] = t.SPECS['clip_b'].state(pos=(t.CLIP_X, t.CLIP_Y['clip_b'], t.Z_C + .0015))
    check('clip body up 1.5 mm with its pin down: not yet broken', t.clip_check(pulled2) == '')
    t.REC = t.fresh()
    pin_first = dict(base); pin_first['clip_b'] = t.SPECS['clip_b'].state(joints={'pin': .0045})
    check('pin raised 4.5 mm: unlocked, no fault', t.clip_check(pin_first) == '' and t.REC['unlocked']['clip_b'])
    pin_first['clip_b'] = t.SPECS['clip_b'].state(pos=(t.CLIP_X, t.CLIP_Y['clip_b'], t.Z_C + .015), joints={'pin': .006})
    check('then the clip drawn 15 mm out: no fault', t.clip_check(pin_first) == '')
    t.REC = t.fresh()
    pin_part = dict(base); pin_part['clip_a'] = t.SPECS['clip_a'].state(joints={'pin': .0035})
    t.clip_check(pin_part)
    pin_part['clip_a'] = t.SPECS['clip_a'].state(pos=(t.CLIP_X, t.CLIP_Y['clip_a'], t.Z_C + .004), joints={'pin': .0035})
    check('pin only 3.5 mm up, then the body drawn 4 mm: broken', 'legs snapped' in t.clip_check(pin_part))
    t.REC = t.fresh()
    pry = dict(base, cover=t.COVER.state(pos=(t.COVER.pos[0], 0., t.Z_S + .006)))
    check('cover pried up 6 mm with every clip fitted: hole torn', 'hole torn' in t.clip_check(pry))
    t.REC = t.fresh()
    check('start: no fault', t.clip_check(base) == '')
    # the cover-off test
    check('cover lying on the fender cover in front of the support: off', t.cover_off(cov_off))
    check('cover lifted above the support: not off', not t.cover_off(cov_lift))
    cov_part = t.COVER.state(pos=(.270, 0., t.Z_MAT + .009))
    check('cover only partly in front of the support: not off', not t.cover_off(cov_part))
    bad = [l for l, c in out if not c]
    print('%d of %d near-miss checks passed' % (len(out) - len(bad), len(out)))
    return not bad


if __name__ == '__main__':
    import sys
    sys.exit(0 if run() else 1)
