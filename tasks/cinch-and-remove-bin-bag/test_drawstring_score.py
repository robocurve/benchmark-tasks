"""Synthetic counterexamples for the tiered drawstring scorer; not physics evidence."""
import unittest
import numpy as np
import trimesh
from drawstring_score import evaluate
from realistic_meshes_v32 import dirty_mesh
from removal_assets import block_positions

PARAMS = {'bin_top_z': .902, 'bin_center_xy': [.22, 0.], 'bin_outer_radius_m': .0855, 'sample_interval_steps': 10}


def gathered_mesh():
    """Authored folded liner and an unfolded, gathered version of the same mesh."""
    import realistic_meshes_v32 as m
    vertices, faces, mouth = dirty_mesh()
    rings = np.array([(0., 0.)] + m.profile())
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(rings, axis=0), axis=1))]
    closed = vertices.copy()
    n = m.N_AROUND
    angle = np.arange(n) * 2 * np.pi / n
    for k in range(1, len(rings)):
        if s[k] <= m.BODY_R:
            radius, z = s[k], 0.
        else:
            z = s[k] - m.BODY_R
            radius = m.BODY_R if z < .14 else max(.012, m.BODY_R - (z - .14) * .6)
        closed[1 + (k - 1) * n:1 + k * n] = np.c_[radius * np.cos(angle), radius * np.sin(angle), np.full(n, z)]
    return vertices, closed, mouth


DOWN = np.diag([1., -1., -1.])   # gripper approach axis pointing down, fingers closing along world x


def gripper(tcp):
    pose = np.eye(4); pose[:3, :3] = DOWN; pose[:3, 3] = np.asarray(tcp) - DOWN @ [0, 0, .134]
    return pose.tolist()


def strings(v, mouth, r0, pulled, tcp, held, slack=False):
    """Hem markers on the mouth ring and two strap loops whose strand ends sit
    above the cut-outs, drawn out by enough string to keep the routed length."""
    neck = v[mouth]; c = neck.mean(0); r = float(np.linalg.norm(neck[:, :2] - c[:2], axis=1).max())
    a = np.arange(20) * 2 * np.pi / 20
    hem = np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(20, c[2])]
    h = .02 + (2 * np.pi * (r0 - r) / 4 + .00125 if pulled else 0.)
    loops = []
    for k, cut in enumerate((5, 15)):
        ea = hem[cut] + [.006, 0, h]; eb = hem[cut] + [-.006, 0, h]
        top = np.asarray(tcp[k]) if held else (ea + eb) / 2 + [0, 0, .02]
        loops.append(np.array([ea, (ea + top) / 2, top, (eb + top) / 2, eb]).tolist())
    return hem.tolist(), loops


def episode(cinch=True, lift=.27, place=True, release=True, spill_at=None, tipped=False, nan_at=None, end=260, reopen=False,
            pull=True, hold=True, film_grip_at=None, film_touch=False):
    initial, gathered, mouth = gathered_mesh()
    neck0 = initial[mouth]; r0 = float(np.linalg.norm(neck0[:, :2] - neck0[:, :2].mean(0), axis=1).max())
    trace, snaps = [], {}
    for step in range(0, end + 1, 10):
        c = np.clip((step - 30) / 30, 0, 1) if cinch else 0.
        up = lift * np.clip((step - 60) / 30, 0, 1) - (lift * np.clip((step - 120) / 30, 0, 1) if place else 0.)
        shift = np.array([-.25, 0., 0.]) * (np.clip((step - 100) / 20, 0, 1) if place else 0.)
        if reopen and step >= 180:
            c = .3
        v = initial * (1 - c) + gathered * c
        offset = np.array([.22, 0, .758 + up]) + shift
        if place and step >= 150:
            offset[2] = .7475 + .003
        if tipped and step >= 150:
            v = v[:, [2, 1, 0]] * [1, 1, 1] + [0, 0, 0]
            v = v - [v[:, 0].mean(), 0, v[:, 2].min()]
        v = v + offset
        centres = [list(np.add(b, [0, 0, v[:, 2].min() - .758])) for b in block_positions()]
        centres = [np.add(cc, [shift[0], 0, 0]).tolist() for cc in centres]
        if tipped and step >= 150:
            centres = [[v[:, 0].mean() + .08 * (i - 1), 0, v[:, 2].min() + .045] for i in range(len(block_positions()))]
        if spill_at is not None and step >= spill_at:
            centres[0] = [.6, .3, .76]
        if nan_at is not None and step == nan_at:
            v = v.copy(); v[0, 0] = np.nan
        neck = v[mouth]
        held = hold and not (release and step >= 170)
        tcp = [[v[:, 0].mean(), .06, v[:, 2].max() + .14], [v[:, 0].mean(), -.06, v[:, 2].max() + .14]] if held else \
              [[.0, .2, 1.3], [.0, -.2, 1.3]]
        opening = [.004, .004] if held else [.09, .09]
        hem, loops = strings(v, mouth, r0, pull and step >= 30, tcp, held)
        if film_grip_at is not None and step >= film_grip_at and step < 170:
            # one hand pinches the film at the mouth instead of (or as well as) a loop
            tcp[1] = v[mouth][0].tolist(); opening[1] = .004
        if film_touch and step >= 70:
            # the outside of one finger rests on the gathered neck (no film between the pads)
            edge = v[mouth][np.argmax(v[mouth][:, 0])]
            tcp[0] = (edge + [.016, 0, .010]).tolist()
            hem, loops = strings(v, mouth, r0, pull and step >= 30, tcp, held)
        trace.append({'step': step, 'mouth_diameter': float(np.nanmax(np.linalg.norm(neck[:, None] - neck[None], axis=2))),
                      'rubbish': centres, 'rubbish_quaternions': [[0, 0, 0, 1]] * len(block_positions()), 'tcp': tcp,
                      'gripper_poses': [gripper(x) for x in tcp], 'finger_opening': opening,
                      'hem_marker_positions': hem, 'loop_points': loops})
        snaps[step] = v
    return trace, snaps, PARAMS


class DrawstringScoreTests(unittest.TestCase):
    def test_full(self):
        r = evaluate(*episode())
        self.assertEqual(r['score'], 1., r)

    def test_never_cinched_is_zero(self):
        self.assertEqual(evaluate(*episode(cinch=False))['score'], 0.)

    def test_cinched_only(self):
        self.assertEqual(evaluate(*episode(lift=0., place=False))['score'], .25)

    def test_carried_not_set_down(self):
        self.assertEqual(evaluate(*episode(place=False, release=False))['score'], .5)

    def test_set_down_still_held(self):
        self.assertEqual(evaluate(*episode(release=False))['score'], .75)

    def test_spill_latches_zero(self):
        r = evaluate(*episode(spill_at=100))
        self.assertEqual(r['score'], 0.)
        self.assertIn('outside', r['reason'])

    def test_tipped_bag_not_set_down(self):
        self.assertLess(evaluate(*episode(tipped=True))['score'], .75)

    def test_mouth_reopening_after_release_is_not_set_down(self):
        r = evaluate(*episode(reopen=True))
        self.assertGreater(r['final_mouth_diameter_m'], .060)
        self.assertEqual(r['score'], .5)

    def test_invalid_geometry_zero(self):
        self.assertEqual(evaluate(*episode(nan_at=200))['score'], 0.)

    def test_squeezed_shut_without_pulling_earns_nothing(self):
        """Loops held but the string never drawn out: the film closed some other way."""
        r = evaluate(*episode(pull=False))
        self.assertEqual(r['score'], 0., r)

    def test_loops_not_held_earns_nothing(self):
        r = evaluate(*episode(hold=False))
        self.assertEqual(r['score'], 0., r)

    def test_film_gripped_before_cinch_voids_credit(self):
        r = evaluate(*episode(film_grip_at=20))
        self.assertEqual(r['score'], 0., r)
        self.assertEqual(r['film_gripped_step'], 20)

    def test_film_gripped_during_lift_voids_carry(self):
        r = evaluate(*episode(film_grip_at=70))
        self.assertEqual(r['score'], .25, r)

    def test_hand_touching_film_while_lifting_voids_carry(self):
        r = evaluate(*episode(film_touch=True, place=False, release=False))
        self.assertEqual(r['score'], .25, r)

    def test_missing_grasp_evidence_is_invalid(self):
        t, s, p = episode()
        del t[3]['gripper_poses']
        self.assertEqual(evaluate(t, s, p)['score'], 0.)

    def test_missing_samples_zero(self):
        t, s, p = episode()
        del t[5]
        self.assertEqual(evaluate(t, s, p)['score'], 0.)


if __name__ == '__main__':
    unittest.main()
