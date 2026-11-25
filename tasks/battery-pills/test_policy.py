"""Demo-policy tests without the simulator: synthetic right-wrist frames ray-cast through the calibrated camera model,
perfect joint tracking. Checks that the disc fit recovers the drawn cell, tablets and dish centre, that every planned
pose is reachable by URDF IK, and that the policy reads nothing but images and joint readings.
Needs yam_arena importable (its pure-Python parts: URDF, IK, station calibration); skipped otherwise."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

import geometry as G

try:
    import yam_arena  # noqa: F401
    HAVE_YAM = True
except ImportError:
    HAVE_YAM = False

OAK = np.array([205, 192, 164], np.uint8)
BLUE = np.array([40, 80, 175], np.uint8)
WHITE = np.array([238, 238, 234], np.uint8)
GREY = np.array([128, 130, 134], np.uint8)


def render(cam, lay):
    """Synthetic frame: oak table, blue dish (floor disc + wall seen as the rim disc), disc tops."""
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    us, vs = uu.ravel(), vv.ravel()
    rays = cam.rays(us, vs)
    img = np.tile(OAK, (len(us), 1))
    dx, dy = np.array(lay['dish']) * G.MM

    def hit(z_card):
        z = G.TABLE_TOP + z_card * G.MM
        t = (z - cam.pos[2]) / rays[:, 2]
        return cam.pos + rays * t[:, None]

    rim = hit(G.DISH_H)
    img[np.hypot(rim[:, 0] - dx, rim[:, 1] - dy) < G.DISH_TOP_R * G.MM] = BLUE
    discs = [(t, G.TAB_R, G.TAB_H, WHITE) for t in lay['tablets']] + [(lay['cell'], G.CELL_R, G.CELL_H, GREY)]
    for (x, y, z), r, h, col in discs:
        top = hit(z + h / 2)
        img[np.hypot(top[:, 0] - x * G.MM, top[:, 1] - y * G.MM) < r * G.MM] = col
    return img.reshape(cam.height, cam.width, 3)


@unittest.skipUnless(HAVE_YAM, 'yam_arena not importable')
class Perception(unittest.TestCase):
    def test_disc_fit(self):
        import demo_policy as D
        import perception as V
        arm = V.ArmModel('right')
        from yam_arena.ik import solve
        for seed in (0, 1, 2, 3, 5):
            lay = G.layout(seed)
            rot = D.r_down(math.pi / 2)
            floor = G.TABLE_TOP + G.DISH_FLOOR_TOP * G.MM
            nominal = np.array(G.DI['start_centre_mm']) * G.MM
            demo = D.BatteryDemo.__new__(D.BatteryDemo)
            tcp = D.BatteryDemo._survey_tcp(demo, SimpleNamespace(model=arm), (*nominal, floor), D.SURVEY_DEPTH, rot)
            q, err, _ = solve(tcp, D.quat_xyzw(rot), q_init=D.Q_SEED_R, base_pos=arm.base, iters=300)
            self.assertLess(err, 1e-3)
            cam = arm.camera(q)
            fit = V.find_discs(render(cam, lay), cam, G.TABLE_TOP)
            self.assertTrue(fit['ok'], fit.get('reason'))
            self.assertLess(np.linalg.norm(fit['cell'] / G.MM - np.array(lay['cell'][:2])), 1.0, seed)
            c, rms = V.lattice_centre([fit['cell']] + fit['tablets'], nominal)
            self.assertLess(np.linalg.norm(c / G.MM - np.array(lay['dish'])), 2.0, (seed, c / G.MM, lay['dish']))

    def test_closing_angle_clear(self):
        import perception as V
        for seed in range(1, 30):
            lay = G.layout(seed)
            cell = np.array(lay['cell'][:2]) * G.MM
            tabs = [np.array(t[:2]) * G.MM for t in lay['tablets']]
            th, clear = V.best_closing_angle(cell, tabs, 0.0055)
            self.assertGreater(clear, 0.0, (seed, lay['draw']))


class FakeObs:
    def __init__(self, js, images):
        self.state = {'joint_pos': js}
        self.images = images


@unittest.skipUnless(HAVE_YAM, 'yam_arena not importable')
class DryRun(unittest.TestCase):
    def _run(self, seed, mode='demo', steps=2400):
        import demo_policy as D
        import perception as V
        lay = G.layout(seed)
        pol = D.make_policy(320, 176, 20, mode)
        arm = V.ArmModel('right')
        js = np.r_[D.READY, 1.0, D.READY, 1.0]
        n = 0
        while not pol.done and n < steps:
            cam = arm.camera(js[7:13])
            img = render(cam, lay)
            obs = FakeObs(js.copy(), {'right_cam': img, 'left_cam': img, 'top_cam': img})
            a = pol.act(obs)
            js = a.copy()
            name = pol.log[-1].name if pol.log else ''
            if name.startswith('pick') or name.startswith('repick') or name in ('to_jar', 'hover_hold'):
                if js[13] < 0.067:   # a held cell reads 0.067 at the 0.063 command (job 4296409)
                    js[13] = 0.067
            if name in ('rim_grasp', 'carry') and js[6] < 0.032:
                js[6] = 0.032
            n += 1
        return pol, n

    def test_demo_every_phase(self):
        for seed in (0, 1, 4):
            pol, n = self._run(seed)
            names = [p.name for p in pol.log]
            for want in ('survey', 'pick', 'to_jar', 'drop', 'rim_grasp', 'carry', 'release', 'final_hold'):
                self.assertIn(want, names, (seed, names))
            self.assertTrue(pol.done)
            self.assertLess(n, 1600, n)
            rep = pol.report()
            for p in rep['phases']:
                self.assertLess(p.get('worst_ik_mm', 0.0), 1.0, (seed, p))
                self.assertLess(p.get('goto_ik_mm', 0.0), 1.0, (seed, p))
            pick = [p for p in rep['phases'] if p['name'] == 'pick'][0]
            self.assertLess(np.linalg.norm(np.array(pick['cell_mm']) - np.array(G.layout(seed)['cell'][:2])), 1.0)

    def test_hover(self):
        pol, n = self._run(1, 'hover')
        self.assertEqual(pol.log[-1].name, 'hover_hold')
        self.assertTrue(pol.done)

    def test_reads_only_observations(self):
        import inspect

        import demo_policy as D
        src = inspect.getsource(D)
        for banned in ('arena_bp', 'body_pose', 'root_link_pose', 'scene[', 'SimScorer', 'G.layout', 'draws('):
            self.assertNotIn(banned, src)


if __name__ == '__main__':
    unittest.main()
