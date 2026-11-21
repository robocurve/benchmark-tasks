"""Offline controls and tier tests; no simulator needed.  Run: python3 -m unittest test_scoring.py"""

import math
import unittest

import numpy as np

from scoring import (CAP_Z0, FOIL_Z, SPIKE_TIP_Z, Evidence, Frame, Tracker, score_evidence)

TOP = 0.75
TUBE = np.array([0.30, 0.0, TOP + 0.100])
FAR = np.array([0.0, 0.4, TOP + 0.3])       # a parked gripper
PLANE = TUBE[2] + FOIL_Z
FOIL_REST = np.array([TUBE[0], TUBE[1], PLANE - 0.0006])
SEATED = TUBE + np.array([0, 0, CAP_Z0])
FLIPPED = np.diag([1.0, -1.0, -1.0])        # a half turn about x: spike points down


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def tilted(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]) @ FLIPPED


def frame(cap, rot=None, foil=None, tcps=None, pads=None, still=True):
    return Frame(np.asarray(cap, float), np.eye(3) if rot is None else rot,
                 FOIL_REST.copy() if foil is None else np.asarray(foil, float),
                 [] if pads is None else pads, [FAR] if tcps is None else tcps, still)


def over_nozzle(depth, rot=None, dx=0.0):
    """Cap origin for a flipped cap whose spike tip is ``depth`` below the foil face."""
    return np.array([TUBE[0] + dx, TUBE[1], PLANE - depth + SPIKE_TIP_Z])


class Episode:
    def __init__(self):
        self.t = Tracker(TUBE, TOP)

    def run(self, frames):
        for f in frames:
            self.t.update(f)
        return self.t.score()

    # building blocks ------------------------------------------------------------------------------------------
    def unscrew(self, degrees=560):
        out = []
        for k in range(0, degrees + 1, 20):
            lift = min(k, 540) / 540 * 0.00225
            out.append(frame(SEATED + [0, 0, lift], rz(math.radians(k))))
        return out

    def lift_and_flip(self):
        out = [frame(SEATED + [0, 0, 0.0025 + 0.005 * k], rz(math.radians(560))) for k in range(1, 6)]
        out += [frame(over_nozzle(-0.008, dx=0.0), FLIPPED)]
        return out

    def pierce(self, depth=0.0025, rot=None, dx=0.0):
        r = FLIPPED if rot is None else rot
        pushed = FOIL_REST - [0, 0, depth]
        return [frame(over_nozzle(d, dx=dx), r, foil=pushed if d >= 0.0005 else None)
                for d in np.linspace(-0.004, depth, 8)]

    def retract_release_hold(self):
        out = [frame(over_nozzle(0.0025 - k * 0.006), FLIPPED, foil=FOIL_REST - [0, 0, 0.0025]) for k in range(1, 8)]
        parked = np.array([0.25, 0.10, TOP + 0.012])
        out += [frame(parked, rz(1.0), foil=FOIL_REST - [0, 0, 0.0025], still=True) for _ in range(25)]
        return out


class Controls(unittest.TestCase):
    def test_untouched_is_zero(self):
        ep = Episode()
        self.assertEqual(ep.run([frame(SEATED) for _ in range(100)]), 0.0)

    def test_clean_pierce_is_one(self):
        ep = Episode()
        s = ep.run(ep.unscrew() + ep.lift_and_flip() + ep.pierce() + ep.retract_release_hold())
        self.assertEqual(s, 1.0)

    def test_body_squeeze_is_zero_and_latched(self):
        ep = Episode()
        pad = TUBE + [0.0, 0.010, -0.040]  # finger pad on the flat of the tube body
        clean = ep.unscrew() + ep.lift_and_flip() + ep.pierce() + ep.retract_release_hold()
        self.assertEqual(ep.run([frame(SEATED, pads=[pad])] + clean), 0.0)
        self.assertTrue(ep.t.evidence.squeezed)

    def test_squeeze_mid_episode_cannot_be_undone(self):
        ep = Episode()
        frames = ep.unscrew()
        frames += [frame(frames[-1].cap_pos, frames[-1].cap_rot, pads=[TUBE + [0.0, 0.0, -0.05]])]
        frames += ep.lift_and_flip() + ep.pierce() + ep.retract_release_hold()
        self.assertEqual(ep.run(frames), 0.0)

    def test_off_axis_shear_is_zero(self):
        ep = Episode()
        frames = ep.unscrew() + ep.lift_and_flip() + ep.pierce(rot=tilted(15.0)) + ep.retract_release_hold()
        self.assertEqual(ep.run(frames), 0.0)
        self.assertTrue(ep.t.evidence.sheared)

    def test_decentred_pierce_earns_no_puncture(self):
        ep = Episode()
        frames = ep.unscrew() + ep.lift_and_flip() + ep.pierce(dx=0.004) + ep.retract_release_hold()
        self.assertLessEqual(ep.run(frames), 0.50)

    def test_foil_poked_without_spike_is_zero(self):
        ep = Episode()
        frames = ep.unscrew() + [frame(SEATED + [0, 0, 0.05], rz(0.0), foil=FOIL_REST - [0, 0, 0.003])]
        self.assertEqual(ep.run(frames), 0.0)
        self.assertTrue(ep.t.evidence.foil_tampered)

    def test_cap_dropped_off_bench_is_zero(self):
        ep = Episode()
        frames = ep.unscrew() + [frame([0.2, 0.1, TOP - 0.2], rz(0.3))]
        self.assertEqual(ep.run(frames + ep.pierce()), 0.0)
        self.assertTrue(ep.t.evidence.dropped)

    def test_axial_pull_without_turning_does_not_detach(self):
        ep = Episode()
        self.assertEqual(ep.run([frame(SEATED + [0, 0, 0.04 + 0.001 * k]) for k in range(20)]), 0.0)
        self.assertFalse(ep.t.thread_released)

    def test_clockwise_turning_does_not_unthread(self):
        ep = Episode()
        ep.run([frame(SEATED, rz(-math.radians(k))) for k in range(0, 1100, 20)])
        self.assertFalse(ep.t.thread_released)


class Tiers(unittest.TestCase):
    def test_tier_025_detached(self):
        ep = Episode()
        s = ep.run(ep.unscrew() + [frame(SEATED + [0, 0, 0.040], rz(1.0))])
        self.assertEqual(s, 0.25)

    def test_tier_050_inverted_and_aligned(self):
        ep = Episode()
        self.assertEqual(ep.run(ep.unscrew() + ep.lift_and_flip()), 0.50)

    def test_tier_075_punctured(self):
        ep = Episode()
        s = ep.run(ep.unscrew() + ep.lift_and_flip() + ep.pierce())
        self.assertEqual(s, 0.75)

    def test_tier_100_requires_release(self):
        ep = Episode()
        near = [np.array([0.30, 0.0, PLANE + 0.02])]
        frames = ep.unscrew() + ep.lift_and_flip() + ep.pierce() + [
            Frame(f.cap_pos, f.cap_rot, f.foil_pos, [], near, True) for f in ep.retract_release_hold()]
        self.assertEqual(ep.run(frames), 0.75)

    def test_tier_100_requires_one_second_stillness(self):
        ep = Episode()
        hold = ep.retract_release_hold()
        for f in hold[-6:]:
            f.still = False
        self.assertEqual(ep.run(ep.unscrew() + ep.lift_and_flip() + ep.pierce() + hold), 0.75)

    def test_tier_100_requires_retraction(self):
        ep = Episode()
        shallow = [frame(over_nozzle(0.0025), FLIPPED, foil=FOIL_REST - [0, 0, 0.0025]) for _ in range(25)]
        self.assertEqual(ep.run(ep.unscrew() + ep.lift_and_flip() + ep.pierce() + shallow), 0.75)

    def test_shallow_touch_is_not_a_puncture(self):
        ep = Episode()
        self.assertEqual(ep.run(ep.unscrew() + ep.lift_and_flip() + ep.pierce(depth=0.0008)), 0.50)

    def test_score_policy_alone(self):
        self.assertEqual(score_evidence(Evidence(detached=True, aligned=True, punctured=True, retracted=True,
                                                 released=True, still_hold=True)), 1.0)
        self.assertEqual(score_evidence(Evidence(detached=True, aligned=True, punctured=True, retracted=True,
                                                 released=True, still_hold=True, squeezed=True)), 0.0)
        self.assertEqual(score_evidence(Evidence(punctured=True)), 0.0)  # no skipping tiers


if __name__ == "__main__":
    unittest.main()
