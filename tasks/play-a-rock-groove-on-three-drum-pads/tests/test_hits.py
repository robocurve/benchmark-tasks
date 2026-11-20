"""The trigger rule and the MIDI file, standard library only: python -I -m unittest discover -s tests"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rdb_drum_groove import hits as H  # noqa: E402
from rdb_drum_groove import layout as L  # noqa: E402
from rdb_drum_groove.midi import read_midi, write_midi  # noqa: E402

DT = 1. / 240
ZONES = {"snare_head": ("snare", "snare"), "snare_rim": ("snare", "snare_rim")}


def run(events, steps=200):
    """events: {step: (impulse, approach speed before, height)}; between events the stick is clear above the head."""
    trig = H.Trigger(ZONES, ["stick_right"], DT)
    speed = 0.
    for k in range(1, steps + 1):
        e = events.get(k)
        imp = {("stick_right", "snare_head"): e[0]} if e else {}
        height = e[2] if e else .01
        trig.step(k, imp, {("stick_right", "snare_head"): (events.get(k + 1, (0, 0, 0))[1])},
                  {("stick_right", "snare_head"): height, ("stick_right", "snare_rim"): None})
    return trig


class TriggerTest(unittest.TestCase):
    def test_one_clean_hit(self):
        t = run({50: (.02, 1.2, 0.)})
        self.assertEqual(len(t.hits), 1)
        self.assertEqual(t.hits[0]["velocity"], L.midi_velocity(1.2))

    def test_bounce_inside_mask_is_one_hit(self):
        t = run({50: (.02, 1.2, 0.), 54: (.01, .9, 0.), 57: (.005, .6, 0.)})      # 17 and 29 ms later
        self.assertEqual(len(t.hits), 1)
        self.assertEqual([c.get("rejected") for c in t.contacts], [None, "masked", "masked"])

    def test_two_strokes_80_ms_apart(self):
        t = run({50: (.02, 1.2, 0.), 70: (.02, 1.0, 0.)})
        self.assertEqual(len(t.hits), 2)

    def test_slow_press_and_rest(self):
        events = {k: (.003, .2, 0.) for k in range(50, 400)}                  # pressed down slowly and left there
        t = run(events, 420)
        self.assertEqual(len(t.hits), 0)
        self.assertEqual(t.contacts[0]["rejected"], "slow")
        self.assertGreaterEqual(t.longest_rest[("stick_right", "snare_head")], 240)

    def test_rearm_needs_clearance(self):
        # contact, then still within 2 mm of the head for the next steps, then a fast second contact after the mask
        trig = H.Trigger(ZONES, ["stick_right"], DT)
        key = ("stick_right", "snare_head")
        for k in range(1, 120):
            imp = {key: .02} if k in (10, 30) else {}
            trig.step(k, imp, {key: 1.0}, {key: .001, ("stick_right", "snare_rim"): None})
        self.assertEqual(len(trig.hits), 1)
        self.assertEqual(trig.contacts[1]["rejected"], "not_rearmed")

    def test_impulse_split_over_two_steps_is_one_hit(self):
        t = run({50: (.001, 1.0, 0.), 51: (.01, .2, 0.)})
        self.assertEqual(len(t.hits), 1)
        self.assertEqual(t.hits[0]["step"], 50)
        self.assertEqual(len(t.contacts), 1)

    def test_not_held_never_plays(self):
        trig = H.Trigger(ZONES, ["stick_right"], DT)
        key = ("stick_right", "snare_head")
        trig.step(1, {}, {key: 1.5}, {key: .01, ("stick_right", "snare_rim"): None})
        trig.step(2, {}, {key: 1.5}, {key: .01, ("stick_right", "snare_rim"): None})
        trig.step(3, {key: .02}, {key: 0.}, {key: 0., ("stick_right", "snare_rim"): None}, held={"stick_right": False})
        self.assertEqual(len(trig.hits), 0)
        self.assertEqual(trig.contacts[0]["rejected"], "not_held")

    def test_low_impulse_pair_is_not_a_hit(self):
        t = run({50: (1e-4, 1.2, 0.)})
        self.assertEqual(len(t.hits), 0)

    def test_midi_round_trip(self):
        ons = [(.5 + .4 * i, 42 if i % 2 else 38, 40 + i) for i in range(8)]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "hits.mid"
            write_midi(path, ons)
            back = read_midi(path)
        self.assertEqual(len(back), 8)
        for (t, n, v), (t2, n2, v2) in zip(ons, back):
            self.assertAlmostEqual(t, t2, delta=.0006)
            self.assertEqual((n, v), (n2, v2))

    def test_velocity_curve(self):
        self.assertEqual(L.midi_velocity(.5), 20)
        self.assertEqual(L.midi_velocity(3.), 127)
        self.assertAlmostEqual(L.speed_for_velocity(L.midi_velocity(1.4)), 1.4, delta=.02)
        # 25 velocity steps need a speed ratio of exp(25 ln 6 / 107) = 1.52
        self.assertAlmostEqual(L.speed_for_velocity(70) / L.speed_for_velocity(45), 1.52, delta=.01)

    def test_written_groove(self):
        self.assertEqual(len(L.NOTES), 41)
        self.assertEqual(sum(n[1] == "hihat" for n in L.NOTES), 31)
        self.assertEqual(sum(n[1] == "snare" for n in L.NOTES), 8)
        self.assertEqual(sum(n[1] == "tom" for n in L.NOTES), 2)
        for b in L.PAIRS:
            self.assertEqual({n[1] for n in L.NOTES if n[0] == b}, {"hihat", "snare"})


if __name__ == "__main__":
    unittest.main()
