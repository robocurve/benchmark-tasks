"""The MIDI scorer on synthetic trials written as MIDI files and read back, one for every control level and every rule
edge. Standard library only (the plot test needs matplotlib and is skipped without it):

    python -I -m unittest discover -s tests
"""

import json
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rdb_drum_groove import judge_midi as J  # noqa: E402
from rdb_drum_groove import layout as L  # noqa: E402
from rdb_drum_groove.midi import read_midi, write_midi  # noqa: E402

VEL = {"loud": 90, "soft": 50, "medium": 75}
STICK = {"hihat": "left", "snare": "right", "tom": "right"}


def trial(bpm=75., t0=1.2, notes=L.NOTES, jitter=None, vel=None, extra=(), drop=(), shift=None, sticks=None,
          end_after=.3, seed=0):
    """(note_ons, events) of a synthetic trial: every written note (bar those in ``drop``) at its time on a ``bpm``
    grid starting at ``t0``, plus ``extra`` [(t, pad, velocity, stick)]."""
    rng = random.Random(seed)
    p = 60. / bpm
    ons, hits = [], []
    for i, (b, pad, dyn) in enumerate(notes):
        if i in drop:
            continue
        t = t0 + b * p
        if jitter:
            t += jitter(i, rng)
        if shift:
            t += shift(i, b, pad)
        v = (vel or VEL).get(dyn, 70) if not callable(vel) else vel(i, b, pad, dyn)
        stick = (sticks or {}).get(i, STICK[pad])
        ons.append((t, L.NOTE_OF_PAD[pad], v))
        hits.append({"t": t, "note": L.NOTE_OF_PAD[pad], "stick": stick})
    for t, pad, v, stick in extra:
        ons.append((t, L.NOTE_OF_PAD[pad], v))
        hits.append({"t": t, "note": L.NOTE_OF_PAD[pad], "stick": stick})
    last = max((t for t, n, _ in ons), default=0.)
    events = {"hits": hits, "end": {"clean_from": t0 + L.LAST_PAIR * p + end_after if end_after is not None else None}}
    return ons, events


def judged(ons, events, **kw):
    """Through a MIDI file, as the real scorer reads it."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "trial.mid"
        write_midi(path, ons)
        back = read_midi(path)
    return J.judge(back, events, **kw)


class Levels(unittest.TestCase):
    """The controls' expected scores (A2 section 5.4, with the lead's rulings: rest caps at 0.75)."""

    def test_nominal_1(self):
        r = judged(*trial(jitter=lambda i, rng: rng.uniform(-.006, .006)))
        self.assertEqual(r["score"], 1., r["why"])
        self.assertEqual((r["H"], r["G"], r["E"]), (41, 8, 0))
        self.assertAlmostEqual(r["bpm"], 75., delta=.05)

    def test_flat_09(self):
        r = judged(*trial(vel=lambda i, b, pad, dyn: 70 if pad == "hihat" else 95))
        self.assertEqual(r["score"], .9, r["why"])
        self.assertFalse(r["accent"])

    def test_tempo82_075(self):
        r = judged(*trial(bpm=82.))
        self.assertEqual(r["score"], .75, r["why"])
        self.assertAlmostEqual(r["bpm"], 82., delta=.05)

    def test_no_fill_075(self):
        drop = [i for i, n in enumerate(L.NOTES) if n[0] >= 15.]
        r = judged(*trial(drop=drop, end_after=None))
        self.assertEqual((r["H"], r["G"]), (37, 7))
        self.assertEqual(r["score"], .75, r["why"])

    def test_flam_bar3_05(self):
        r = judged(*trial(shift=lambda i, b, pad: .045 if pad == "snare" and b in (9., 11.) else 0.))
        self.assertEqual(r["G"], 6)
        self.assertEqual(r["score"], .5, r["why"])

    def test_gripper_05(self):
        ons, ev = trial()
        ev["arm_contacts"] = [{"t": 5.0, "part": "right gripper", "pad": "snare_rim"}]
        r = judged(ons, ev)
        self.assertEqual(r["score"], .5, r["why"])

    def test_one_arm_025(self):
        drop = [i for i, n in enumerate(L.NOTES) if n[1] != "hihat"]
        r = judged(*trial(drop=drop))
        self.assertEqual((r["H"], r["G"]), (31, 0))
        self.assertEqual(r["score"], .25, r["why"])

    def test_air_and_press_0(self):
        r = judged([], {"end": {"clean_from": 15.}})
        self.assertEqual(r["score"], 0.)

    def test_rest_075(self):
        ons, ev = trial()
        ev["rests"] = [{"t": 2.0 + 1.6 * k, "stick": "right", "pad": "snare", "duration_s": .6} for k in range(8)]
        r = judged(ons, ev)
        self.assertEqual(r["score"], .75, r["why"])

    def test_flail_0(self):
        rng = random.Random(3)
        ons = sorted((rng.uniform(1., 14.), L.NOTE_OF_PAD[rng.choice(["hihat", "snare", "tom"])], rng.randint(30, 120))
                     for _ in range(78))
        r = judged(ons, {"hits": [{"t": t, "note": n, "stick": "left"} for t, n, _ in ons]})
        self.assertEqual(r["score"], 0., r["why"])

    def test_no_stop_0(self):
        p = .8
        t_end = 1.2 + 16 * p
        extra = [(t_end + p * (1 + k / 2), "hihat", 80, "left") for k in range(8)]
        extra += [(t_end + p * 2, "snare", 90, "right"), (t_end + p * 4, "snare", 90, "right")]
        r = judged(*trial(extra=extra))
        self.assertEqual(r["E"], 10)
        self.assertEqual(r["score"], 0., r["why"])


class Rules(unittest.TestCase):
    def test_count_in_is_not_extra(self):
        extra = [(4.0 - .8 * k, "hihat", 60, "left") for k in (4, 3, 2, 1)]
        r = judged(*trial(t0=4.0, extra=extra))
        self.assertEqual(r["count_in"], 4)
        self.assertEqual(r["score"], 1., r["why"])

    def test_one_extra_allowed_two_not(self):
        r1 = judged(*trial(extra=[(3.3, "tom", 60, "right")]))
        self.assertEqual((r1["E"], r1["score"]), (1, 1.))
        r2 = judged(*trial(extra=[(3.3, "tom", 60, "right"), (6.5, "tom", 60, "right")]))
        self.assertEqual((r2["E"], r2["score"]), (2, .75))

    def test_rim_is_a_wrong_hit(self):
        ons, ev = trial()
        ons.append((5.0, 40, 80))
        r = judged(ons, ev)
        self.assertEqual(r["wrong"], 1)

    def test_a_missed_soft_note_is_075(self):
        i = next(i for i, n in enumerate(L.NOTES) if n[2] == "soft")
        r = judged(*trial(drop=[i]))
        self.assertEqual((r["H"], r["score"]), (40, .75))

    def test_a_pair_from_one_stick_is_not_together(self):
        i = next(i for i, n in enumerate(L.NOTES) if n[0] == 5. and n[1] == "hihat")
        r = judged(*trial(sticks={i: "right"}))
        self.assertEqual((r["G"], r["score"]), (7, .75))

    def test_rms_over_25_ms_is_075(self):
        # every note on a count 30 ms early or late, alternately (pairs stay together)
        r = judged(*trial(jitter=lambda i, rng: .03 if int(2 * L.NOTES[i][0]) % 2 else -.03))
        self.assertGreater(r["rms_ms"], 25.)
        self.assertEqual(r["score"], .75, r["why"])

    def test_pair_spread_edge(self):
        ok = judged(*trial(shift=lambda i, b, pad: .038 if pad == "snare" and b == 5. else 0.))
        bad = judged(*trial(shift=lambda i, b, pad: .042 if pad == "snare" and b == 5. else 0.))
        self.assertEqual((ok["G"], bad["G"]), (8, 7))

    def test_tempo_edges(self):
        self.assertEqual(judged(*trial(bpm=71.))["score"], 1.)
        self.assertEqual(judged(*trial(bpm=79.))["score"], 1.)
        self.assertEqual(judged(*trial(bpm=70.))["score"], .75)
        self.assertEqual(judged(*trial(bpm=60.))["score"], .75)
        self.assertLess(judged(*trial(bpm=96.))["score"], .5)

    def test_double_note_on_is_merged(self):
        r = judged(*trial(extra=[(1.2 + .8 + .02, "hihat", 40, "left")]))
        self.assertEqual((r["E"], r["score"]), (0, 1.))

    def test_hits_after_the_last_pair_spoil_the_end(self):
        r = judged(*trial(extra=[(1.2 + 16 * .8 + .7, "tom", 60, "right")]))
        self.assertFalse(r["clean_end"])
        self.assertEqual(r["score"], .75)

    def test_late_clean_end(self):
        self.assertEqual(judged(*trial(end_after=3.5))["score"], .75)
        self.assertEqual(judged(*trial(end_after=2.9))["score"], 1.)

    def test_real_trial_events(self):
        ons, _ = trial()
        ev = {"pairs": {str(b): True for b in L.PAIRS}, "end": {"clean": True}}
        self.assertEqual(judged(ons, ev)["score"], 1.)
        ev2 = dict(ev, pairs={**ev["pairs"], "7.0": False})
        self.assertEqual(judged(ons, ev2)["G"], 7)
        self.assertEqual(judged(ons, dict(ev, pad_moved=True))["score"], 0.)
        # a note sent by a gripper plays no note
        ons2 = ons + [(4.4, 38, 70)]
        ev3 = dict(ev, removed=[{"t": 4.4, "note": 38, "why": "gripper"}])
        self.assertEqual(judged(ons2, ev3)["E"], 0)

    def test_without_stick_records_pairs_do_not_count(self):
        ons, _ = trial()
        r = judged(ons, {"end": {"clean": True}})
        self.assertEqual(r["G"], 0)

    def test_pedal_is_ignored(self):
        ons, ev = trial()
        r = judged(ons + [(3.0, 44, 30)], ev)
        self.assertEqual((r["E"], r["score"]), (0, 1.))

    def test_csv_and_title_are_ascii(self):
        ons, ev = trial()
        r = judged(ons, ev)
        title = J.title_line(r, "scripted-1")
        self.assertTrue(title.isascii())
        self.assertIn("score 1.00", title)
        self.assertIn("notes 41/41", title)
        with tempfile.TemporaryDirectory() as tmp:
            J.write_csv(r, Path(tmp) / "scored-hits.csv")
            text = (Path(tmp) / "scored-hits.csv").read_text()
        self.assertTrue(text.isascii())
        self.assertEqual(len(text.strip().splitlines()), 42)


class Plot(unittest.TestCase):
    def test_plot_text_is_english_ascii(self):
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest("matplotlib not installed")
        from rdb_drum_groove.plot import plot_score

        ons, ev = trial(extra=[(5.0, "snare", 60, "right")])
        r = judged(ons, ev)
        with tempfile.TemporaryDirectory() as tmp:
            texts = plot_score(r, Path(tmp) / "p.png", "scripted-1")
            self.assertTrue((Path(tmp) / "p.png").stat().st_size > 10000)
        self.assertTrue(all(t.isascii() for t in texts))
        self.assertTrue(any(t.startswith("Written score: slow 4/4 rock-ballad groove, 41 notes at 75 bpm.") and
                            t.endswith("Diagnostic plot; not a policy input.") for t in texts))
        self.assertFalse(any("review" in t.lower() for t in texts))


if __name__ == "__main__":
    unittest.main()
