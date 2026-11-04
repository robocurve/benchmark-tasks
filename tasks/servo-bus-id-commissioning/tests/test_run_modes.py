"""run.sh <-> demo_plan.py glue (C3): parse the environment variables each run.sh mode exports, check demo_plan reads
them, check each mode's plan differs from the default the way the mode claims, and predict each mode's score by driving
the real Scorer with the poses/button travel implied by the plan timeline (the same method as the GATE_F numbers)."""

from __future__ import annotations

import os
import re
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
from scoring import Scorer

Q_SEATED = np.array([0.0, 0.0, np.sin(-np.pi / 4), np.cos(-np.pi / 4)])
ALL_VARS = ("DEMO_SETTLE_ONLY", "DEMO_ALL_PLUGGED", "DEMO_WRONG_ORDER", "DEMO_RECOVER")


def run_sh_modes() -> dict[str, dict[str, str]]:
    """{mode: {ENV_VAR: value}} parsed from the case lines of run.sh."""
    out = {}
    for line in (HERE / "run.sh").read_text().splitlines():
        m = re.match(r"\s*([a-z_]+)\)\s*(.*)$", line)
        if not m or "STEPS=" not in line:
            continue
        out[m.group(1)] = dict(re.findall(r"export (\w+)=(\w+)", m.group(2)))
    return out


def plan_for(env: dict[str, str]):
    saved = {k: os.environ.pop(k, None) for k in ALL_VARS}
    try:
        os.environ.update(env)
        return demo_plan.plans()
    finally:
        for k in env:
            os.environ.pop(k, None)
        for k, v in saved.items():
            if v is not None:
                os.environ[k] = v


def starts(wps):
    t, out = 0.0, []
    for w in wps:
        out.append((t, t + w.duration, w))
        t += w.duration
    return out


def predict(plans, drop_reset: bool = False):
    """Drive the real Scorer from the plan timeline. Returns (final score, max score, final ids, steps)."""
    left, right = starts(plans["left"]), starts(plans["right"])
    total = max(left[-1][1], right[-1][1])
    # servo states from the right arm's waypoints
    seated = {c: [(0.0, True)] for c in ("red", "green", "blue")}
    for a, b, w in right:
        m = re.match(r"r: pull (\w+) out", w.label)
        if m:
            seated[m.group(1)].append((b, False))
        m = re.match(r"r: push (\w+) into socket", w.label)
        if m:
            seated[m.group(1)].append((b, True))
    # button travel from the left arm's press/hold waypoints (cap at its stop between the end of 'press' and the start of 'release')
    presses = []
    for a, b, w in left:
        m = re.match(r"l: press (assign|reset)", w.label)
        if m:
            end = next(bb for aa, bb, ww in left if ww.label == w.label.replace("press", "hold", 1))
            if m.group(1) == "reset" and drop_reset:
                continue
            presses.append((m.group(1), b - 0.1, end))
    scorer = Scorer()
    best, res = 0.0, None
    n = int(round(total * 20)) + 1
    for k in range(n):
        t = k / 20.0
        servos = {}
        for c in ("red", "green", "blue"):
            state = [s for tt, s in seated[c] if tt <= t][-1]
            pos = g.START_POS[c] if state else g.STAGE_POS[c]
            servos[c] = (np.array(pos, float), Q_SEATED, np.zeros(3), np.zeros(3))
        sl = {"assign_btn": 0.0, "reset_btn": 0.0}
        for which, a, b in presses:
            if a <= t <= b:
                sl[f"{which}_btn"] = -g.BUTTON_TRAVEL
        res = scorer.update(np.zeros(3), sl, servos, t >= total - 1.5)
        best = max(best, res.score)
    return res.score, best, res.details["ids"], n


class TestRunShGlue(unittest.TestCase):
    def test_run_sh_modes_present(self) -> None:
        self.assertEqual(set(run_sh_modes()), {"settle", "demo", "all_plugged", "wrong_order", "recover", "dream"})

    def test_every_exported_variable_is_read_by_demo_plan(self) -> None:
        src = (HERE / "demo_plan.py").read_text()
        for mode, env in run_sh_modes().items():
            for var in env:
                self.assertIn(f'"{var}"', src, f"run.sh mode {mode} exports {var} but demo_plan.py never reads it")

    def test_each_mode_changes_the_plan_as_claimed(self) -> None:
        modes = run_sh_modes()
        default = plan_for({})
        self.assertEqual(plan_for(modes["settle"]), {})
        ap = plan_for(modes["all_plugged"])
        self.assertFalse([w for w in ap["right"] if "grip" in w.label], "all_plugged must not touch any module")
        self.assertEqual(len([w for w in ap["left"] if w.label.startswith("l: press assign")]), 1)
        self.assertEqual(len([w for w in ap["left"] if "reset" in w.label]), 0)
        wo = plan_for(modes["wrong_order"])
        unplugs = [re.match(r"r: grip (\w+)( \(2nd\))?$", w.label).group(1) for w in wo["right"] if re.match(r"r: grip (\w+)( \(2nd\))?$", w.label)]
        self.assertEqual(unplugs, ["blue", "red"], "wrong_order leaves GREEN on the bus and numbers it first")
        self.assertEqual(len([w for w in wo["left"] if "reset" in w.label]), 0, "wrong_order never presses RESET")
        rc = plan_for(modes["recover"])
        left_labels = [w.label for w in rc["left"]]
        self.assertIn("l: press reset", left_labels)
        i_reset = left_labels.index("l: press reset")
        self.assertEqual(len([x for x in left_labels[:i_reset] if x.startswith("l: press assign")]), 3, "3 wrong-order ASSIGNs before RESET")
        self.assertEqual(len([x for x in left_labels[i_reset:] if x.startswith("l: press assign")]), 3, "3 correct ASSIGNs after RESET")
        first_grips = [re.match(r"r: grip (\w+)( \(2nd\))?$", w.label).group(1) for w in rc["right"] if re.match(r"r: grip (\w+)( \(2nd\))?$", w.label)]
        self.assertEqual(first_grips[:2], ["blue", "red"])
        self.assertEqual(first_grips[2:4], ["green", "blue"])
        for name, plan in (("wrong_order", wo), ("recover", rc), ("all_plugged", ap)):
            self.assertNotEqual(plan, default, name)

    def test_tuning_knobs_change_the_plan(self) -> None:
        base = plan_for({})
        for var, val in (("DEMO_OVERSHOOT", "0.004"), ("DEMO_PUSH_S", "1.5"), ("DEMO_GRIP_DX", "0.002"), ("DEMO_GRIP_DY", "0.002")):
            self.assertNotEqual(plan_for({var: val}), base, var)

    def test_predicted_scores_per_mode(self) -> None:
        modes = run_sh_modes()
        expect = {"settle": None, "demo": 1.0, "all_plugged": 0.25, "wrong_order": 0.25, "recover": 1.0}
        for mode, want in expect.items():
            plans = plan_for(modes[mode])
            if want is None:
                self.assertEqual(plans, {})
                continue
            final, best, ids, _n = predict(plans)
            self.assertEqual(final, want, f"{mode}: final {final} best {best} ids {ids}")
        _f, _b, ids_wo, _ = predict(plan_for(modes["wrong_order"]))
        self.assertEqual(ids_wo, {"red": 2, "green": 1, "blue": 3})

    def test_recover_fails_without_reset(self) -> None:
        """RESET is essential: the recover plan with its RESET press removed ends at 0.25, not 1.0."""
        plans = plan_for(run_sh_modes()["recover"])
        final, best, ids, _ = predict(plans, drop_reset=True)
        self.assertLess(final, 1.0)
        self.assertEqual(best, 0.25)
        # and with RESET it reaches 1.0 with the right ids
        final2, _b, ids2, _ = predict(plans)
        self.assertEqual((final2, ids2), (1.0, {"red": 1, "green": 2, "blue": 3}))

    def test_recover_reseats_servos_after_reset(self) -> None:
        """After RESET the plan unplugs and re-seats green and blue (the ids can only be right if it does)."""
        rc = plan_for(run_sh_modes()["recover"])
        labels = [w.label for w in rc["right"]]
        n_pull = len([x for x in labels if x.startswith("r: pull ")])
        n_push = len([x for x in labels if x.startswith("r: push ")])
        self.assertEqual((n_pull, n_push), (4, 4))

    def test_steps_at_least_1_25x_plan(self) -> None:
        steps = {}
        for line in (HERE / "run.sh").read_text().splitlines():
            m = re.match(r"\s*([a-z_]+)\).*STEPS=(\d+)", line)
            if m:
                steps[m.group(1)] = int(m.group(2))
        for mode, env in run_sh_modes().items():
            plans = plan_for(env)
            if not plans:
                continue
            n = round(max(sum(w.duration for w in plans[a]) for a in plans) * 20)
            self.assertGreaterEqual(steps[mode], 1.25 * n, f"{mode}: STEPS={steps[mode]} vs plan {n}")
            self.assertLessEqual(n, 3000)


if __name__ == "__main__":
    unittest.main()
