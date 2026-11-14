"""Offline checks of the open-loop scripted demo in the kinematic world of world.py (about 90 s per full run): full
score at the canonical seed it is scripted for (GNOCCHI_SEED=0), a laggy arm, and that the demo's code reads no object
state at run time (it is a fixed list of waypoints built from the task file's constants).

    python -m pytest -q test_demo.py
"""
import ast
from pathlib import Path

import world as W

M = W.M
HERE = Path(__file__).parent


def test_demo_is_a_fixed_plan_within_the_run_limit():
    plan = M.demo()
    assert set(plan) == {"right", "left"}
    for side in plan:
        steps = sum(w.duration for w in plan[side]) / M.DT
        assert steps <= M.RUN_LIMIT_STEPS - 20, (side, steps)


def test_full_score_canonical_seed():
    w, _ = W.run_demo(seed=0)
    assert w.scores[-1] == 1.0, [(n, r.t_s, r.t_o) for n, r in M._L.p.items()]
    assert not M.hand_on_spoon(w.objs)  # the spoon is let go on the counter
    assert M.burner_off(w.objs) and M.knob_angle(w.objs) == 0.0  # the knob turned back into its OFF click
    # the knob is turned only after the last piece is off the spoon
    assert all(r.t_o is not None for r in M._L.p.values())


def test_laggy_arm_still_full_score():
    w, _ = W.run_demo(seed=0, lag=0.3)  # the hand closes only 70% of the gap to its goal each step
    assert w.scores[-1] == 1.0


def test_other_seed_is_not_scripted_for():
    # the demo is open loop for seed 0 only: with the cooking order reversed it takes raw pieces and scores 0
    truth = dict(zip(M.G_NAMES, reversed(M.COOK_AT)))
    w, _ = W.run_demo(seed=0, cook_override=truth)
    assert w.scores[-1] < 1.0


def test_demo_code_reads_no_object_state():
    # parse task.py itself: the real yam_arena loader exec's it without a sys.modules entry (no inspect.getsource)
    text = (HERE / "task.py").read_text()
    wanted = {"demo", "scoop_schedule", "_lift_and_plate", "plan_scoop", "spoon_clear", "hold", "knob_plan"}
    nodes = [n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name in wanted]
    assert {n.name for n in nodes} == wanted
    names = set()
    for node in nodes:
        names |= {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
        names |= {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}
        names |= {a.arg for a in ast.walk(node) if isinstance(a, ast.arg)}
    for bad in ("objects", "fk", "_L", "state", "surface_z", "pos_now", "trial_seed"):
        assert bad not in names, bad
