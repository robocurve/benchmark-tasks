"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # inline literals and the handle tests in released/handled are checked as text
assert (t.SCORE_OVER, t.POUR_TILT, t.OVER_CUP, t.ABOVE_MIN, t.ABOVE_MAX, t.SPILL_STEPS, t.FULL_STEPS, t.MIN_STEPS,
        t.LID_SHIFT, t.LID_LIFT, t.LID_TILT, t.LID_CHECK_TILT, t.HOME_TOL, t.HOME_OFF, t.HOME_TILT, t.POT_NEAR,
        t.RELEASE_DIST, t.DROP_FALL, t.DROP_STEPS, t.STILL_MOVE, t.STILL_STEPS, t._POT_C, t._POT_S,
        getattr(t, "_HANDLE_C", None), getattr(t, "_HANDLE_S", None)) == (
        "end", 45.0, 0.025, 0.010, 0.080, 10, 60, 20, 0.008, 0.005, 8.0, 10.0, 0.006, 0.030, 10.0, 0.020, 0.008, 0.05,
        10, 0.002, 20, (0.0, 0.0, 0.040), (0.100, 0.23, 0.080), (-0.080, 0.0, 0.060), (0.060, 0.020, 0.016)) and all(
        s in src for s in ("or kk.touched_by_any(_HANDLE_C, _HANDLE_S, pot, POT_NEAR)",
                           "or kk.touched_by_any(_HANDLE_C, _HANDLE_S, pot, RELEASE_DIST))")), \
    "task.py thresholds changed: update the scoring text"
assert (t.TRIVET_XY, t.TRIVET, t.TRIVET_H, t.POT, t.POT_H, t.POT_T, tuple(t.SPOUT_TIP), t.HANDLE_XYZ, t.HANDLE, t.CAP,
        t.CAP_T, t.KNOB, t.KNOB_H, t.CUP_XY, t.CUP, t.CUP_H, t.CUP_T) == (
        (0.30, 0.07), 0.140, 0.010, 0.100, 0.080, 0.006, (0.0, -0.0995, 0.075), (-0.080, 0.0, 0.060),
        (0.060, 0.020, 0.016), 0.098, 0.006, 0.022, 0.018, (0.30, -0.058), 0.070, 0.045, 0.004) and all(
        s in src for s in ('kk.part("teapot", POT_POS, POT_GEOMS, mass=0.30,',
                           'kk.part("lid", LID_POS, LID_GEOMS, mass=0.05,', "(0.018, 0.024, 0.020), color=POT_C)",
                           "color=TEA, collide=False", "mass=None, color=TRIVET_C", "mass=None, color=CUP_C",
                           "POT_POS = (*TRIVET_XY,", "LID_POS = (*TRIVET_XY,")), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Pour the tea with one hand on the lid",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "pouring", "teapot", "loose lid", "tea service", "hospitality", "unstated rule",
                 "hold and steady"],
    "instruction": t.INSTRUCTION,
    "context": (
        "Tea-service guidance says to hold the teapot's handle with one hand and keep the lid on with the "
        "other while pouring: English Tea Store's guide to pouring British style, Some Tea With Me's 'How to "
        "pour a teapot' (two hands, one on the lid and one on the handle) and Teasenz's Yixing guide (one "
        "hand raises the pot, the other's finger rests on the lid). Here the lid is loose, with no locking "
        "tab, and pouring needs the pot tilted 45 degrees or more. In the recorded one-arm run (the same "
        "pour, lid unheld) the lid slid 17 cm off its seat and scored 0, so one arm tilts the pot by its "
        "handle while the other holds the lid by its knob. The instruction does not mention the lid, so the "
        "task checks whether the robot keeps it on unprompted. Done by: restaurant, cafe, hotel and tea-house"
        " servers, and anyone serving tea at home."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Trivet: brown, 14 x 14 cm, 1 cm thick, fixed to the table, centred at x 30, y 7. Teapot: 0.30 kg, "
        "cream, standing on the trivet: an open square body 10 x 10 cm outside and 8 cm tall, with 6 mm walls and "
        "floor, full of tea (visual only); a spout of three stepped blocks (each 1.8 x 2.4 x 2 cm) on its right "
        "side, toward the cup, its tip 9.95 cm right of the pot's axis and 7.5 cm above the pot's base; and a "
        "straight blue side handle, 6 cm long, 2 cm wide and 1.6 cm thick, sticking out 6 cm from the pot's back "
        "wall toward the robot (its centre 8 cm behind the pot's axis and 6 cm above its base), at a right angle to "
        "the spout, as on a Japanese kyusu. Lid: 0.05 kg, loose, resting on the pot's rim with no locking tab: a "
        "cream cap 9.8 x 9.8 cm, 6 mm thick, with a blue knob 2.2 x 2.2 cm, 1.8 cm tall, at its centre. Cup: white, "
        "empty, fixed to the table, an open square cup 7 x 7 cm outside and 4.5 cm tall with 4 mm walls and floor, "
        "centred at x 30, y -5.8."),
    "scoring": (
        "Every step, from poses; the score at the end counts, and the episode ends once success has held for 1 s. A "
        "pouring step: the pot tilted at least 45 degrees from upright, with its spout tip within 2.5 cm "
        "(horizontally) of the cup's axis and 1 to 8 cm above the cup's rim. The lid is seated when, relative to the "
        "pot, it is within 8 mm (sideways) and 5 mm (up or down) of its seat on the rim and tilted no more than 8 "
        "degrees; it must be seated at every step the pot is tilted 10 degrees or more, and at the end for 1.0. 1.0 "
        "(success): at least 60 pouring steps (3 s at 20 Hz, not necessarily in a row), the lid seated, and the pot "
        "back on its trivet (its base within 6 mm of the trivet's top, within 10 degrees of upright, its centre "
        "within 3 cm of the trivet's centre), released (no finger pad within 8 mm of the pot's body, a 10 x 23 x 8 cm "
        "box from its base to its rim around the body and spout, or of its 6 x 2 x 1.6 cm side handle) and still "
        "(moving less than 2 mm over 1 s). 0.5: at least 60 pouring steps, but not all of the end conditions for 1.0 "
        "met (pot back on the trivet, released, still, lid seated). 0.25: 20 to 59 pouring steps (1 to 3 s). 0 "
        "otherwise, and 0 from the moment the lid is off its seat while the pot is tilted 10 degrees or more, the pot "
        "is tilted 45 degrees or more with the spout not over the cup for 10 steps in a row (0.5 s: pouring onto the "
        "table), or the pot is dropped (it falls 5 cm within 0.5 s with no finger pad within 2 cm of its body or "
        "handle)."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed (the tea is not simulated). For a real trial, empty the cup and refill the "
                    "pot, seat the lid and stand the pot on the trivet with its handle toward the robot and its spout "
                    "toward the cup."),
    "fidelity": (
        "Rigid boxes with PhysX contacts and friction; the lid is a loose part with no locking tab. Liquid is"
        " not simulated: the tea is a visual block and the pot's mass does not change. Pouring is the spout's"
        " pose: the pot tilted 45 degrees or more with the spout tip within 2.5 cm of the cup's axis and 1-8 "
        "cm above its rim; 0.5 s that tilted elsewhere counts as pouring on the table. A lid leaving its seat"
        " while the pot is tilted 10 degrees or more scores 0. The cup and trivet are fixed."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
