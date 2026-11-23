"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift). An existing
task.json's media ID is kept."""
import importlib.util
import json
import os
import sys

import numpy as np

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # masses and friction values are passed inline to the parts, not as named constants
corner = max(float(np.hypot(abs(c[0]) + s[0] / 2, abs(c[1]) + s[1] / 2)) for c, s in t.FOOD_BOXES)
assert (t.SCORE_OVER, t.PRESS, t.RUB_SPEED, t.GRID, t.PAD_GAP, t.WIPE_TRAVEL, t.CLEAN_FRAC, t.STILL_MOVE, t.MAX_MOVE,
        t.MAX_YAW, t.LIFT_MAX, t.FOOD_TOUCH, t.FOOD_MARGIN, t.DT, len(t.SMUDGES),
        sorted({len(p) for p in t.SMUDGE_PTS.values()}), t.SAUCE_D, t.OFF_GAP, t.OFF_HOLD) == (
        "end", 1.2, 0.010, 0.0025, 0.002, 0.010, 0.95, 0.010, 0.030, 5.0, 0.020, 0.02, 0.010, 0.05, 3, [113], 0.030,
        0.020, 1.0), \
    "task.py thresholds changed: update the scoring text"
assert (len(t.SAUCE_TILES) == 113 and t.SAUCE_TILE == t.GRID
        and len(t.PLATE_GEOMS) == t.N_SOLID + len(t.SMUDGES) * len(t.SAUCE_TILES)), "sauce tiles changed: update the text"
assert (set(t.ARM_LINKS) == {"forearm link4", "wrist link5", "gripper body", "finger tip_left", "finger tip_right"}
        and len(t.FOOD_WATCH) == 11 and "towel" in t.FOOD_WATCH), "watched links changed: update the scoring text"
assert (t.PASS_XY, t.PASS, t.PLATE_XY, t.PLATE, t.PLATE_T, t.LIP_W, t.LIP_H, t.BAND, round(t.SMUDGE_R, 4),
        {k: (round(x, 4), round(y, 4)) for k, (x, y) in t.SMUDGES.items()}, t.TOWEL, t.STAND_XY, t.STAND_H,
        t.FOOD_D, t.FOOD_H, t.FOOD_MASS, len(t.FOOD_GEOMS), round(corner, 4)) == (
        (0.24, 0.0), (0.30, 0.42, 0.040), (0.24, 0.0), 0.236, 0.010, 0.016, 0.012, 0.037, 0.0835,
        {"far": (0.059, -0.059), "right": (0.0, -0.0835), "near": (-0.0835, 0.0)}, (0.030, 0.030, 0.025), (0.15, -0.24),
        0.030, 0.095, 0.030, 0.057, 15, 0.0507) and all(
        s in src for s in ("mass=0.45", "mass=0.03", "mass=FOOD_MASS", "friction=(0.02, 0.02)", "friction=(0.10, 0.10)",
                           "friction=(0.8, 0.7)", "(SAUCE_TILE, SAUCE_TILE, 0.001)", "(0.05, 0.05, STAND_H)", "k=15")), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Wipe the plate rim, holding the plate",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "hold and wipe", "kitchen", "plating", "wiping", "contact force", "area coverage",
                 "keep-out zone", "unstated technique"],
    "instruction": t.INSTRUCTION,
    "context": ("Restaurant cooks (O*NET 35-2014.00, about 1.4 million US workers) list among their core tasks "
                "'Portion, arrange, and garnish food, and serve food to waiters or patrons'; BLS OEWS (May 2022) counts "
                "988,060 of them in full-service restaurants, where plates go out to seated guests. Wiping the rim is "
                "the last step of plating: Culinary Arts Switzerland says to wipe it with a damp cloth before serving, "
                "Chefitt before the plate leaves the kitchen, and the caterer Personal Touch Dining every rim before a "
                "plate goes out. The food must stay untouched. On a wet stainless-steel pass, wet porcelain slides "
                "when a towel is dragged across the rim: with the same wiping and nobody holding the plate, the "
                "recorded run scored 0 because the plate slid away. So one arm holds the plate by its rim while the "
                "other presses the towel over each whole smudge, clear of the food. The instruction does not say to "
                "hold the plate or to keep off the food. Done by: restaurant and catering cooks at the pass."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Pass: a wet stainless-steel shelf, 30 x 42 cm and 4 cm tall, centred at x 24, y 0 (fixed). "
        "Plate: 0.45 kg porcelain, square, 23.6 x 23.6 cm, standing on the pass centred at x 24, y 0, where it is "
        "wiped, ready for pick-up (so lifting it more than 2 cm scores 0): a 1 cm thick "
        "base with a raised lip round its edge, 1.6 cm wide and 1.2 cm tall, and inside the lip a flat rim band "
        "3.7 cm wide (6.5-10.2 cm from the plate's centre). Three round smudges of tomato ketchup, 3 cm across "
        "(visual only: 2.5 mm tiles that vanish as they are wiped), centred 8.35 cm from the plate's centre: near "
        "side (x 15.65, y 0) and right side (x 24, y -8.35) on the rim band, and far-right (x 29.9, y -5.9) at the "
        "band's inner far-right corner, about 45% of it on the band and the rest on the plate's flat middle, as sauce that ran off the band; it is wiped whole, as a cook would. Food: the stand-in used in the real trials, one whole English muffin (Thomas' Original), 9.5 cm "
        f"across, 3 cm tall and 57 g, lying loose in the middle of the plate; it collides (fifteen box slats whose "
        f"corners reach {100 * corner:.2f} cm from its centre). Towel pad: a damp folded cotton towel, 0.03 kg, "
        "3 x 3 x 2.5 cm (in real trials a folded cotton handkerchief, its top taped), lying on a steel stand 5 x 5 cm "
        "and 3 cm tall at x 15, y -24."),
    "scoring": (
        "Simulation, every step from poses and contact forces; the score at the end counts. Each smudge is 113 "
        "sample points, a 2.5 mm grid over its whole 3 cm disc. A point is wiped once the towel pad's pressed "
        "underside (its 3 x 3 cm footprint, within 2 mm of the rim) has slid at least 1 cm over it in total, counting "
        "steps in which the towel presses on the plate with at least 1.2 N (a simulation proxy for firm contact; real trials score the sauce actually removed) and moves at least 1 cm/s relative to it. "
        "A smudge is clean when at least 95% of its points are wiped, so short strokes over part of a smudge do not "
        "clean it. 1.0: all three clean, with the plate kept within 1 cm and 5 degrees of where it was when the "
        "towel first touched it. Otherwise one third per clean smudge (0.33, 0.67), capped at 0.5 if the plate slid "
        "more than 1 cm or turned more than 5 degrees once the towel touched it. 0 from the moment the plate is "
        "more than 3 cm from its start or lifted more than 2 cm, or the food is touched: a contact force of 0.02 N "
        "or more between the food and the towel or the forearm, wrist, gripper body or either finger of either arm, "
        "or a finger pad or any point of the towel's surface within 1 cm of any part of the food (a simulation-only margin, because grazes under 0.02 N may not register; real trials score an actual touch). The trial ends (success) once all three are clean with the plate "
        "within its limits and the towel has then been off the plate (not touching it, 2 cm above the rim) for 1 s; "
        "the score counts clean smudges either way. Real trials: the "
        "same bands; the trial ends at 2,400 steps or once all three smudges look clean with the plate within its "
        "limits and the robot has held the towel off the plate (not touching it, 2 cm above the rim) for 1 s, when "
        "the after photo is taken. A smudge is clean when no red or "
        "pink mark of 2 mm or more shows within 1 cm of its stencil circle or where the towel smeared it, in the "
        "zoomed after photo with a ruler in view, any tint judged against the clean plate surface or a white card "
        "in the same photo; only sauce removed by the towel counts. Slide and turn come from "
        "tape marks on the plate's corners (two that stay visible) in the overhead video; a lift over 2 cm is judged "
        "in a side view. "
        "0 if any part of either arm or gripper, or the towel, touches the muffin or moves it relative to the "
        "plate."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": (
        "Nothing is consumed in simulation: put the plate back in the middle of the pass, the muffin in the middle "
        "of the plate and the towel pad on its stand. Real trials: wash and dry the plate; dry the pass, pour 5 mL "
        "of tap water (a measuring teaspoon) where the plate stands and slide the plate twice round a small circle "
        "onto its tape marks. Slide check: with nobody holding the plate, a 130 g weight set on the damp pad and drawn once along the near rim band by a thread at about 2 cm/s must slide it 1 cm or turn it 5 degrees "
        "(if not, add 5 mL; if it still holds, use a lighter plate within the tolerances, preferably with a glazed "
        "flat foot). Then lay the card template (template.svg, printed 1:1) inside the lip and spread 0.2 mL of Heinz Tomato Ketchup (room temperature, 20-25 C, bottle "
        "shaken) from a 1 mL oral syringe into each 3 cm hole, and lift the template. Place one whole, untoasted "
        "Thomas' Original English Muffin (room temperature, from a sealed pack within its date; a new one each "
        "session and after any touch) with its edge 5.45 cm from the lip's inner face on all four sides. Rinse the "
        "towel pad, dip its untaped bottom and blot or add water until it weighs 30 g; the grippers hold only its "
        "dry, taped top. Start 2 minutes after the sauce goes on. Keep vinyl tape on both grippers' finger pads."),
    "fidelity": (
        "Rigid boxes: square plate (0.45 kg; real porcelain is heavier), rigid towel pad, slatted muffin; contact "
        "forces read each step. Wet pass and damp towel are friction settings. Sauce never smears: each smudge is "
        "113 points, wiped once the pad, pressed with at least 1.2 N (the press the scripted wipe holds; firm "
        "contact for a rigid pad) and moving, has slid 1 cm over them. 95% is clean, so up to 5 unwiped points "
        "vanish from view; real trials allow no sauce mark of 2 mm or more."),
    "scores": scores,
}
if os.path.exists("task.json"):
    old = json.load(open("task.json"))
    if old.get("media"):
        task["media"] = old["media"]
for k, n in (("title", 120), ("context", 1000), ("fidelity", 500)):
    assert len(task[k]) <= n, f"{k} is {len(task[k])} chars (limit {n})"
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars;", {k: len(task[k]) for k in ("title", "context", "fidelity")})
