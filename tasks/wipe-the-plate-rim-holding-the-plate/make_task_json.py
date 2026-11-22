"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # masses and friction values are passed inline to the parts, not as named constants
assert (t.SCORE_OVER, t.PRESS, t.OVER, t.RUB_SPEED, t.RUB_STEPS, t.STILL_MOVE, t.MAX_MOVE, t.MAX_YAW, t.LIFT_MAX,
        t.FOOD_NEAR, t.DT, t.FOOD_S, t.FOOD_C[:2], round(t.FOOD_C[2], 4), len(t.SMUDGES)) == (
        "end", 0.8, 0.020, 0.010, 10, 0.010, 0.030, 5.0, 0.020, 0.010, 0.05, (0.07, 0.07, 0.04), (0.0, 0.0), 0.03, 3), \
    "task.py thresholds changed: update the scoring text"
assert (t.PASS_XY, t.PASS, t.PLATE_XY, t.PLATE, t.PLATE_T, t.LIP_W, t.LIP_H, t.BAND, round(t.SMUDGE_R, 4),
        {k: (round(x, 4), round(y, 4)) for k, (x, y) in t.SMUDGES.items()}, t.TOWEL, t.STAND_XY, t.STAND_H) == (
        (0.24, 0.0), (0.30, 0.42, 0.040), (0.24, 0.0), 0.236, 0.010, 0.016, 0.012, 0.037, 0.0835,
        {"far": (0.059, -0.059), "right": (0.0, -0.0835), "near": (-0.0835, 0.0)}, (0.030, 0.030, 0.025), (0.15, -0.24),
        0.030) and all(s in src for s in ("mass=0.45", "mass=0.03", "friction=(0.02, 0.02)", "friction=(0.10, 0.10)",
                                          "friction=(0.8, 0.7)", "(0.030, 0.030, 0.001)", "(0.05, 0.05, STAND_H)")), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Wipe the plate rim, holding the plate",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "hold and wipe", "kitchen", "plating", "wiping", "contact force", "keep-out zone",
                 "unstated technique"],
    "instruction": t.INSTRUCTION,
    "context": ("Plating guides tell cooks to clean the rim before a plate goes out: Culinary Arts Switzerland says to "
                "wipe the rim with a damp cloth before serving, Chefitt to wipe the plate edge before it leaves the "
                "kitchen, and the caterer Personal Touch Dining to wipe every rim before sending a plate out (there one "
                "cook polishes, wiping off drips). The arranged food in the middle must stay "
                "untouched while the rim is wiped. Here the plate stands on a wet stainless-steel pass, and wet "
                "porcelain on wet steel slides when a towel is dragged across the rim: with the same wiping and nobody "
                "holding the plate, the recorded run scored 0 because the plate slid away under the towel. So one arm "
                "has to hold the plate still by its rim while the other wipes. The instruction says only to clean the "
                "rim with the towel pad; it does not say to hold the plate or to keep off the food. Done by: cooks and "
                "chefs at the pass of a restaurant kitchen, and catering cooks, before each plate is sent out."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Pass: a wet stainless-steel shelf, 30 x 42 cm and 4 cm tall, centred at x 24, y 0 (fixed). "
        "Plate: 0.45 kg porcelain, square, 23.6 x 23.6 cm, standing on the pass centred at x 24, y 0: a 1 cm thick "
        "base with a raised lip round its edge, 1.6 cm wide and 1.2 cm tall, and inside the lip a flat rim band "
        "3.7 cm wide. Three sauce smudges, 3 x 3 cm (visual only), on the rim band 8.35 cm from the plate's centre: "
        "near side (x 15.65, y 0), right side (x 24, y -8.35) and far-right (x 29.9, y -5.9). Food: meat, mash and "
        "greens in the middle of the plate (visual only); the scorer's food box is 7 x 7 cm, centred on the plate, "
        "and 4 cm tall from the top of the base. Towel pad: a damp folded towel, 0.03 kg, 3 x 3 x 2.5 cm, lying on a "
        "steel stand 5 x 5 cm and 3 cm tall at x 15, y -24."),
    "scoring": (
        "Every step, from poses and the towel-plate contact force; the score at the end counts. A step rubs a smudge "
        "when the towel pad's centre is within 2 cm (horizontally, in the plate's frame) of the smudge's centre, the "
        "towel presses on the plate with at least 0.8 N and it moves at least 1 cm/s relative to the plate. A smudge "
        "is clean after 10 rubbing steps (0.5 s at the 0.05 s step) in total. 1.0: all three smudges clean, with the "
        "plate kept within 1 cm and 5 degrees of where it was when the towel first touched it (the grip on the lip "
        "may nudge it before then). Otherwise one third per clean smudge (0.33, 0.67), capped at 0.5 if the plate "
        "slid more than 1 cm or turned more than 5 degrees once the towel touched it. 0 from the moment the plate is "
        "more than 3 cm from its start, is lifted more than 2 cm, or anything comes near the food: a finger pad of "
        "either arm within 1 cm of the food box, or the towel pad overlapping it."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Put the plate back in the middle of the pass and the towel pad back on its "
                    "stand. In a real trial, put three fresh sauce smudges on the rim and wet the pass and the towel "
                    "again."),
    "fidelity": (
        "Rigid boxes (a square plate, a rigid towel pad); the towel-plate contact force is read each step. "
        "Liquids are not simulated: the wet pass and damp towel are friction settings (0.02 steel, 0.10 plate"
        " base, 0.8 pad), so a pressed, dragged towel pushes an unheld plate. The smudges are visual and "
        "stay, so cleaning counts the steps in which the towel rubs each smudge with at least 0.8 N while "
        "moving relative to the plate. The food has no collision: a keep-out box over the plate's middle "
        "stands in."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
