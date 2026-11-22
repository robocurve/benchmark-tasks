"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
SRC = open("task.py").read()  # for the few values written inline in task.py rather than as named constants
assert (t.SCORE_OVER, t.TOUCH, t.PRESS, t.PRESS_SHARE, t.STROKE_MIN, t.PUSH_TOL, t.SAW, t.GAP_STEPS, t.STILL, t.PASSES,
        t.HAND_KEEP, t.SH_TIP, t.ALIGN, t.DOCK_UPRIGHT, t.DOCK_HEADING, t.DOCK_HEEL_RANGE, t.DOCK_LIFT, t.KNIFE_NEAR,
        "kk.FallWatch(fall=0.03, steps=10)" in SRC, "-0.010 <= s <= EDGE_LEN" in SRC,
        "*_HANDLE_BOX, knife, 0.010)" in SRC) == (
        "end", 0.5, 1.5, 0.80, 0.120, 0.010, 0.020, 3, 0.020, 3, 0.015, 20.0, 0.94, 30.0, 15.0, 0.080, 0.006, 0.020,
        True, True, True), \
    "task.py thresholds changed: update the scoring text"
assert (t.SCENE, t.KNIFE_POS[:2], t.KNIFE_MASS, t.BLADE_T, t.BLADE_H, t.HEEL_L, t.TIP_RUN, t.TIP_RISE,
        round(t.EDGE_LEN, 3), t.HANDLE_L, t.HANDLE_T, t.HANDLE_H, t.BOLSTER[1], t.DOCK_H, t.DOCK_L, t.DOCK_W,
        t.DOCK_SLOT_W, t.DOCK_SLOT_D, t.DOCK_TOP_W, tuple(round(v, 4) for v in t.DOCK_XY), t.BOARD_XY, t.BOARD,
        t.SH_XY, t.SH_MASS, t.FOOT_H, t.BASE, round(t.BASE_TOP, 4), t.SH_HANDLE_C[:2], t.SH_HANDLE, t.SLOT_L,
        t.SLOT_D, t.SLOT_W, t.SLOT_TOP_W, t.HOUSING_W, t.SLOTS, t.SLOT_X, t.BLADE_FRICTION,
        "friction=(0.05, 0.05)" in SRC, "friction=(0.10, 0.10)" in SRC, "friction=(0.6, 0.5)" in SRC) == (
        "station", (0.30, -0.24), 0.20, 0.002, 0.045, 0.150, 0.050, 0.030, 0.208, 0.105, 0.016, 0.024, 0.020, 0.050,
        0.230, 0.050, 0.004, 0.012, 0.016, (0.41, -0.24), (0.36, 0.0), (0.24, 0.34, 0.015), (0.40, 0.0), 0.10, 0.004,
        (0.060, 0.220, 0.020), 0.024, (0.0, 0.065), (0.035, 0.080, 0.035), 0.050, 0.018, 0.004, 0.014, 0.034,
        {"coarse": -0.035, "fine": -0.075}, 0.0, (0.50, 0.45), True, True, True), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Sharpen a knife in a pull-through sharpener: hold it, coarse then fine, pull only",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "kitchen", "knife sharpening", "pull-through sharpener", "hold and work",
                 "unstated manufacturer's instructions", "contact force"],
    "instruction": t.INSTRUCTION,
    "context": (
        "Pull-through sharpener instructions describe a fixed procedure. Etch Couteaux: hold the sharpener's "
        "handle firmly with one hand so it cannot slip, pull the knife toward you heel to tip in the coarse "
        "slot, then the fine slot, never back and forth. The item 191H manual: blade vertical; the No. 1 "
        "coarse slot sets the edge, the No. 2 fine slot hones it; always pull heel to tip. Smith's directions"
        " add moderate downward pressure; the KitchenKnives.com guide gives 3-5 passes per slot. The "
        "instruction only says to sharpen the knife and put it back, so the task checks the procedure "
        "unprompted. Two arms by friction: the 100 g sharpener stands on slippery feet on a wet board while "
        "its slot grips the blade; in the recorded one-arm variant it slid 15.32 cm with the knife and no "
        "pass counted (score 0); held, it moved 1.09-1.25 cm per pass. Done by: cooks and home cooks."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Knife dock: dark wood, fixed, 23 cm long (along x) and 5 cm wide, centred at x 41, y -24 (it runs "
        "from x 29.5 to 52.5), with a V-groove along its top whose floor is 5 cm above the table (the groove is "
        "1.2 cm deep, 0.4 cm wide at its floor and 1.6 cm at its top). Chef's knife: 0.20 kg, standing edge-down in "
        "the groove with its point away from the robot and its handle toward the robot; the heel of the edge rests "
        "on the groove's floor at x 30, y -24. Steel blade 2 mm thick and 4.5 cm high; the edge is 20.8 cm long, "
        "straight for 15 cm and then rising 3 cm over the last 5 cm to the point; a steel bolster 2 cm long; a black "
        "hollow handle 10.5 cm long, 1.6 cm thick and 2.4 cm high (32.5 cm long in all, from x 17.5 to 50). Cutting "
        "board: white plastic, wet, fixed, 24 x 34 x 1.5 cm, centred at x 36, y 0. Pull-through sharpener: 0.10 kg, "
        "a blue body 6 x 22 x 2 cm (long side along y) on four black feet 0.4 cm high, centred at x 40, y 0 on the "
        "board; a handle 3.5 x 8 x 3.5 cm on top of its left end (centred at y 6.5); two slots running front to back "
        "(along x), each 5 cm long and centred at x 40: the coarse carbide slot (dark) at y -3.5 and the fine "
        "ceramic slot (white) at y -7.5. Each slot is a V-groove 1.8 cm deep, 0.4 cm wide at its floor and 1.4 cm "
        "at its top, in a housing 3.4 cm wide; the slot floor (the abrasive) is 2.4 cm above the board, 3.9 cm "
        "above the table."),
    "scoring": (
        "Every step, from poses and the knife-on-sharpener contact force; the score at the end counts. The blade is "
        "in a slot when it lies along the slot and upright (each within about 20 degrees, cosine 0.94), the middle "
        "of the slot meets the edge between 1 cm behind the heel and the point, and there the edge is inside the "
        "slot (within its 1.4 cm wide top opening and no higher than its top). A pass is one contact: it starts "
        "when the blade is in a slot with a contact force of at least 0.5 N and ends after more than 3 steps off "
        "the abrasive or when the blade moves to the other slot. A pass is valid when the point where the slot's "
        "middle meets the edge travels at least 12 cm from heel toward tip over the steps pressed at 1.5 N or more, "
        "at least 80% of the pass's touching steps are pressed, it has no more than 1 cm of backward (tip-to-heel) "
        "travel, and the sharpener moves no more than 2 cm horizontally during it. Valid coarse passes count; "
        "valid fine passes count only after three valid coarse passes (earlier ones are recorded and do not "
        "count). Score: 1/6 per counted coarse pass and per counted fine pass, up to three of each, capped at 0.75 "
        "while the knife is not back in its dock. The knife is in its dock when its heel is within the first 8 cm "
        "of the groove from the dock's near end, within 0.8 cm of the groove's centre line and no more than 6 mm "
        "above its floor, the blade within 30 degrees of upright, its point within 15 degrees of straight away "
        "from the robot, and no finger pad within 1 cm of its handle. 1.0 (success): three counted passes in each "
        "slot and the knife in its dock. 0 from the moment of sawing (more than 2 cm of tip-to-heel travel in one "
        "contact), any finger pad of the left (holding) hand within 1.5 cm of a slot housing, the sharpener tipped "
        "more than 20 degrees, or the knife dropped (its heel falls 3 cm within 10 steps while no finger pad is "
        "within 2 cm of its handle)."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Stand the knife back in its dock, edge down with its point away from the "
                    "robot, and put the sharpener back in the middle of the board with its handle on the left. For "
                    "real trials, wet the board again; each real pass removes a little steel from the blade."),
    "fidelity": (
        "Rigid boxes (knife; sharpener feet, body, handle and slots; fixed board and dock). The knife-on-"
        "sharpener contact force is read each step. Abrasion is not simulated (the strips are visual, the "
        "edge does not change), so passes are counted as the instructions describe: pressed, heel to tip over"
        " at least 12 cm, never back, coarse before fine, sharpener held still. The wet board is friction "
        "0.05 under feet of 0.10. A 1.5 cm finger keep-out from the slots stands in for the blade's path."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
