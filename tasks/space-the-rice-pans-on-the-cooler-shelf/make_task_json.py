"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
# the stacking share (over the other pan's outline) is a literal in pan_state(), not a named constant
_STACK_SHARE_03 = "(PAN_W, PAN_L)).mean() > 0.3)" in open("task.py").read()
assert (t.SCORE_OVER, t.REST_TOL, t.REST_TILT, t.ON_SHARE, t.GAP, t.TOUCH_GAP, t.SPILL_TILT, t.STACK_Z, t.PAN_NEAR,
        t.RELEASE_DIST, t.DROP_FALL, t.DROP_STEPS, t.STILL_MOVE, t.STILL_STEPS, round(t._EXTENT, 4),
        tuple(round(v, 4) for v in t._PAN_S), (len(set(t._U[:, 0])), len(set(t._U[:, 1]))), _STACK_SHARE_03,
        getattr(t, "SUCCESS_HOLD_S", 1.0)) == (
        "end", 0.006, 5.0, 0.90, 0.050, 0.010, 30.0, 0.030, 0.020, 0.008, 0.03, 10, 0.002, 20, 0.162,
        (0.175, 0.324, 0.065), (5, 9), True, 1.0), \
    "task.py thresholds changed: update the scoring text"
assert (t.SHELF_XY, t.SHELF, t.LIP_H, t.STOP_H, t.RAIL_T, t.HOLD_XY, t.HOLD, t.PAN_W, t.PAN_L, t.PAN_H, t.PAN_T,
        t.FLANGE_W, t.FLANGE_T, tuple(round(v, 4) for v in t.PAN_OUT), round(t.PAN_MASS * 9.81 * t.PAN_OUT[1] / 2, 1),
        t.PAN_MASS, t.PAN_POS["pan_a"][:2], t.PAN_POS["pan_b"][:2],
        "(0.0, 0.0, PAN_T + 0.015), (_iw - 0.002, PAN_L - 2 * PAN_T - 0.002, 0.030)" in open("task.py").read()) == (
        (0.38, 0.0), (0.22, 0.74, 0.060), 0.015, 0.030, 0.006, (0.17, 0.0), (0.18, 0.36, 0.060), 0.145, 0.294, 0.065,
        0.006, 0.015, 0.005, (0.175, 0.324), 2.4, 1.50, (0.38, 0.0), (0.17, 0.0), True), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Space the rice pans on the cooler shelf",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "two-handed lift", "kitchen", "food safety", "cooling", "hotel pans",
                 "unstated safety rule", "make room first"],
    "instruction": t.INSTRUCTION,
    "context": (
        "FDA Food Code 3-501.15(B): food containers being cooled are arranged to give the most heat transfer "
        "through their walls (FDA's guide for food employees on cooling; adopted in Minnesota Rules "
        "4626.0390). A food-safety guide for cafes (Mays) puts it as arranging containers so air can move "
        "around them, and names stacking hot pans tight together as the classic mistake. The Food Code gives "
        "no distance; this task asks for at least 5 cm of clear space and no stacking. The instruction does "
        "not state the rule, and with the first pan in the middle of the shelf there is no room for the "
        "second, so the first must move to one end first. Two arms: each 1.5 kg pan has no handles and is "
        "gripped by its rim; held by one end its weight acts about 16 cm from the grip (2.4 N m), and in "
        "the recorded one-arm variant the pan tipped 14 degrees, slipped out of the pinch and came to rest off the shelf. Done by: cooks and food-prep staff."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Cooler shelf (fixed): grey, 22 cm deep (x) by 74 cm long (y) and 6 cm high, centred at x 38, y 0 "
        "(from x 27 to 49 and y -37 to 37), with a lip 1.5 cm high along its front edge (the side facing the arms), "
        "a back stop 3 cm high along its back edge (both 0.6 cm thick) and rack lines marked across its top (visual "
        "only). Hot-holding block (fixed): dark grey, 18 cm (x) by 36 cm (y) and 6 cm high, as high as the shelf, "
        "centred at x 17, y 0; its back edge is 1 cm in front of the shelf's front edge. Two hotel pans: steel "
        "third-size steam-table pans with no handles, 14.5 cm (x) by 29.4 cm (y) and 6.5 cm tall, 0.6 cm walls and "
        "floor, with a flat flange 1.5 cm wide and 0.5 cm thick round the rim, its top flush with the rim (17.5 x "
        "32.4 cm over the flange); 1.5 kg each with the rice, long side along y, filled 3 cm deep with hot cooked "
        "rice (visual; its mass is part of the pan's). The first pan stands in the middle of the shelf, centred at x 38, y 0; the second stands "
        "centred on the hot-holding block, at x 17, y 0."),
    "scoring": (
        "Every step, from poses; the score at the end counts. A pan rests on the shelf when at least 90% of a grid "
        "of 45 points (5 across by 9 along) over its underside lies inside the shelf's 22 x 74 cm outline, its "
        "underside is within 6 mm of the shelf's top and it is tilted 5 degrees or less. It is released when no "
        "finger pad is within 8 mm of its box out to the flange (17.5 x 32.4 x 6.5 cm). With both pans resting and "
        "released, the clear gap between them is measured along the shelf, flange edge to flange edge (the "
        "distance between their centres along y less 32.4 cm). 1.0 (success): the gap is at least 5 cm. 0.5: 1-5 cm. 0.25: "
        "under 1 cm (touching). 0 if either pan is not resting on the shelf or not released, a pan is stacked on "
        "the other (more than 30% of its underside grid over the other pan's 14.5 x 29.4 cm outline and its underside "
        "more than 3 cm above the other's), or the pans are not still (each moving less than 2 mm over 20 steps, "
        "1 s at 20 Hz); and 0 from the moment either pan tips past 30 degrees (the rice spills) or is dropped (it "
        "falls 3 cm within 10 steps, 0.5 s, with no finger pad within 2 cm of it). The episode ends once success() "
        "has held for 1 s (yam_arena's default success_hold_s)."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Set the first pan back in the middle of the cooler shelf and the second "
                    "back in the middle of the hot-holding block, both with their long side along the shelf."),
    "fidelity": (
        "Rigid boxes (pans with a rim flange; a shelf with lip and back stop; a hot-holding block) with contacts and "
        "friction; each pan is one 1.5 kg body. Heat, steam, cooling and the rice are not simulated (the rice"
        " is a visual block). The scorer stands in for heat transfer with poses: both pans level on the "
        "shelf, at least 5 cm of clear space, neither stacked; a tilt past 30 degrees stands in for rice "
        "sliding out. The pans have no lids, so the covering part of 3-501.15(B) is not tested."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
