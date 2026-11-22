"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # values written inline in task.py (not named constants) are checked in its source
assert (t.SCORE_OVER, t.WIPE_FORCE, t.WIPE_SPEED, t.BINS, t.FULL_COVER, t.PART_COVER, t.EDGE_KEEP, t.DROP_FALL,
        t.DROP_STEPS, t.KNIFE_NEAR, t.DT, t.HOLD_DIST, t._HANDLE_S, t._EDGE_S) == (
        "end", 0.3, 0.005, 16, 0.90, 0.50, 0.008, 0.05, 10, 0.020, 0.05, 0.008, (0.022, 0.100, 0.016),
        (0.002, 0.160, 0.002)) and round(t.EDGE_X, 4) == 0.025 and all(s in src for s in (
            'abs(tb["gap"]) <= 0.006', "-TOWEL[1] / 2 <= s <= BLADE_L + TOWEL[1] / 2", 'tb["x_hi"] > EDGE_X',
            'heel_to_tip = _S["to_tip"] > _S["to_heel"]', "res = (1.0, True)", "res = (0.5, False)",
            "res = (0.25, False)")), \
    "task.py thresholds changed: update the scoring text"
assert (t.SCENE, t.SHEET_XY, t.SHEET, t.BLADE_L, t.BLADE_W, t.BLADE_T, t.SPINE_X, t.HANDLE_L, t.HANDLE_W, t.HANDLE_T,
        t.REST_XY, t.REST, t.REST_H, t.KNIFE_POS[:2], t.TOWEL, t.STAND_XY, t.STAND_H, t.TOWEL_POS[:2]) == (
        "station", (0.30, 0.0), (0.26, 0.40, 0.003), 0.160, 0.035, 0.002, -0.010, 0.100, 0.022, 0.016, (0.30, 0.012),
        (0.040, 0.025), 0.060, (0.30, 0.0), (0.025, 0.030, 0.040), (0.18, -0.24), 0.030, (0.18, -0.24)) \
    and round(t.KNIFE_POS[2] - t.TOP, 4) == 0.0715 and all(s in src for s in (
        "mass=0.15", "mass=0.03", "friction=(0.08, 0.08)", "for i in range(4)", "-0.020 - 0.035 * i",
        "(0.010, 0.012, 0.0006)", "collide=False")), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Wipe the knife heel to tip, edge away",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "hold and wipe", "kitchen", "knife care", "unstated safety rule", "contact force"],
    "instruction": t.INSTRUCTION,
    "context": (
        "Knife-care guidance says to hold the knife by its handle with the edge facing away and wipe the flat"
        " of the blade from heel to tip, never across the edge (Cangshan Cutlery, How to Clean Your Knives); "
        "to fold a cloth over the spine and drag it heel to tip (KnifeAid); and to pinch the spine in a towel"
        " and wipe toward the tip (Kitchengearly). The instruction only says to wipe the knife clean, so the "
        "task checks this practice unprompted. Bimanual by rule: passes count only while a gripper holds the "
        "knife by its handle, so one arm holds and the other wipes. This is the guidance's rule, not physics:"
        " with an earlier scorer the unheld knife stayed put and one arm wiped 62.5% of it (0.5); with the "
        "hold rule the recorded one-arm run scores 0. Not tried: holding the knife and rubbing it on the pad "
        "left on its stand. Done by: cooks and kitchen staff, and home cooks."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Draining sheet: steel, wet (low friction), 26 x 40 cm, 3 mm thick, fixed, centred at x 30, y 0. "
        "Knife rest: a fixed block 4 x 2.5 cm and 6 cm tall, standing on the sheet at x 30, y 1.2. Chef's knife: "
        "0.15 kg, lying level along y with its cutting edge facing away from the robot (+x) and its handle's axis "
        "at x 30, 7.15 cm above the table. Its dark handle, 10 cm long, 2.2 cm wide and 1.6 cm thick, runs from the "
        "heel at y 0 to its butt end at y 10, and its first 2.5 cm rests on the knife rest. Its steel blade, 16 cm "
        "long, 3.5 cm wide and 2 mm thick, runs from the heel at y 0 to the tip at y -16, level with the handle's "
        "middle and clear of the sheet, with the spine at x 29 and the cutting edge at x 32.5. Four food smears, 1 "
        "cm across and 1.2 cm along the blade, lie on its top face, centred 2, 5.5, 9 and 12.5 cm from the heel "
        "(visual only). Towel pad: a folded towel, 0.03 kg, 2.5 x 3 cm and 4 cm tall, standing on a fixed stand 5 "
        "x 5 cm and 3 cm tall at x 18, y -24."),
    "scoring": (
        "Every step, from poses and the towel-on-knife contact force; the score at the end counts. A wiping step: "
        "the towel presses on the knife with at least 0.3 N, its underside within 6 mm of the blade's top face and "
        "its centre between 1.5 cm before the heel and 1.5 cm past the tip, it moves along the blade at least 0.5 "
        "cm/s, and a gripper holds the knife by its handle (a pad point of each finger within 8 mm of the 10 x 2.2 x "
        "1.6 cm handle, the handle between them). Each wiping step marks the 1 cm sections of the blade (16, heel "
        "to tip) under the towel's 3 cm length and adds its travel to the heel-to-tip or the tip-to-heel total. "
        "Pressed steps on a knife nobody holds by the handle do not count. 1.0: at least 90% of the blade wiped (15 "
        "of the 16 sections) and more travel toward the tip than toward the heel. 0.5: at least 50% wiped (8 "
        "sections), in either direction. 0.25: some of the blade wiped, but less than half. 0 if nothing is "
        "wiped, and 0 from the moment any finger pad comes within 8 mm of the cutting edge, the towel reaches over "
        "the cutting edge while pressed on the blade (its far side more than 2.5 cm in front of the handle's axis, "
        "held or not), or the knife is dropped (it falls 5 cm within 0.5 s with no finger pad within 2 cm of it). "
        "success() is true on the 1.0 "
        "condition, and yam_arena ends the episode once it has held for 1 s."),
    "run_limit": "1,200 steps (1 minute at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Lay the knife back with the first 2.5 cm of its handle on the rest, the "
                    "blade off the rest to the right and its edge away from the robot, and stand the towel pad back "
                    "on its stand. For real-world trials, renew the smear on the blade."),
    "fidelity": (
        "Rigid boxes with contact forces: a 16 x 3.5 cm blade, 2 mm thick, square tip, no bevel, on a 10 cm "
        "handle; the towel pad is a rigid 2.5 x 3 x 4 cm box. Food, cleaning, water and cutting are not "
        "simulated: the smears are visual, so cleaning counts the 1 cm blade sections the towel passes over "
        "pressed (at least 0.3 N) and moving along it, with the knife held by its handle. The sheet's wetness"
        " is low friction (0.08). An 8 mm finger keep-out from the edge stands in for cutting."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
