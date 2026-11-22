"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import math
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
SRC = open("task.py").read()  # for the two values written inline in task.py (plate mass, placemat size)
assert (t.SCORE_OVER, t.LIFTED, t.DOWN, t.SYNC_STEPS, t.SLOW_STEPS, t.COVER_TOL, t.FACE_TOL, t.TURN_ON_TABLE,
        t.CARRY_TILT, t.MAX_TILT, t.FOOD_NEAR, t.PLATE_NEAR, t.RELEASE_DIST, t.DROP_FALL, t.DROP_STEPS, t.STILL_MOVE,
        t.STILL_STEPS, t.FOOD_C, t.FOOD_S) == (
        "end", 0.010, 0.003, 10, 60, 0.030, 25.0, 10.0, 15.0, 30.0, 0.008, 0.020, 0.008, 0.03, 10, 0.002, 20,
        (0.0, -0.0035, t.PLATE_T + 0.02), (0.12, 0.133, 0.04)), \
    "task.py thresholds changed: update the scoring text"
assert (t.PASS_XY, t.PASS, t.PLATE, t.PLATE_T, t.LIP_W, t.LIP_H, t.START_XY, t.START_YAW, tuple(t.PROTEIN_DIR),
        t.COVER_XY, "mass=0.30," in SRC, "(0.26, 0.26, 0.001)" in SRC) == (
        (0.14, 0.0), (0.20, 0.50, 0.050), 0.200, 0.010, 0.012, 0.015, {"left": (0.14, 0.13), "right": (0.14, -0.13)},
        {"left": 0.0, "right": math.pi}, (0.0, -1.0, 0.0), {"left": (0.36, 0.15), "right": (0.36, -0.15)}, True,
        True), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Serve two plates together, each steak toward its guest",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "plated service", "restaurant service", "synchronised placing", "object orientation",
                 "unstated service rule"],
    "instruction": t.INSTRUCTION,
    "context": (
        "In plated service a table's plates are set down at the same moment, each with the protein at 6 "
        "o'clock, toward the guest. Nicolet College's chapter on American service puts the protein at 6 "
        "o'clock and serves all guests simultaneously; HappyChef's fine-dining guide calls for synchronised "
        "serving, one server per guest; Instawork's plated-service basics also put the protein at 6 o'clock "
        "and say not to rotate plates in front of guests. Here two plated mains wait on the pass with their "
        "steaks pointing to the middle, so each plate must turn a quarter turn before it reaches the table. "
        "One gripper carries one plate, so setting both down together takes both arms, one per server. The "
        "instruction states neither rule, so the task checks whether the robot serves as trained staff do, "
        "unprompted. Done by: waiters and banquet and catering servers."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Pass: a fixed grey shelf 20 cm deep, 50 cm wide and 5 cm tall, centred at x 14, y 0. On it, two "
        "plated main courses: white porcelain plates, 0.30 kg each, square, 20 x 20 cm with a 1 cm base and a raised "
        "rim lip 1.2 cm wide and 1.5 cm high round the edge (2.5 cm tall in all), centred at x 14, y 13 (left plate) "
        "and x 14, y -13 (right plate), resting on the pass 5 cm above the table. On each plate a steak sits on one "
        "half and mashed potato and greens on the other (visual only); both steaks point toward the middle (the left "
        "plate's toward -y, the right plate's toward +y). Place settings: two dark brown placemats, 26 x 26 cm, flat "
        "on the table (visual only), centred at x 36, y 15 (left) and x 36, y -15 (right). The two guests are seated "
        "beyond them, on the far side of the table (+x); they are not modelled."),
    "scoring": (
        "Every step, from poses; the score at the end counts. A plate has left the pass once its base is more than "
        "1 cm above its starting height; it is set down at the first step after that when its underside is within 3 "
        "mm of the table, and the gap is the number of steps (20 Hz) between the two plates' set-down steps. A "
        "plate is turned on the table if, while on the table after its set-down, its yaw differs from its yaw at "
        "set-down by more than 10 degrees (turning it on the pass or during the carry, before its first set-down, "
        "does not count). The steak's angle is the horizontal angle between the direction from the plate's centre "
        "to its steak and the direction to the guest (+x). The carry tilt is the largest tilt of either plate "
        "between leaving the pass and its set-down. Placed: both plates on the table (underside within 3 mm of it), "
        "each plate's centre within 3 cm (horizontally) of the centre of the place setting on its own side, no "
        "finger pad within 8 mm of either plate, and both plates moving less than 2 mm over 1 s. 1.0: placed, set "
        "down at most 10 steps (0.5 s) apart, both steaks within 25 degrees of toward their guests, neither plate "
        "turned on the table, and a carry tilt of at most 15 degrees. 0.5: the same, but set down more than 10 and "
        "at most 60 steps (0.5 to 3 s) apart. 0.25: placed and set down at most 60 steps (3 s) apart, but a steak "
        "more than 25 degrees off, a plate turned on the table, or a carry tilt of more than 15 (up to 30) degrees. "
        "0 otherwise: a plate not set down or not placed, or the two set-downs more than 60 steps (3 s) apart; and "
        "0 from the moment a finger pad comes within 8 mm of the food (a zone 12 x 13.3 x 4 cm over the plate's "
        "centre, shifted 3.5 mm toward the steak, from the top of the base up), a plate is carried tilted more than "
        "30 degrees, or a plate is dropped (it falls 3 cm within 0.5 s with no finger pad within 2 cm of it)."),
    "run_limit": "1,200 steps (1 minute at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Put both plates back on the pass, centred at y 13 (left) and y -13 (right), "
                    "each with its steak toward the middle."),
    "fidelity": (
        "Rigid boxes with contacts and friction; each plate is a box with a rim lip, held by pinching the "
        "lip. The food is visual (no mass or collision), so carry-tilt limits (15 and 30 degrees) and an 8 mm"
        " finger keep-out stand in for keeping it in place. The placemats are visual; placement is the "
        "plate's centre within 3 cm. The guests are not modelled: each faces +x. 'At the same moment' is "
        "judged from each plate's underside coming within 3 mm of the table, timed to the 20 Hz step."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
