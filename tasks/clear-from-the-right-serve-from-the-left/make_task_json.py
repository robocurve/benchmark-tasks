"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # inline literals quoted below are checked against task.py's source text
assert (t.SCORE_OVER, t.LIFTED, t.CROSS_R, t.SIDE, t.COVER_TOL, t.FACE_TOL, t.LAP_X, t.OFF_COVER, t.FOOD_NEAR, t.NEAR,
        t.RELEASE_DIST, t.DROP_FALL, t.DROP_STEPS, t.STILL_MOVE, t.STILL_STEPS, t.FOOD_C, t.FOOD_S) == (
        "end", 0.010, 0.10, 0.050, 0.030, 25.0, 0.50, 0.20, 0.008, 0.020, 0.008, 0.03, 10, 0.002, 20,
        (0.0, 0.0, 0.010 + 0.02), (0.13, 0.13, 0.04)) and all(s in src for s in (
        "m_down = mn[0][2] - TOP <= 0.004 and m_off <= 0.08",
        "stacked = (float(np.linalg.norm(mn[0][:2] - st[0][:2])) <= 0.08 and 0.0 < mn[0][2] - st[0][2] <= LIP_H + 0.010",
        "and not kk.touched_by_any(_PC, _PS, mn, NEAR))",
        "if (m_down and not _S[\"starter_gone\"]) or stacked:\n        _S[\"early\"] = True",
        "abs(st[0][0] - TUB_XY[0]) <= TUB[0] / 2 - 0.05 and abs(st[0][1] - TUB_XY[1]) <= TUB[1] / 2 - 0.05",
        "and st[0][2] - TOP <= 0.012)",
        "served = m_down and m_off <= COVER_TOL",
        "clear_ok = exit_y is not None and exit_y >= SIDE",
        "serve_ok = entry_y is not None and entry_y <= -SIDE",
        "exit_y, entry_y = _side(_S[\"s_path\"], True), _side(_S[\"m_path\"], False)",
        "base = 1.0 if clear_ok and serve_ok else (0.5 if clear_ok or serve_ok else 0.25)",
        "if facing > FACE_TOL:\n            base = max(0.0, base - 0.25)",
        "if pose[0][0] > LAP_X:",
        "kk.FallWatch(DROP_FALL, DROP_STEPS)",
        "kk.StillWatch(STILL_MOVE, STILL_STEPS)")), \
    "task.py thresholds changed: update the scoring text"
assert (t.SCENE, t.PLATE, t.PLATE_T, t.LIP_W, t.LIP_H, t.COVER_XY, t.PASS_XY, t.PASS, t.TUB_XY, t.TUB,
        tuple(float(v) for v in t.PROTEIN_DIR)) == (
        "station", 0.200, 0.010, 0.012, 0.015, (0.36, 0.0), (0.14, 0.0), (0.22, 0.24, 0.040), (0.20, 0.30),
        (0.24, 0.24, 0.015), (1.0, 0.0, 0.0)) and all(s in src for s in (
        "STARTER_POS = (*COVER_XY, TOP + 0.0005)",
        "MAIN_POS = (*PASS_XY, PASS_TOP + 0.0005)",
        "STARTER_GEOMS, mass=0.25, color=PORCELAIN, friction=(0.6, 0.5)",
        "MAIN_GEOMS, mass=0.30, color=PORCELAIN, friction=(0.6, 0.5)",
        "(0.045, 0.0, PLATE_T + 0.0125), (0.050, 0.075, 0.025), color=MEAT, collide=False",
        "(-0.04, 0.03, PLATE_T + 0.015), (0.045, 0.05, 0.030), color=MASH, collide=False",
        "(-0.04, -0.035, PLATE_T + 0.01), (0.04, 0.045, 0.020), color=GREEN, collide=False",
        "(0.02, -0.02, PLATE_T + 0.003), (0.05, 0.04, 0.006), color=CRUMB, collide=False",
        "(COVER_XY[0] - PASS_XY[0], COVER_XY[1] - PASS_XY[1], 0.0005), (0.26, 0.26, 0.001), color=MAT,",
        "(0.62 - PASS_XY[0], 0.0, 0.20), (0.18, 0.34, 0.40), color=GUEST_C, collide=False",
        "(0.0, 0.0, 0.003), (TUB[0], TUB[1], 0.006)",
        "(0.006, TUB[1], TUB[2])",
        "(TUB[0] - 0.012, 0.006, TUB[2])",
        "], mass=None, color=PASS_C, friction=(0.5, 0.4)",
        "], mass=None, color=TUB_C, friction=(0.5, 0.4)")), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Clear from the right, serve from the left",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "restaurant service", "table service", "clearing and serving", "plated service",
                 "guest's left and right", "frame mapping", "order of steps", "unstated rule"],
    "instruction": t.INSTRUCTION,
    "context": (
        "The instruction gives the house standard of table service. Restaurantware's fine-dining tips for "
        "servers: serve from the left, clear from the right; WebstaurantStore's guide: always clear from the "
        "guest's right (the serving side varies by house); Tonno restaurant's server training: pre-plated "
        "courses served from the left, cleared from the right. The guest faces the robot, so the guest's "
        "right is the robot's left: the robot must map the guest's sides onto its own frame. One rule is "
        "unstated: the new plate is never set down before the used one has gone (doing so scores 0). The "
        "natural way is the left arm clearing toward the guest's right while the right arm serves from the "
        "guest's left, at the same time, as two servers would. One arm could do both in turn, so two arms are"
        " natural, not required. Done by: waiters, bussers, banquet and catering servers."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table; sizes x by y by height. The guest (visual only: a blue-grey block 18 x 34 x 40 cm, centred at "
        "x 62, y 0) sits facing the robot, so the guest's right is the robot's left (+y) and the guest's left is the "
        "robot's right (-y). Cover (the guest's place setting): a brown placemat 26 x 26 cm, 1 mm thick (visual "
        "only), centred at x 36, y 0. Starter plate: 0.25 kg, porcelain, square, 20 x 20 cm: a 1 cm base with a "
        "raised rim lip 1.2 cm wide and 1.5 cm high round its edge (2.5 cm tall in all), on the placemat at x 36, "
        "y 0, with crumbs on it (visual, 5 x 4 x 0.6 cm). Pass: a fixed grey shelf 22 x 24 x 4 cm, centred at x 14, "
        "y 0. Main course plate: 0.30 kg, the same plate, on the pass at x 14, y 0, with a steak 5 x 7.5 x 2.5 cm "
        "centred 4.5 cm from the plate's centre toward the guest, and mash (4.5 x 5 x 3 cm, on the left) and greens "
        "(4 x 4.5 x 2 cm, on the right) centred 4 cm from the plate's centre toward the robot (the food is visual "
        "only). Bus tray: fixed, dark grey, 24 x 24 cm with a rim 1.5 cm high (floor and walls 0.6 cm thick), "
        "centred at x 20, y 30, on the robot's left (the guest's right)."),
    "scoring": (
        "Every step, from poses; the score at the end counts. A plate's path is its centre's horizontal position at "
        "every step from the first step its underside is more than 1 cm above its starting height. The side is "
        "judged where the path crosses the circle 10 cm round the cover's centre (interpolated between steps): the "
        "starter is cleared from the guest's right if its path first leaves the circle at least 5 cm to the robot's "
        "left (+y) of the cover's centre, and the main is served from the guest's left if its path last enters the "
        "circle at least 5 cm to the robot's right (-y), that is within 60 degrees of that side; a plate never "
        "lifted has no side. Placed: the starter rests on the bus tray (its centre within 7 cm of the tray's centre "
        "along x and along y, its underside within 1.2 cm of the table) and the main is down on the cover (its "
        "underside within 4 mm of the table, its centre within 3 cm of the cover's centre), no finger pad within "
        "8 mm of either plate, and both plates moving less than 2 mm over 1 s. If placed: 1.0 with both sides "
        "right, 0.5 with one side wrong, 0.25 with both wrong; minus 0.25 (not below 0) if the steak points more "
        "than 25 degrees away from the guest (+x), seen from above. 0 otherwise, and 0 from the moment the main is "
        "down on the table (underside within 4 mm) within 8 cm of the cover's centre before the starter's centre "
        "has been 20 cm from it, the main is left resting on the starter (their centres within 8 cm horizontally, "
        "the main's underside above the starter's and at most 2.5 cm above it, no finger pad within 2 cm of the "
        "main), either plate's centre passes beyond x 50 cm (over the guest's lap), a finger pad "
        "comes within 8 mm of the main course's food (a zone 13 x 13 x 4 cm over the plate's centre, from the top "
        "of its base up), or a plate is dropped (it falls 3 cm within 0.5 s with no finger pad within 2 cm of it). "
        "Success: 1.0."),
    "run_limit": "1,200 steps (1 minute at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Put the starter plate back on the placemat at the cover and the main course "
                    "plate back on the pass, steak toward the guest."),
    "fidelity": (
        "Rigid boxes: each plate is a base and four rim lips; the pass and bus tray are fixed. Contacts and "
        "friction are simulated. The food is visual and fixed; an 8 mm finger keep-out over the main plate "
        "stands in for it. The guest is a static block; the line x 50 cm, which no plate's centre may cross, "
        "stands in for the lap. The arms face the guest across the table, so the serving side is the plate's "
        "path: the side on which it crosses a 10 cm circle round the cover, at least 5 cm to that side."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
