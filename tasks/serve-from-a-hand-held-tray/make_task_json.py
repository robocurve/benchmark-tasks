"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
src = open("task.py").read()  # numbers written inline in task.py (part masses, the guest, the tray-home margins)
assert (t.SCORE_OVER, t.COASTER_TOL, t.UPRIGHT, t.REST, t.RELEASE_DIST, t.RIM_NEAR, t.TRAY_TOUCH, t.TRAY_HOME_TILT,
        t.FALL, t.TIP, "abs(p[0] - STATION_XY[0]) <= (STATION[0] - TRAY[0]) / 2 + 0.01" in src,
        "abs(p[1] - STATION_XY[1]) <= (STATION[1] - TRAY[1]) / 2 + 0.01 and abs(p[2] - STATION_TOP) <= 0.006" in src,
        "kk.near(a, _GLASS_C, _GLASS_S, g, 0.02)" in src) == (
        "end", 0.020, 10.0, 0.006, 0.010, 0.004, 0.005, 5.0, (0.05, 10), 45.0, True, True, True), \
    "task.py thresholds changed: update the scoring text"
assert (t.SCENE, t.STATION_XY, t.STATION, t.CLOTH_XY, t.CLOTH, t.COASTERS, t.COASTER, t.TRAY, t.TRAY_T, t.RIM_W,
        t.RIM_H, t.HANDLE, t.TRAY_XY, t.GLASS, t.GLASS_H, t.RIM_ZONE, t.GLASS_ON_TRAY,
        tuple(round(t.GLASS_POS[n][i], 4) for n in ("glass_a", "glass_b") for i in (0, 1)),
        "TRAY_PART = kk.part(\"tray\", TRAY_POS, TRAY_GEOMS, mass=0.15," in src, "], mass=0.10, color=GLASS_C," in src,
        "kk.part(\"guest\", (0.64, 0.25, TOP), [" in src, "kk.Geom(\"box\", (0.0, 0.0, 0.20), (0.20, 0.30, 0.40)," in src,
        "kk.Geom(\"box\", (0.0, 0.0, 0.47), (0.14, 0.14, 0.14)," in src, "color=COASTER_C,\n              collide=False)" in src) == (
        "station", (0.36, -0.32), (0.34, 0.22, 0.040), (0.40, 0.25), (0.20, 0.28, 0.002),
        {"near": (0.36, 0.20), "far": (0.40, 0.29)}, 0.080, (0.30, 0.16), 0.008, 0.008, 0.006, (0.020, 0.080, 0.030),
        (0.36, -0.30), 0.060, 0.120, 0.025, {"glass_a": (-0.042, 0.0), "glass_b": (0.042, 0.0)},
        (0.318, -0.3, 0.402, -0.3), True, True, True, True, True, True), \
    "task.py layout changed: update the setup text"
task = {
    "title": "Serve from a hand-held tray, never by the rim",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "soft bimanual", "tray service", "hold and serve", "restaurant", "hotel", "food safety",
                 "lip-contact surface", "unstated service rule"],
    "instruction": t.INSTRUCTION,
    "context": (
        "Hotel F&B practice is to hold the service tray in one hand, serve with the other, and never leave "
        "the tray on the guest's table (setupmyhotel.com tray-handling SOP; Food and Beverage Knowledge tray-"
        "service guide). FDA Food Code 4-904.11(A), as adopted in WAC 246-215-04930, requires clean tableware"
        " to be handled so its lip-contact surfaces are not contaminated, and tumblers are tableware: a clean"
        " glass is not held by its rim. The instruction states none of these, nor that the tray goes back to "
        "the station, so the task checks whether the robot serves as a server would, unprompted. Soft "
        "bimanual: the demo holds the tray up beside the table with one arm and serves with the other, but "
        "either arm reaches the station and the coasters, so one arm could serve straight from the tray left "
        "on the station, and the score allows that. Done by: restaurant and hotel servers."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Service station (fixed): a steel block 34 x 22 cm, 4 cm tall, centred at x 36, y -32. Tray: "
        "0.15 kg, dark grey, on the station, centred at x 36, y -30 with its long side along x: a slab 30 x 16 cm, "
        "0.8 cm thick, with a rim 0.8 cm wide and 0.6 cm high along each long side and a handle bar 2 cm (along the "
        "tray) x 8 cm x 3 cm tall standing across each end. Two clean tumblers (glass_a and glass_b): 0.10 kg each, "
        "empty, square 6 x 6 cm and 12 cm tall, the top 2.5 cm (the band that touches the guest's lips) a lighter "
        "colour, standing on the tray 4.2 cm either side of its centre along its length (centres at x 31.8 and "
        "x 40.2, y -30). Guest's table (fixed): a cloth 20 x 28 cm, 0.2 cm thick, centred at x 40, y 25, with two "
        "8 x 8 cm coasters marked on it (visual only, no collision): the near one at x 36, y 20 and the far one at "
        "x 40, y 29. Guest (fixed), seated beyond the table: a block body 20 x 30 cm and 40 cm tall with a 14 cm "
        "cube for a head on top (54 cm tall in all), centred at x 64, y 25."),
    "scoring": (
        "Every step, from poses; the score at the end counts. A glass is served on a coaster when it stands within "
        "10 degrees of upright, its base within 6 mm of the cloth's top and its centre within 2 cm (horizontally) "
        "of the coaster's centre, with no gripper pad within 1 cm of it; each coaster counts once (two glasses on "
        "one coaster count as one). The tray is back when its centre is within 3 cm (in x) and 4 cm (in y) of the "
        "station's centre, its underside within 6 mm of the station's top, tilted at most 5 degrees, and no "
        "gripper pad is within 1 cm of either handle bar. Score: 0.5 per served coaster (1.0 for both), capped at "
        "0.75 while the tray is not back. 0 from the moment any gripper pad comes within 4 mm of a glass's rim "
        "zone (its top 2.5 cm), the tray touches the guest's table (the lowest corner of its underside within "
        "5 mm of the cloth's top while its footprint overlaps the cloth), or a glass falls (it drops 5 cm within "
        "10 steps, 0.5 s, with no gripper pad within 2 cm of it) or tips more than 45 degrees. Success: both "
        "coasters served and the tray back, with none of these; the episode ends once success has held for 1 s."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Put the tray back on the station and stand the two glasses on it, either "
                    "side of its centre. In real-world trials, swap in a clean glass for any glass whose rim was "
                    "touched."),
    "fidelity": (
        "Rigid boxes with contacts and friction; station, cloth and guest are fixed, coasters are visual "
        "marks. The score uses poses, not forces. Liquid is not simulated: the glasses are empty (0.10 kg), "
        "so the tray is lighter than with full drinks, and spilling is not scored; served glasses must stand "
        "within 10 degrees of upright and none may tip past 45. A pad within 4 mm of a glass's top 2.5 cm "
        "stands in for touching its lip. The tray's underside within 5 mm of the cloth counts as setting it "
        "down."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
