"""Writes task.json from task.py's constants and the texts below (so instruction and numbers cannot drift)."""
import ast
import importlib.util
import json
import sys

scores = sys.argv[1] if len(sys.argv) > 1 else "Dream-1 run: pending; demo: 1.0"  # read before argv is cleared
sys.argv = [sys.argv[0]]
spec = importlib.util.spec_from_file_location("t", "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
assert (t.ON_OFF, t.ON_TILT, t.ON_HEIGHT, t.DRAG_DOWN, t.DRAG_MAX, t.TIP_MAX, t.POT_NEAR, t.DROP_FALL, t.DROP_STEPS,
        t.RELEASE_DIST, t.STILL_MOVE, t.STILL_STEPS) == (
        0.015, 10.0, 0.005, 0.003, 0.030, 30.0, 0.020, 0.05, 10, 0.008, 0.002, 20), \
    "task.py thresholds changed: update the scoring text"
assert (t.COOK_XY, t.COOK_SIZE, t.LARGE_XY, t.SMALL_XY, t.LARGE_RING_R, t.SMALL_RING_R, t.LARGE_OUT, t.SMALL_OUT,
        t.WALL, t.LARGE_H, t.SMALL_H, t.LARGE_POS[:2], t.SMALL_POS[:2], t.EAR, t.EAR_Y, t.EAR_Z, t.REACH_Y) == (
        (0.30, 0.0), (0.28, 0.48, 0.010), (0.30, -0.110), (0.30, 0.125), 0.100, 0.064, 0.109, 0.070, 0.004, 0.090,
        0.075, (0.30, 0.125), (0.28, -0.335), (0.020, 0.030, 0.012), 0.120, 0.075,
        {"large_pot": 0.135, "small_pot": 0.070}), "task.py layout changed: update the setup text"
src = open("task.py").read()  # the masses and the sliding rule are not module constants: check them in the source
masses = {c.args[0].value: ast.literal_eval(k.value) for c in ast.walk(ast.parse(src))
          if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "part" for k in c.keywords if k.arg == "mass"}
assert masses == {"cooktop": None, "large_pot": 1.50, "small_pot": 0.35}, \
    "task.py masses changed: update the setup text"
assert all(s in src for s in ("low = z - r_out * np.sin(np.radians(tilt))",
                              "if _over_glass(xy) and low - CT <= DRAG_DOWN:",
                              'np.linalg.norm(xy - _S["anchor"][name])', '_S["anchor"][name] = None')), \
    "task.py sliding rule changed: update the scoring and fidelity texts"
task = {
    "title": "Put each pot on the burner that fits it",
    "category": "Impact on jobs",
    "keywords": ["bimanual", "two-handed carry", "kitchen", "cooktop", "size matching", "task ordering",
                 "lift, don't slide"],
    "instruction": t.INSTRUCTION,
    "context": ("Cookware guidance gives both rules. Le Creuset's care and use guide says never to slide its "
                "cookware on a glass-topped cooktop but always to lift it on and off, and to use a burner similar in "
                "size to the base of the pan; Kaizen Cookware's buyer guide for glass-top stoves says to lift pans "
                "instead of sliding them (most scratches come from dragging pans or debris trapped under the base) and "
                "to match the pan size to the burner so heat is not wasted. Here the big pot was left on the small "
                "burner, so it has to move "
                "first, before the small pot can take its place. The big pot weighs 1.5 kg and has a handle on each "
                "side; the scripted demo lifts it with both arms, one per handle, and moves the small pot with one "
                "arm. Done by: cooks (O*NET 35-2014.00, 35-2012.00) and anyone cooking at home."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Cooktop: black glass, 28 cm along x, 48 cm along y, 1 cm thick, centred at x 30, y 0, with two "
        "red burner rings (6 mm wide) printed on it: a large one 21.2 cm across centred at x 30, y -11, and a small "
        "one 14 cm across centred at x 30, y 12.5. Large pot: 1.5 kg, steel, 21.8 cm across outside, 9 cm tall, 4 mm "
        "walls, with two dark side handles, one pointing left and one pointing right (along y). Each handle is 2 cm "
        "wide (along x) and 1.2 cm thick, sticks out 2.6 cm from the wall and has its centre 7.5 cm above the pot's "
        "base (its top 9 mm below the rim); the pot is 27 cm across from handle tip to handle tip. It starts centred "
        "on the small burner (and hides it). Small pot: 0.35 kg, copper, 14 cm across outside, 7.5 cm tall, 4 mm "
        "walls, no handles; it starts on the counter at x 28, y -33.5, to the right of the cooktop."),
    "scoring": (
        "Every step, from poses; the score at the end counts. A pot is on a burner when its centre is within 15 mm "
        "of the burner's centre, it is within 10 degrees of upright and its base is within 5 mm of the glass. 0.5 for "
        "each pot on the burner that matches its size (large on large, small on small), counted once both pots are "
        "released (no finger pad within 8 mm of either pot, the large pot's handles included) and both still (each "
        "moving less than 2 mm over 1 s); 1.0 for both. 0 if a pot was ever slid on the glass, tipped past 30 "
        "degrees, or dropped (it falls 5 cm within 0.5 s with no finger pad within 2 cm of it). A pot is on the glass "
        "while its centre is over the cooktop and the lowest point of its base's rim (the base's height minus the "
        "pot's outer radius times the sine of its tilt) is within 3 mm of the glass. Each time a pot comes onto the "
        "glass, the position of its centre is noted; if its centre then gets more than 3 cm (horizontally, in a "
        "straight line) from that position before the pot leaves the glass again, it was slid. Lifting a pot more "
        "than 3 mm clear of the glass ends the stretch, so a lifted carry is not sliding, and small shifts in "
        "separate stretches do not add up."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": ("Nothing is consumed. Put the large pot back on the small burner with its handles pointing left "
                    "and right, and the small pot back on the counter."),
    "fidelity": (
        "Rigid boxes. The burner rings are drawn on the glass; there is no heat. Scratching is not simulated,"
        " so sliding is judged from how far a pot's centre moves after touch-down while its rim's lowest "
        "point stays on the glass. The pots are empty: the large one 1.5 kg with two rigid side handles, the "
        "small one 0.35 kg with none. In the recorded one-handed run the large pot slid about 25 cm, did not "
        "tip, and scored 0. Each pot is within 6 mm of its ring's diameter."),
    "scores": scores,
}
json.dump(task, open("task.json", "w"), indent=2)
print("task.json written:", len(json.dumps(task)), "chars")
