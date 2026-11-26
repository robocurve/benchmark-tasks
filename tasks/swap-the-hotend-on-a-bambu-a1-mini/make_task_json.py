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


def cm(v):
    s = f"{100 * v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


assert (t.SEAT_TOL, t.SEAT_TILT, t.LOCK_DEG, t.LATCH_MM, t.SLOT_TOL, t.SLOT_TILT, t.RELEASE, t.HOLD_STEPS,
        "FallWatch(0.02, 10)" in SRC, "kk.near(arm, *_sink_box(name), 0.015)" in SRC,
        "np.all(np.abs(err) <= SEAT_TOL) and tilt <= SEAT_TILT" in SRC,
        "abs(ps[2] - STAND_H) <= 0.003" in SRC, "StillWatch(0.0015, HOLD_STEPS)" in SRC,
        "latched = shut and track > -LATCH_MM" in SRC,
        '0.2 * removed + 0.4 * seated + 0.3 * lock_ok + 0.1 * (_S["hold"] >= HOLD_STEPS)' in SRC,
        'veto = _S["dropped"]["hotend_new"]' in SRC, "BED_TOP + 0.004" in SRC, "tip[2] < TOP + 0.004" in SRC,
        'done = (not veto) and removed and _S["hold"] >= HOLD_STEPS' in SRC) == (
        0.001, 3.0, 4.0, 0.6, 0.003, 8.0, 0.010, 20, True, True, True, True, True, True, True, True, True, True,
        True), "task.py thresholds changed: update the scoring text"
assert ((t.SINK_X0, t.SINK_X1, t.SINK_W, t.SINK_H), (t.STUB_R, t.STUB_H), (t.TUBE_R, t.TUBE_L), round(t.TIP_Z, 4),
        t.HOTEND_MASS, t.BAR_LEN, t.TRACK, tuple(round(v, 4) for v in t.HINGE), tuple(t.TAB["size"]), t.HEAD_FRONT,
        (t.RECESS_L, t.RECESS_R), t.PLATE_X, t.CEIL, t.HEATER_Z0, (t.DUCT_Y, t.DUCT_X0, t.DUCT_Z), t.STAND_H,
        t.SLOT_W, t.STAND_X, t.STAND_Y, (t.BOSS_X, t.BOSS_Y, t.BOSS_H), t.STAND.mass, t.PRINTER.mass,
        "limits=(0.0, 95.0)" in SRC, "(-8.4, -6.4, 6.0, DUCT_Y, -18.5, -15.5, BUCKLE_C)" in SRC,
        tuple(round(v, 4) for v in t.TH - [0, 0, t.TOP]), tuple(round(v, 3) for v in t.STAND_POS[:2]), t.SLOT_Y) == (
        (-0.0115, 0.0085, 0.0155, 0.0165), (0.0022, 0.0027), (0.00325, 0.0253), -0.0281, 0.018, 0.015, 0.0025,
        (-0.0046, -0.0075, -0.017), (0.008, 0.005, 0.005), -3.0, (12.75, -9.75), 10.0, 17.5, -20.9,
        (10.5, -3.0, (-24.6, -4.0)), 0.100, 0.0075, 0.040, 0.160, (0.012, 0.010, 0.030), 0.2, 5.5, True, True,
        (0.375, 0.0, 0.2246), (0.25, 0.03), {"new": -0.065, "empty": 0.065}), \
    "task.py layout changed: update the setup text"

P0, TH = t.P0, t.TH
thz = TH[2] - t.TOP  # the heatsink's underside (the heating assembly's top) above the table
duct_z = thz + t.DUCT_Z[0] * t.MM
sy = {k: t.STAND_POS[1] + y for k, y in t.SLOT_Y.items()}

task = {
    "title": "Swap the hotend on a Bambu Lab A1 mini 3D printer: buckle open, old hotend to its holder, new one seated, locked",
    "category": "Impact on AI timelines",
    "keywords": ["3D printer maintenance", "hotend swap", "push-to-unlock buckle", "slide-in seating", "fixed order",
                 "Bambu Lab A1 mini", "printed robot parts"],
    "instruction": t.INSTRUCTION,
    "context": (
        "Robot building: low-cost robot arms are built from printed parts (LeRobot's SO-ARM100 printing guide lists "
        "Bambu Lab A/P/X-series printers; XLeRobot's parts were printed on a Bambu Lab A1), so a lab building them "
        "keeps its printers running as part of the build. Between parts the hotend is swapped to clear a clog, replace "
        "a worn nozzle or change size for the next part (0.2 mm for fine detail, hardened steel for abrasive "
        "carbon-fibre filament). Bambu's procedure for the A1 series (wiki: Hotend Replacement for A1 Series; the A1 "
        "mini takes the same hotend): with the printer off and cool, the filament cut and the front cover and silicone "
        "sock off, push the nozzle buckle to the right to unlock it and swing its left side out, hold the heatsink and "
        "pull the hotend out, put the new one in until it sits flush, close the buckle and lock it. The hard part: a "
        "small part taken out and seated under the toolhead, between unlocking and locking."),
    "setup": (
        "Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left); heights above "
        "the table. Bambu Lab A1 mini 3D printer, powered off and cold, front toward the robot; a loose 5.5 kg block "
        f"model: base 28 x 24 x 7 cm (x {cm(P0[0] - 0.14)} to {cm(P0[0] + 0.14)}, y -12 to 12) with an 18 x 18 cm bed; "
        "Z column at the right rear; X arm and carriage behind the toolhead. Toolhead (front cover and silicone sock "
        f"off, filament cut) parked with the hotend's axis at x {cm(TH[0])}, y 0, its part-cooling ducts' undersides "
        f"{cm(duct_z)} cm up. Installed hotend (0.4 mm, dark heatsink) as on the A1: heatsink 2 cm front to back, 1.55 "
        f"cm wide, 1.65 cm tall, in a recess under the toolhead's body with its front 0.85 cm proud of the body, "
        f"resting on the heating assembly ({cm(thz)} cm up) with its magnet block against the assembly's mounting "
        "plate; 0.65 cm tube in the assembly's groove; nozzle 0.35 cm below the ducts. Buckle: a 1.5 cm bar across "
        "the front of the tube, 1.55 to 1.85 cm below the heatsink, on a vertical hinge at its right end that rides "
        "a 0.25 cm slide; locked, its left end sits behind a catch on the left duct; a 0.8 x 0.5 x 0.5 cm tab on its "
        "front near the left end. Hotends: Bambu Hotend - A1 Series, 4.73 cm long, 18 g. Holder: printed PLA, 200 g, "
        f"loose: a 4 x 16 x 7 cm block (x {cm(t.STAND_POS[0] - t.STAND_X / 2)} to "
        f"{cm(t.STAND_POS[0] + t.STAND_X / 2)}, y {cm(t.STAND_POS[1] - t.STAND_Y / 2)} to "
        f"{cm(t.STAND_POS[1] + t.STAND_Y / 2)}) with two 1.2 x 1 x 3 cm bosses on top (tops {cm(t.STAND_H)} cm up), "
        f"each with a 0.75 cm square hole for the tube: the right slot at y {cm(sy['new'])} holds the new hotend "
        f"(0.2 mm, green tag, heatsink on the boss), the left one at y {cm(sy['empty'])} is empty."),
    "scoring": (
        "Score = 0.2 x old out + 0.4 x new seated + 0.3 x locked + 0.1 x finished; 0 if the new hotend is dropped. "
        "Old out: the old hotend stands in a holder slot (either; only the left one is free at the start; axis within "
        "0.3 cm of the slot's, heatsink on the boss within 0.3 cm, within 8 degrees of upright) and was not dropped on "
        "the way. Dropped: a hotend falls 2 cm within 0.5 s with no finger pad within 1.5 cm of its heatsink while not "
        "in the recess or a slot, or its nozzle reaches the bed or the table. New seated: the new hotend within 0.1 cm "
        "of its seat along each axis (on the heating assembly, flush against the mounting plate, centred) and within "
        "3 degrees of upright. Locked: with the new hotend seated, the buckle within 4 degrees of shut and its slide "
        "within 0.06 cm of the locked end (the bar's left end behind the catch). Finished: seated and locked, no finger "
        "pad within 1 cm of the new hotend's heatsink or the buckle, and the new hotend still for 20 steps (1 s); the "
        "run is a success, and ends, once that has held with the old hotend in a slot. Real trial, from photos and "
        "the video: old out = standing upright in either holder slot (end photo); locked = the buckle shut and pushed "
        "back left so that it is latched (end photo; it will not open by pulling its left side); finished = no part of "
        "either gripper touches the new hotend or the buckle in the last 1 s (video); seated = the heatsink flush "
        "against the toolhead (a strip of printer paper, about 0.1 mm, does not slide between it and the heating "
        "assembly) and no wobble when the heatsink is nudged; dropped = the new hotend falls or is set down on the "
        "bed, the table or the floor."),
    "run_limit": "2,400 steps (2 minutes at 20 Hz)",
    "reset_notes": (
        "Nothing is consumed. Put the old hotend back in the recess and close and lock the buckle, stand the new "
        "hotend in the holder's right slot, and put the printer and the holder back on their tape marks. Real trials: "
        "the printer stays off and cool (below 50 C) with the front cover and silicone sock off and the filament cut; "
        "powered off, its toolhead slides along the X arm, so slide it back to its tape mark (hotend axis at y 0) "
        "before each trial; the two hotends may swap roles between trials."),
    "fidelity": (
        "Rigid bodies; printer a loose 5.5 kg block model, toolhead fixed to it. Hotend at the real one's size (47.3 mm, "
        "heatsink 20 x 15.5 x 16.5 mm, 18 g); toolhead and duct sizes estimated from photos. No magnet: the hotend "
        "rests on the heating assembly and the closed buckle holds it. Buckle: push 2.5 mm right on a damped slide to "
        "free its left end from a catch, swing out (a light spring holds it open), shut, push back left to lock; its "
        "parts are 20-50 g (real ~5 g) for the solver."),
    "scores": scores,
}
assert len(task["title"]) <= 120 and len(task["context"]) <= 1000 and len(task["fidelity"]) <= 500 \
    and len(task["scores"]) <= 300 and len(task["run_limit"]) <= 200, \
    (len(task["title"]), len(task["context"]), len(task["fidelity"]), len(task["scores"]))
try:  # keep the board media ID written by tools/upload_media.py
    _media = json.load(open("task.json")).get("media")
except (OSError, ValueError):
    _media = None
if _media:
    task["media"] = _media
json.dump(task, open("task.json", "w"), indent=2, ensure_ascii=False)
open("task.json", "a").write("\n")
print("task.json written:", len(json.dumps(task)), "chars; title", len(task["title"]), "context", len(task["context"]),
      "fidelity", len(task["fidelity"]), "setup", len(task["setup"]), "scoring", len(task["scoring"]))
