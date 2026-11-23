from scorer import evaluate, grade

for grasp, recovery, want in [
    (False, 1., 0.), (True, -.1, .25), (True, .249999, .25),
    (True, .25, .5), (True, .499999, .5), (True, .5, .75),
    (True, .899999, .75), (True, .9, 1.)
]:
    assert grade(grasp, recovery, verified_brush_wipe=True, direct_gripper_wipe=False) == want

for bad in (float("nan"), float("inf")):
    try:
        grade(True, bad, verified_brush_wipe=True, direct_gripper_wipe=False)
    except ValueError:
        pass
    else:
        raise AssertionError("Non-finite recovery accepted")

images = {"recovery_fraction_raw": .99}
simulated = {"grasp_achieved": True, "contact_force_target": "camera_window_body",
             "contact_steps": 0, "residual": 1.0}
assert evaluate(simulated, images)["score"] == .25  # pickup and improved image alone
assert evaluate(dict(simulated, contact_steps=1, residual=.98), images)["score"] == 1.
assert evaluate(dict(simulated, contact_steps=1, residual=.98,
                     contact_force_target="camera_mount"), images)["score"] == .25
physical = {"grasp_achieved": True, "direct_gripper_wipe": True, "verified_brush_wipe": False}
assert evaluate(physical, images, physical=True)["score"] == .25  # bare gripper only
assert evaluate(dict(physical, verified_brush_wipe=True), images, physical=True)["score"] == .25  # mixed wiping
assert evaluate(dict(physical, verified_brush_wipe=True, direct_gripper_wipe=False),
                images, physical=True)["score"] == 1.
print("Scoring boundaries and brush-attribution cases passed.")
