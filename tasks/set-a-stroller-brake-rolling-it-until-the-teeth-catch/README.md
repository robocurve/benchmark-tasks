# Set a stroller's brakes, nudging it till the pins drop

Set both rear brakes on the stroller so it cannot roll, then let go.

An empty sit-and-stand stroller stands with its rear toward the robot. Each rear wheel's brake lever drops a pin
between the teeth of a ring on the wheel's hub. At the start each pin sits over a tooth tip, so pressing the lever
alone stops short and it springs back when let go. The maker's step: press the lever and move the stroller slightly
forward or back until the teeth line up, then check by pushing the stroller.

## Run and score a trial

    python -m yam_arena.run --task tasks/set-a-stroller-brake-rolling-it-until-the-teeth-catch \
        --policy <policy> --scorer task_score

The scripted demo (`--policy yam_scripted`): each hand pinches its lever's pad lightly toward the housing below it,
both hands nudge the stroller 27 mm forward together (half a tooth pitch of wheel turn) until both pins drop between teeth, then press each pad fully
down past the detent, let go and back away.

- Runtime per trial: up to 900 control steps (45 s at 20 Hz); the demo finishes in about 15 s.
- Seeds: none; no seed is needed, the layout is fixed (no object jitter).
- Score: 0, 0.25, 0.5, 0.75 or 1.0 at the end of the trial, as task.json's scoring says.

## Real trials are safe

- Pinch points: the brake lever's pin and the hub teeth, and the lever pad over its housing; the arms work at the pads
  only, and nobody's hands go near the wheels while the robot runs.
- Rolling: the stroller stands on the plywood floor panel with its front wheels locked straight; the scorer stops a
  trial (score 0) once it rolls 150 mm, well short of the panel's end. An operator stands at the e-stop.
- Forces: the levers need about 20 N at full press; the stroller is empty, 12.5 kg, and is never lifted.

## Files

- `set_a_stroller_brake_rolling_it_until_the_teeth_catch.py`: the scene, the brake mechanism, the scorer and the demo.
- `ourkit/`: the shared mechanism kit the task file builds on (unchanged copy).
- `test_set_a_stroller_brake_rolling_it_until_the_teeth_catch.py`: text, json, cheat, demo and hold tests.
