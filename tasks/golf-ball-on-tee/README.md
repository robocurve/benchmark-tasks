# Push a tee into a rubber holder and balance a golf ball on it

Instruction: "Push the tee into the rubber holder, then balance the golf ball on the tee."

The robot pushes a wooden tee upright into a rubber tee holder standing on the table, then sets a golf ball on the
tee's cup so that it stays. The scene, the scorer and the scripted demo are in `golf_tee.py`; every size, mass and
friction, with its source, is in `layout.py`.

## Run and score a trial

On a machine with the YAM Isaac Lab arena installed:

    bash dev/run.sh <log name> <policy> 0 1200            # policy: dream, or yam_scripted for the demo
    DEMO_VARIANT=careless bash dev/run.sh careless yam_scripted 0 1200
    DEMO_VARIANT=nothing  bash dev/run.sh nothing  yam_scripted 0 1200

The score is printed as `task_score:` in `run.out`; `debug.log` holds the end-state report. The layout is fixed
(`object_jitter=0`; the seed changes nothing). A trial of 1,200 steps takes about 3 to 15 minutes, depending on
how busy the GPU is.

## Scoring

Read once at the end, with both hands at least 3 cm from the ball and the tee:

- 1.0: the tee stands in the holder within 5 degrees of vertical, its top 38 to 51 mm above the holder's top, and
  the ball rests on its cup (its centre inside the cup's 9.2 mm ring), moving under 2 cm/s.
- 0.5: the tee stands so; the ball is not on it.
- 0: anything else (the untouched scene, a leaning or high tee, a hand still near the ball or tee). A hand whose
  position cannot be read counts as not away.

## Scripted demo

The demo reads the tee's and ball's poses from the simulator to plan; it moves them only through the grippers, by
contact. The right hand pinches the lying tee's shank, lifts it, turns it tip down over the holder, starts it
33 mm in, lets go and backs away. The left hand, fingers closed, presses the tee's head straight down until its tip is
about 1 mm over the table (the tee then rests on the table, top 44.5 mm above the holder), opens and lifts away. Then the right hand picks the ball up by its equator, lowers it onto the cup over the tee's axis, opens
slowly and lifts away. (The arms would meet if the ball were fetched while the left presses the tee.)

## Results (seed 0, fixed layout)

| Run | Predicted | Score | Steps |
|---|---|---|---|
| demo (the video) | 1.0 | 1.0 (tee 0.3 deg, top 44.5 mm above the holder; ball 0.1 mm off the axis) | 1,063 |
| demo, repeat | 1.0 | 1.0 (same end state) | 1,063 |
| nothing done (`DEMO_VARIANT=nothing`) | 0 | 0 | 40 |
| careless: ball let go 9 mm off the tee's axis (`DEMO_VARIANT=careless`) | 0.5 | 0.5 (ball rolled off; tee upright, top 44.5 mm) | 1,119 |
| Dream-1 (`dream`, seed 0) | | 0 (pushed the ball about; never picked up the tee) | 1,200 |
