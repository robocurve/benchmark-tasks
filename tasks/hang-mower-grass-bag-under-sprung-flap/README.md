# Hang a push mower's rear grass bag on its coupling hooks under the spring-loaded ejector flap

A 14 in corded electric push mower (Sun Joe MJ401E, about $140) collects clippings in a 10.6 gal rear bag that comes
off to be emptied every few minutes. The manual's way to put it back: raise the spring-loaded ejector flap, set the
bag's rod over the two coupling hooks on the mower body, release the flap onto the bag. The flap is sprung shut and
swings out over the space the bag must go into, so one arm has to hold it up while the other lifts the bag by its
handle, carries it in under the flap, squares it to the mower and drops its 10 mm rod into two hooks that the bag
itself hides from view. The flap must then come down onto the bag, not stay propped up or drop onto a hand.

## Files

- `task.py`: scene, seeded start pose and scoring (yam_arena task file). `score()` and `success()` are the rubric in
  `task.json`. The rod stays in the hooks only by contact: there is no latch or proximity trigger.
- `robokit.py`: rigid parts built from several boxes and cylinders, with sprung joints (the flap). It hooks
  `yam_arena.scene` and the embodiment from the task's import; no change to yam_arena is needed.
- `demo.py` and `script.txt`: the scripted demo (see below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps over the work area); `BASELINE_SEED` seeds them.
- `media/checks.md`: demo and baseline runs, real-part sources, the observable-inputs check and the overlap check.
- `media/`: demo videos (top camera and both wrist cameras) and `setup.png`.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

The score is `task_score` in the log: 1.0 on success (held 1 s), otherwise `score()` from `task.py` at the end of the
episode. After every reset the task file places the bag with a seeded offset (0 to 6 mm, +-1 cm, +-2 deg) and lets
the scene settle before the first observation; `-E object_jitter=0` keeps the mower where the setup puts it.

Scripted demo (writes one MP4 per camera, `setup.png` and `result.json` per seed):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 1 2 --max-steps 2400

The demo uses only what a policy has: the first top-camera image, the arms' own joint angles (forward kinematics)
and the mower's fixed layout from the setup text. It never reads the simulator's object poses or the flap's joint.
It finds the bag on the image (side edges against the bench give its centre line and heading; the handle bar's
front end gives how far forward it lies), swings the flap up with the left arm along the hinge arc, lifts the bag by
its handle with the right arm, squares it to the mower and carries it over the hooks, correcting the bag's pose from
the measured joints (the wrist sags and turns under the load), lowers it and lets go 8 mm above the slot floors. It
then lowers the flap, reading the flap's angle from where the left fingertip is, until the fingertip stops following
its command (the flap is lying on the bag), lets go and backs both arms off.

## Runtime and seeds

- Run limit 2,400 control steps (2 min at 20 Hz). The demo reaches success at steps 863 to 873 on seeds 0 to 7.
- Wall time: about 25 to 35 minutes per trial on one TITAN RTX at 640 x 352 per camera, depending on load.
- `media/` holds the seed-0 demo videos and the 960 x 528 setup shot.
