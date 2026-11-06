# Measure a pin's diameter with a digital caliper, holding it in the jaws

A digital caliper clamped flat on a holder block, its jaws pointing at the robots and overhanging the block, and a
turned aluminium pin 23.40 mm across standing on the bench. One arm holds the pin upright between the open jaws,
loosely and close to the fixed jaw; the other pushes the slider's thumb rest until the sliding jaw has carried the pin
onto the fixed jaw and both just touch it, then lets go. The live display then shows the diameter. There is nothing
under the jaws, so the pin must be held, and the display reads the slider, not the jaw tip: squeezing flexes the jaw and the display reads low.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file). Every control step, an `on_step` hook reads the
  slider's position and drives the display's 28 segments to show the jaw gap it gives. The sliding jaw rides on the
  slider through a stiff spring (0.1 mm per newton); the score reads the jaw force from that spring's flex. The demo's
  pushing hand leans its closed fingers 30 degrees towards the fixed jaw, which keeps the two elbows apart; `task.py`
  lets a demo waypoint carry that full gripper rotation (the `Orient` class), so yam_arena itself is unchanged.
- `labkit.py`: parts built from several shapes, with hinges and slides, including joints chained on other joints
  (`parent=`): the slider is a damped slide with light friction, and the sliding jaw and the display segments ride on
  it. It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1200 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 36 s of motion; 800 steps are enough). The demo's
push is closed loop on the sliding jaw: the finger advances at 3 mm/s and stops as soon as the jaw flexes by 0.02 mm
(0.2 N, first touch), then backs off so the jaw rests on the pin without load. The holding hand holds the pin 3 mm
clear of the fixed jaw, loosely: while the sliding jaw presses on the pin, the hand gives way towards the fixed jaw
(10 mm/s per 0.1 N of jaw force), so the jaw carries the pin across and seats it on the fixed jaw. Should the jaw
press with 0.2 N before the pin is seated, the push pauses until the hand has given way; once the pin is seated, 0.2 N
stops the push. Run one trial per process: the
push and hold state is kept in the task module and cleared at each reset.

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state: the pin's pose between
the jaws, the display reading and the jaw's flex.

## Runtime and seeds

- 1,200 control steps (60 seconds of simulated time) per trial; about 5 minutes of wall time on one L40S with frames
  stored at 640 x 352 (estimated from similar tasks; not yet measured).
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
