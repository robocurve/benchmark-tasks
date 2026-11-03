# Hand scissors to a person

Files: `scissors_handoff.py` (scene, scoring, scripted demo), `task.json`, `check.py` (offline check),
`zero_action.py` (the same task with an empty plan),
`measure_scissors.py` (how the scissors' blade, rings and grip point were measured from the library model).

Environment: Isaac Lab Arena 0.3.1, Isaac Sim 6.1, inspect-robots 0.60.0, yam-isaac-lab-arena at 0059d02. The
scissors are `scissors_ycb_robolab` from the Arena object library.

## Offline check (no simulator)

```bash
python tasks/scissors-handoff/check.py
```

It tests the scoring on hand-built states: start, on the palm with the blades out, on the palm with the blades
over the wrist or the fingers, held over the palm, still moving, on the table, held at the start. It then plays
the demo through the IK at 20 Hz: the worst IK error and the joint-limit margins must be within limits.

## Run and score a trial

```bash
INSTR="Place the scissors on top of the person's hand."
TASK=tasks/scissors-handoff/scissors_handoff.py

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream -E task=$TASK -E object_jitter=0 -E control_hz=20 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --seed 0

# zero-action run (stands in for Dream-1, which refused the instruction): both arms hold READY
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scissors-handoff/zero_action.py \
  -E object_jitter=0 -E control_hz=20 -P hold_s=5 --instruction "$INSTR" --scorer task_score --max-steps 1200 --seed 0

# scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$TASK -E object_jitter=0 -E control_hz=20 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --seed 0
```

`task_score` gives 1.0 when `success()` holds for 3 s (`SUCCESS_HOLD_S`), otherwise `score()` on the last step. The rules are in
`task.json` (`scoring`) and in the docstring of `scissors_handoff.py`.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s). The demo plan lasts 14.0 s.
- Runtime per trial: TBD.
- Seeds: `--seed 0`, with `-E object_jitter=0` for a fixed layout. The hand is fixed, so jitter moves only the
  scissors (up to 3 cm). The start distance d0 in the score is from the nominal layout.

## Submitted videos and scores

- Dream-1: the Dream API refused the instruction with `DA-4000 BAD_REQUEST (malformed): unsafe instruction,
  refused` and sent no actions. The model video is therefore a zero-action run (`zero_action.py`): `task_score`
  0.0.
- Demo: `task_score` 1.0, ended by `success` after 306 steps (15.3 s).
