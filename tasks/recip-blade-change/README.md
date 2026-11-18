# Fit a reciprocating-saw blade into its tool-free clamp

A cordless reciprocating saw lies on its side on the bench, its blade clamp facing the arms. The clamp block has a
blade slot and a jaw whose pointed pin reaches down into the slot through the block's roof. A worn blade has been
removed. A 150 mm wood blade lies flat across two wooden rests to the left, teeth down, its tang toward the saw's far
side.

The robot must fit that blade: pick it up off its rests, carry it to the clamp, line the tang up with the slot and push
it home. The tang lifts the jaw's pin by its point. When the tang's hole reaches the pin, the jaw drops back onto its
stop with the pin in the hole, and the blade's shoulder comes up against the clamp face. Then the robot lets go.

Swapping a blade is the first thing a trade does when the material changes, and on current cordless saws it is a
tool-free push-in. The task tests picking up a thin, flat part, presenting it end-first and level to a slot 3 mm tall,
and pushing it home against a sprung lock.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the saw and its clamp, the jaw, the blade and its rests, `success()` and `score()` |
| `partkit.py` | Rigid parts built from several boxes with spring-loaded sliders and hinges, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its negative variants |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the spare blade into the saw's clamp so it is locked, square and teeth down." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir dream_out
```

## Scripted demo

```bash
python demo.py task.py demo_out                    # full score: writes demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=short python demo.py task.py short    # stops with the tang 10 mm short of home: scores 0
DEMO_VARIANT=held python demo.py task.py held      # pushed home but never let go: scores below 1 (0.25)
```

The left arm closes across the blade between its rests, lifts it, carries it to the clamp line and lowers it level with
the slot. It pushes the tang 6 mm into the slot at 10 mm/s, presses it down onto the slot's floor, and pushes on until
the shoulder reaches the clamp face. Then it opens, lifts clear and withdraws. Full score is reached in 377 steps.

The demo uses only what a policy could see: the fixed layout in the task file (`object_jitter=0`) and its own joints.
Once the blade is held, the arm reads its own tool point from its joint positions and shifts every later target by the
difference from the layout. It never reads the jaw, the blade's pose or any task state.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if the saw shifted more than 3 mm at any point.
- Seated: shoulder within 1.5 mm of the clamp face, centreline within 1.5 mm of the slot's, tang on the slot's floor
  (within 1.5 mm), square within 4 degrees in pitch and roll.
- Locked: the jaw within 0.9 degrees of its closed stop, which means its pin is in the tang's hole.
- 1.0 (success, held for 1 s): seated and locked, every finger pad at least 30 mm from the blade and the jaw, and
  nothing moving more than 1 mm over that second; 0.75: seated and locked but a gripper is within 30 mm or something
  still moves; 0.5: seated with the jaw open and both grippers 30 mm or more away; 0.25: the tang is in the slot, up to 30 mm from home (or seated but still held); else 0.

In a real trial, check with a ruler that the blade's shoulder touches the clamp face and that the blade's edge is
square to the clamp; the lock is checked by pulling the blade gently (it does not move).

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus Isaac Sim start-up. The layout is fixed (`object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies from boxes at true size. The tool-free clamp is a hinged jaw on a closed stop with a pointed pin into the
tang's hole, as on push-in clamps; a release lever is not modelled (the jaw is lifted by hand to reset). The blade is
1.6 mm stock with a 0.4 mm tooth strip, not cut teeth. The saw's body is a box behind the clamp. Blade flex and tooth
sharpness are not modelled.

## Real-world build

- A cordless reciprocating saw whose tool-free clamp takes the blade straight in and snaps shut on it. Makita and
  Bosch clamps lock open for the blade and close as it goes in. Lay the saw on its side, clamp end toward the robot,
  and clamp its body to the bench with the shoe flat on the table.
- A standard 150 mm (6 inch) wood blade with a universal tang.
- Two 25 x 30 x 30 mm wooden blocks as rests, set 85 mm apart on the line where the blade lies.

Reset by opening the clamp, sliding the blade out and laying it back across its rests, teeth down, tang toward the
saw's far side.
