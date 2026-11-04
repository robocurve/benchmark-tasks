# Replace a worn conveyor roller

Between the arms stands a short gravity-conveyor section: two steel side rails 30 cm apart with open U-slots in their
tops, holding two hollow aluminium drop-in rollers (4.8 cm across, 26 cm long, on 12 mm axle stubs), a grey one and a worn red one. A new
grey roller rests in a spare stand in front of the rails, and a green rubber mat lies on the table past them. The robot must
lift the red roller out, lay it on the mat, and drop the new roller into the red roller's slots, leaving the grey roller
where it is.

Replacing worn rollers is routine maintenance on the gravity conveyors of warehouses, packing lines and distribution
centres (O*NET 49-9041.00, 49-9071.00, 53-7062.00). Drop-in rollers are handled with a hand at each end: lifted level so
both axle ends clear the rails, and lowered level so both ends drop into their slots together. The demo uses two arms
moving one rigid object as a single unit through lift, carry and a two-point insertion, beside a roller that must not be
disturbed. The scoring doesn't require two arms; it credits the finished replacement. Held in one place, a roller tilts
in the grip: this demo run with the right arm alone, gripping 0, 6 or 9 cm from the centre, scored 0 each time, the
rollers tilting 7 to 16 degrees and catching on the rails. That rules out only this simple one-arm plan, not every
one-arm strategy.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, cylinders and spheres (each roller with its axle stubs) |

`rigidparts.py` wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena,
so the task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the worn red roller: put it on the green mat and drop the new roller into its slots." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/roller-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the worn red roller: put it on the green mat and drop the new roller into its slots." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/roller-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

Both arms move together, each gripping the roller 9 cm from its centre with the fingers across it (opened only 61 mm so
they clear the neighbouring roller):

1. They lift the red roller out of its slots, carry it past the rails' ends and lay it on the green mat.
2. They grip the new roller in its stand, pause for the grip to settle, and lift it slowly out of the stand.
3. They carry it over the empty slots and lower it until its axle stubs drop in, then let go and back up.

The demo reaches 1.0 (success) in about 440 steps (22 s). It scored 1.0 with `-E object_jitter=0` on NVIDIA L40S and RTX A5000 GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0.

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- 1.0 (success, held for 1 s): the red roller lies on the green mat (its centre over the mat, at most 1.5 cm above lying
  on the table); the new roller rests in the red roller's slots (both axle stubs within the slots' 22 mm width and 6 mm
  of the slot bottoms); the grey roller is still in its own slots; all three are still.
- Otherwise 0.4 for the red roller on the mat, plus 0.4 for the new roller in the red roller's slots.
- 0 if the grey roller is out of its slots.

In a real trial: a photo from above shows which roller is in which slots and the red roller on the mat.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 8 minutes on one RTX-class GPU, mostly waiting on the model (about 0.6 s per reply), plus about 1 minute for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout, so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid. Each roller is one rigid body (a cylinder with two axle-stub cylinders): it doesn't spin on
bearings, but it rolls freely where it rests. The rails and the stand are fixed boxes with open slots. The stubs are
round where real drop-in rollers often have hex ends. Nothing is deformable.

## Real-world build

Two 17 cm lengths of steel or aluminium angle with 22 mm notches cut in their top edges, clamped 30 cm apart; three
light hollow aluminium drop-in conveyor rollers (48 mm, about 26 cm between the frames, 12 mm axles, 0.4 kg), one painted
red; two short notched steel posts for the spare stand; and a 1 mm green rubber mat.
