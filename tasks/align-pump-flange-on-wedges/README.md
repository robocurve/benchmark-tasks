# Line a pump's outlet up with its pipe by raising its baseplate 15 mm on two wedges, never tilting it over 2 degrees

A pump on a blue baseplate stands on the thin ends of two opposing yellow wedges, which lie tips inward on a low-friction slide
plate. Its outlet flange sits 16 mm below a fixed pipe's flange. The robot must push both wedges in by their red pegs until
the baseplate's feet sit on the wedges' flat seats, 15 mm higher, so that the flanges line up, without the baseplate ever
tilting more than 2 degrees.

Millwrights and pipefitters set pumps on their foundations and raise them on wedges until the outlet lines up with the pipe
(O*NET 49-9044.00, Millwrights). The task needs both arms for a physical reason: the 1-in-5 slope is steeper than friction can
hold, so a wedge squeezes back out the moment it is let go, until the feet reach its seat. One arm can't work the two sides
in turn, and pushing one side ahead of the other tips the pump. It tests two arms sharing a load at a matched pace, and a
sustained push through a stroke that ends in a stable seat.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, spheres and convex prisms (the baseplate and pump, the wedges, and the fixed plate, posts and pipe) |

yam_arena's objects are single primitives, so `rigidparts.py` builds these parts. It wraps `yam_arena.scene.task_object` and
`object_states` when they load, without modifying yam_arena, so the task runs with the standard `yam_arena` embodiment,
`yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Line the pump's outlet flange up with the fixed pipe flange: push the two yellow wedges in together by their red pegs until the blue baseplate's feet sit on the wedges' flat seats, 15 mm higher, never letting the baseplate tilt more than 2 degrees." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pump-wedges-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Line the pump's outlet flange up with the fixed pipe flange: push the two yellow wedges in together by their red pegs until the blue baseplate's feet sit on the wedges' flat seats, 15 mm higher, never letting the baseplate tilt more than 2 degrees." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pump-wedges-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

1. Each arm grips its wedge's peg, its fingers in front of and behind the peg, so that a finger pushes it directly.
2. Both arms push in together over 8 s. The arms give about 2 cm under the load, so each hand travels 10.4 cm; the wedge tips
   meet at about 9.6 cm and stop there.
3. Both arms hold for 1.5 s, let go and back up. The baseplate stays on the seats.

The demo reaches 1.0 (success) in about 300 steps (15 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000 GPUs,
and also with 5 mm of object jitter (two seeds). The baseplate ends 16.1 mm up, and tilts at most 0.5 degrees on the way.
Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0.

Scripted checks of the rules: the right arm alone scored 0, the baseplate tipping past the limit; the left arm starting 4 s
after the right scored 0, for the same reason; both arms letting go 6 cm in scored 0, the wedges squeezing back out; both
arms pushing 7.5 cm in and holding to the end of the run scored 0.5.

## Scoring

Evaluated every control step; the score at the end of the episode counts. Rise is how far the baseplate's centre is above where it
started. How far in is 9.5 cm minus the distance from a wedge's tip to the midpoint. Tilt is the baseplate's angle from level.

- 0 if the baseplate ever tilts more than 2 degrees or ever rises more than 3 cm, whatever happens afterwards.
- Otherwise, with both wedges lying flat on the slide plate (within 4 mm of its surface):
  - 1.0 (success, held for 1 s): rise at least 15 mm, both wedges in at least 8 cm, the baseplate and wedges still.
  - 0.5: at the end, rise at least 10 mm with both wedges in at least 4 cm.
  - 0.25: at the end, rise at least 5 mm with both wedges in at least 2 cm.
  - Otherwise 0.

The wedge travel checks mean that lifting the baseplate by hand earns nothing. Partial credit needs the wedges held in place at the
end of the run, since they squeeze back out once released.

In a real trial: a tape measure gives the rise and each wedge's travel, and a digital level on the baseplate records the tilt.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 6 minutes on one RTX-class GPU, mostly waiting on the model, plus about 1 minute for
Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

All parts are rigid. The pump is a light hollow scale model, so the small arms can move it. Friction is 0.02 on the slide plate and 0.05 at the feet, a little below what oak would give on a real PTFE plate, and low enough that the 1-in-5 slope always squeezes a wedge back out until the feet reach the flat seats. The guide posts stand in for a real baseplate's anchor studs. The flanges are only lined up, not bolted. Nothing is deformable.

## Real-world build

A PTFE sheet 18 x 53 cm on the table with two steel rails and two steel end stops; two oak wedges 15 x 16 cm, each with a 2 x 2 x
5 cm peg; a 24 x 20 cm baseplate with two edge notches, a hollow steel pump body and outlet with a flange, and four ball-transfer
feet, 1.5 kg in all; two steel guide posts; a fixed pipe with a matching flange on a stand. The load is light and low, so real
trials are safe.
