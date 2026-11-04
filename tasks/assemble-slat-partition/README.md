# Assemble a slotted partition

Three slotted rigid foam-board slats (16 mm thick, 8 cm tall) stand on edge in low racks on the station table: a long slat (30 cm)
with two slots cut halfway down from its top edge, 18 cm apart, and two short slats (20 cm), each with one slot cut
halfway up from its bottom edge at its middle. The robot must stand the long slat on the white line and slide each short
slat down across it so the slots interlock, the slats' top edges flush.

Partitions for crates, cartons, trays and fixtures are built this way from slotted boards (O*NET 51-9198.00,
51-7042.00, 53-7064.00). A 16 mm slat does stand on edge by itself, but it tips over past about 11 degrees and shifts when
pushed, so two arms are the natural way to do this: one steadies the long slat on its mark while the other slides the
first short slat down onto it. Once one crossing is interlocked the pair stands firmly, and the second short slat can go
on. The scoring doesn't require two arms; it credits the finished partition. The task tests a sustained hold by one arm
while the other inserts at a precise pose (4 mm of play each way in the slots) without knocking the held part.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, cylinders and spheres (each slat with its slots) |

`rigidparts.py` wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena,
so the task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build the partition: stand the long slat on the white mark and slide both short slats down across it so their slots interlock." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/partition-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build the partition: stand the long slat on the white mark and slide both short slats down across it so their slots interlock." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/partition-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

1. The right arm grips the long slat mid-height, 4.5 cm from its centre, lifts it 10 cm (clear of the short slats), carries
   it to the white line and stands it there, then holds it upright.
2. Meanwhile the left arm takes the left short slat out of its rack, 8 cm from its centre, and waits above the rack until
   the long slat is standing; then it carries the short slat over the long slat's left slot and lowers it across.
3. The pair now stands by itself: the right arm lets go, takes the right short slat and lowers it into the other slot.

The demo reaches 1.0 (success) in about 550 steps (27 s). It scored 1.0 with `-E object_jitter=0` on NVIDIA L40S and RTX A5000 GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0.

Closing the fingers shifts a slat a few millimetres along its length, the same way every run with `-E object_jitter=0`;
the demo carries each short slat so that the slat, not the fingers, lands on its slot (`SHORT_GRIP_SHIFT` in `task.py`,
measured in a traced run).

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- 1.0 (success, held for 1 s): both of the long slat's slots hold a short slat with the slots interlocked: all three
  slats standing on edge on the table; each short slat's centre within 6 mm of its slot along the long slat and within
  6 mm across it, measured in the long slat's frame (a slat outside a slot is at least 2 cm off); top edges flush within
  4 mm; the long slat's centre within 2 cm of the white line's centre and the slat turned no more than 5 degrees from
  the line; all three still.
- Otherwise 0.4 for each interlocked crossing (0.8 for a full partition off the line).
- Slats laid flat in a cross, or resting on top of each other without interlocking, score 0.

In a real trial: from the side, the three top edges are level; a tape measure gives the long slat's distance from the
line; from above, each short slat crosses the long one at a
slot.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 8 minutes on one RTX-class GPU, mostly waiting on the model (about 0.7 s per reply), plus about 1 minute for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout, so it may not reach 1.0 with object jitter.

## Fidelity

Each slat is one rigid body built from boxes, its slots real gaps; 16 mm slats in 24 mm slots leave 4 mm of play each
way, looser than a snug real partition. The racks and the white line are fixed boxes. Surface friction in the slots isn't
modelled beyond the station's default contact material, and each slat has the same 40 g, where a real short slat is a
little lighter. Nothing is deformable.

## Real-world build

Three 16 x 80 mm rigid foam boards (one 30 cm, two 20 cm) with 24 mm slots cut halfway through, two blocks taped either
side of each board as a rack, and a strip of white tape on the table.
