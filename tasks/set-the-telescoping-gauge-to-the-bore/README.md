# Set a telescoping gauge to a bearing housing's bore diameter

A telescoping gauge has been left in a bearing housing's bore, lightly locked and tilted, so its span is about 3 mm
too large. The robot must set it to the bore's true diameter and put it on its stand. The plungers only move in when
the bore's walls push them harder than the lock holds. Swinging the handle through level, with the head on the
bore's centre line, does that. A gauge pulled straight out keeps its oversize span, and one swept off the centre line
reads short.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. The folder also has:
- `parts.py`, a kit for multi-shape parts with sliding joints. This copy adds spheres, turned boxes, a part's full
  start orientation, and slider extension, rest point and friction fields.
- `lots.py`, which runs layout lots. This copy adds per-lot joint starts and a hook for the task's own start-up steps.
- `tilt.py`, which lets the scripted demo lean the gripper. It is used only by `demo()`.
- `near_misses.py`, plain-Python checks of `score()` and `success()` that need no simulator.

All three helper files patch the stock `yam_arena` package in the task's own process only; nothing is changed on disk.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Set the telescoping gauge to the diameter of the bore it is in, and leave it on its stand."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/gauge-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/gauge-demo
```

- **Lots:** trial k uses lot k mod 3 (A, B, C), so `--epochs 3` runs each once and `task_score` is their mean.
  `TASK_LOT=B` in the environment pins every trial to one lot. Without `--epochs 3` only lot A runs.
- **Runtime per trial:** at most 30 s of simulated time (600 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so every object starts where `task.py` and the lot put it.

## Objects and lots

| lot | housing at the work position | gauge's starting span | spare housings at y -0.040 / -0.160 |
|---|---|---|---|
| A | 40.00 mm bore | 42.6 mm | 44.00 / 48.00 mm |
| B | 44.00 mm bore | 46.9 mm | 48.00 / 40.00 mm |
| C | 48.00 mm bore | 51.2 mm | 40.00 / 44.00 mm |

**Housings.** Each is 40 x 90 x 90 mm aluminium, 0.68 to 0.74 kg, with its bore axis 55 mm up. The bore is 72 flat
5-degree faces with flats at the top and bottom. Across flats it is the nominal size; across corners it is about 0.1%
larger.

**Gauge.** 80 g in all, range 32 to 54 mm. Each plunger is 5 mm across with a 2.5 mm radius tip. It sits on a slide
with a 120 N/m spring pushing it fully out, about 0.4 to 1.3 N in use. A sprung steel pad (300 N/m, about 4 N of
preload) presses on its side; at the simulator's friction of 0.5 this holds it against about 2 N along its axis.

**Start of each trial.** After the lot's housings are placed, the pads are released, the gauge is put back in its
start pose with the plungers 0.1 mm short of the walls, the springs push them out for 0.15 s, and the pads are set
again. That is what an inspector does: release the lock, let the plungers touch, lock lightly.

**V-rest.** The handle rests near its end on a fixed yellow V-rest, a holding fixture added so the gauge rests in the
bore at the start; a free gauge left tilted there falls sideways in the simulator.

**Colours.** The gauge's handle and knob are orange and its head, plungers and the V-rest yellow, so the gauge and its
tilt show in the screenshot against the grey housings.

## How the simulator scores it

- **Span:** the declared tip-to-tip span at joint 0 minus both plunger positions.
- **"Within 0.10 mm":** `abs(span - bore) <= 0.00010 m`. Every limit is compared with a 1e-6 margin.
- **"On the stand":** either way round; the gauge's long axis is within 10 degrees of level. Where it crosses each post, the axis is
  over the knurled handle, within 1.75 mm of the slot's centre across it, and 0.5 mm below to 1 mm above its resting
  height (45.0 mm).
- **"Out of every housing":** the gauge's box does not overlap any housing's box.
- **"Tips touch nothing":** each tip sphere is at least 1 mm from the table and from every other object's shapes.
- **"Let go":** every control step an `on_step` callback measures the distance from both arms' finger and gripper
  collision hulls to the gauge's shapes. Credit needs more than 5 mm. Before that callback has run, or if the arms
  cannot be read, the gauge counts as held and earns nothing (fail closed). `near_misses.py` checks this at import.
- **"Still":** moving under 1 cm/s and turning under 0.5 rad/s. A trial ends at 1.0 once that has held for 1 s.

## Scripted demo

`demo()` uses the left arm only and is the same in every lot; it never reads the lot or the bore size.
1. It grips the handle 75 mm from the head, from above, with the hand straight down.
2. It turns the gauge about the head, which is held at the bore's centre, from 21.4 degrees down to 8 degrees up in
   eight steps. It goes back to level and up to 8 degrees again. The hand leans with the gauge, 29 degrees at most,
   with the fingertips away from the robot, using `tilt.py`.
3. It lifts 1.5 mm (the loaded arm sags), draws the gauge straight out at 8 degrees up, levels it, and lowers it
   into the stand's slots from 2 mm up.
4. It opens slowly, waits, and rises straight up.

In GPU traces the span was set to about 40.08, 43.98 and 47.98 mm in lots A, B and C (the trace's joint readings
are rounded to 0.1 mm). It held unchanged through the carry and on the stand.

## Shortcuts considered

All scored offline in `near_misses.py`; for lots B and C the same file checks the scores directly.
- Pulled straight out, or rocked only part way: the span stays over. Score 0.
- Swept off the centre line: the span is a short chord, 0.11 mm or more under. Score 0.
- Left in the bore at the right span, or held over the stand: 0.
- Put down with a tip on the table: 0.
- Laid down out of the housing at the right span: 0.5, never 1.0.
- Lots: a span learned in lot A scores 0 in lots B and C. Lots stop one fixed span from passing; they do not stop a
  policy that memorises all three.

## Limits

- **Simulated contact:** in PhysX the tips stop up to about 0.08 mm short of the walls, so the simulated reading can
  run slightly over; that is why the band is 0.10 mm.
- **Solver:** the `parts.py` copy runs the gauge's articulation at 64 position and 8 velocity iterations, double the
  kit's 32 and 4.
- **Steps left out:** the real job's final step, reading the gauge with an outside micrometer, is not part of the
  task. The 0.10 mm tolerance is coarser than shop practice (about 0.01 mm).
- **Third-party assets:** none; every shape is a primitive.
