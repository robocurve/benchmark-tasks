# Clear a snapped piece of filament out of a 3D printer's extruder and leave the extruder ready to load

The corner of a 3D printer stands on the table: a black aluminium frame beam with a NEMA 17 stepper motor and an
aluminium-bodied lever extruder of the common MK8 type on it, the extruder's red tension lever lying on top, and a spool
of orange filament on its holder behind. A 13 cm piece of standard 1.75 mm orange PLA filament has snapped off inside
the extruder: it sticks out of the extruder's inlet to the left, its broken tip held in the nip between the brass drive
gear and the idler bearing that the lever's spring presses onto it. The job is to get the piece out, drop it in the
grey scrap tray in front, leave the lever back down where it rests, and take both arms back to where they started. The
task is one task file,
[`pull_a_snapped_filament_out_of_a_3d_printers_extruder.py`](pull_a_snapped_filament_out_of_a_3d_printers_extruder.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: knowing that a lever extruder only lets go of the filament while its tension lever is held open, and
doing it: one hand holds the sprung lever down steadily while the other pulls a thin part straight out along its
length; then finishing the job (the piece in the tray, the lever let go, both hands back). The instruction names only
the goal.

Why two hands: the drive gear holds the piece until the lever is pressed down, and the lever springs back up as soon
as it is let go; the pad is 16 cm from where the piece sticks out. With one hand (each arm doing its part of the demo
alone, the other never moving) the demo scored 0 both ways: the left alone pulled the piece a few millimetres before
its fingers slid off it, and it sprang back into the drive gear; the right alone pressed the lever down, and the piece
stayed where it was.

## The real hardware

A real trial uses the same parts: a NEMA 17 (42 mm) stepper with an aluminium-bodied lever extruder of the common MK8
type for 1.75 mm filament (brass MK8 drive gear, 40 teeth, 11 mm across, 5 mm bore, on the motor shaft; a 623 idler
bearing, 10 mm across, on the lever 1.2 cm from its pivot; a 2 mm filament bore from the inlet fitting through the nip
to a PTFE tube, 4 mm outside and 2 mm bore). The lever's compression spring is chosen so that, with the lever's own
weight helping, the pad needs about 0.85 N to start moving, 2.5 N to turn the lever 8 degrees (pad about 1.3 cm down),
where the idler leaves the filament, and 5.1 N at its 20-degree stop. A 13 cm piece of 1.75 mm PLA weighs 0.39 g.
Reset: snap a fresh 13 cm piece off the spool, feed it in through the inlet with the lever held open until its tip is
3.5 cm past the centre line, and let the lever go.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The printer (frame beam, motor, extruder housing,
PTFE tube, spool) and the scrap tray are kinematic, fixed where they are put; the lever and the piece of filament are
loose rigid bodies made of shapes from `bodykit.py`.

- The lever: a hinge about the left-right axis on top of the housing (a `Hinge` from `bodykit.py`), sprung down onto
  its stop; pressing its pad down turns it up to 20 degrees. Pressed past 8 degrees, the idler is off the filament.
- The piece of filament: a rigid rod 1.75 mm across and 13 cm long, 0.39 g, as real PLA.
- The piece in the extruder: a joint (a `Thread` from `bodykit.py`, switched on from the start) keeps it on the
  filament path, free to slide along it, pulled by a stiff spring (4,000 N/m) toward a target the task file sets after
  every control step. While the lever is up and the piece's tip is still between the drive gear and the idler, the
  target stays where the piece is held: pulling it moves it a few millimetres at most, and it springs back. While the
  lever is pressed down, or once the tip is past the gear, the target follows the piece, so it slides out freely.
  Once its tip is out of the inlet the task file switches the joint off and the piece is loose. The piece passes
  through the extruder (they never collide).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the piece still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pull_a_snapped_filament_out_of_a_3d_printers_extruder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A piece of orange filament has snapped off inside the 3D printer's extruder. Take the broken piece out of the extruder and drop it in the grey scrap tray, leave the extruder's tension lever back down where it normally rests, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/filament-dream
```

The scripted demo, which reaches full score: the right hand comes down on the lever's pad with its fingers closed and
presses it down, and holds it. The left comes down over the piece where it sticks out of the inlet, takes it across,
pulls it straight out along its length, lifts it, carries it over the scrap tray, lays it in and lets go. The right lets
the lever back up, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pull_a_snapped_filament_out_of_a_3d_printers_extruder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A piece of orange filament has snapped off inside the 3D printer's extruder. Take the broken piece out of the extruder and drop it in the grey scrap tray, leave the extruder's tension lever back down where it normally rests, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/filament-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/filament-dream/*.json; do inspect-robots video "$log"; done
```

Set `FILAMENT_DEBUG=1` to print what the score is built from twice a second: the lever's angle and whether the idler
is off the filament, how far the piece has come out (and the most so far), whether it is still in the extruder's
path, where it is and whether it lies in the tray, where each TCP is and the fingers' openings.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 900 steps (104 s on the clock, with Dream answering in 0.61 s on average), and about 1 minute for the scripted demo (361 steps, 18.1 s of simulated time, 83 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
