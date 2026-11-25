# Light a burned-down jar candle with a utility lighter: latch held in while pulling the trigger, nozzle deep in the jar

A glass jar candle that has burned down (its wick 3.7 cm below the rim) stands on the table, and a long utility
lighter lies on its side beside it. The instruction is "Light the candle in the glass jar with the utility lighter,
then put the lighter down on the table." It does not say how.

Every utility lighter sold in the US is child-resistant: the trigger is blocked until a separate latch is pressed,
and the gas flows only while the trigger is pulled. BIC Consumer Products, answering on its Multi-Purpose Lighter's
retail page ([BIC Multi-Purpose Lighter, Target][bic]): "hold the utility lighter with your thumb resting on the latch
and your index finger on the trigger" ... "depress the latch with the tip of your thumb and hold in" ... "pull the
trigger with your index finger" ... "The child-resistant mechanism will automatically reset after each use." The US
rule behind it (16 CFR 1212, multi-purpose lighters) requires the extra latch or switch. A burned-down jar candle is
the everyday reason long lighters exist: the wick is deep inside the glass, out of reach of a match.

The hand skill. This is not the same as lighting a dinner candle with a match: there the difficulty is striking the
match and holding a short flame at an exposed wick. Here:

- one grip must press a 3 mm latch fully in and keep it in while pulling a 10 mm trigger against 12 N, and the trigger
  moves only once the latch is in (a pinch on the trigger alone, or a grip that misses the latch, gives nothing);
- the lighter must be carried without firing it (the latch may be in, but the trigger not pulled);
- the 10 mm nozzle at the end of a 15 cm wand must go about 3 cm down into an 8.5 cm jar, its tip brought to within about 1 cm of the wick (3.7 cm below the rim)
  and held there, without knocking the 650 g jar off its place or over;
- then the trigger let go (the flame goes out), the nozzle drawn back out of the jar and the lighter laid down.

One arm is enough (as a person does it one-handed); the task does not claim two arms are needed. The left arm stays at
its ready pose in the demo.

The task is the task file
[`light_a_jar_candle_with_a_utility_lighter.py`](light_a_jar_candle_with_a_utility_lighter.py) for the `yam_arena`
embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: every part is built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one part on a spring.
The jar, the lighter with its sprung latch and trigger, and the flames are bodies built from several shapes, so
`bodykit.py` adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives the task file each body's pose after every control step.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, heights above the table top. Both arms start at the ready pose, grippers open. The layout is the same in every
trial.

- Jar candle, loose, 650 g: a clear glass jar 8.5 cm across and 10 cm tall (4 mm wall, 8 mm base) centred at (24, 0),
  filled with dark red wax up to 5.5 cm; a cotton wick 2 mm thick stands 8 mm out of the middle of the wax (its tip
  6.3 cm up, 3.7 cm below the rim).
- Utility lighter, loose, 60 g in all, the size of a BIC Multi-Purpose Classic: a red handle 10.5 cm long, 3 cm from
  its latch side to its trigger side and 2.4 cm thick, with a black end cap; a grey metal wand 15 cm long and 1 cm
  across from the handle's front end, ending in a steel nozzle (25.5 cm overall). It lies on its side with its handle
  centred at (15, -17), the wand pointing to the robot's left (+y; the nozzle tip at about (15.7, 3.3), 2.3 cm up),
  the latch side facing forward (+x) and the trigger side facing the robot.
  - Latch: a black button 14 mm long and 12 mm wide on the handle's top (latch) side, 4.2 cm behind the handle's front
    end, standing 3 mm proud; it is pressed straight in, up to 3 mm, against its spring (4.5 N fully in) and springs
    back out when let go.
  - Trigger: a black bar 24 mm long and 10 mm wide standing 12 mm out of the handle's underside, its middle 2.2 cm
    behind the front end (20 mm ahead of the latch, so a fingertip pinch across the handle covers both); it is pulled
    straight in, up to 10 mm, against its spring (12 N fully in). While the latch is in less than 2.5 mm the trigger
    stops after 2 mm (the child-resistant block).

What is modelled and what is not. Everything is rigid; there is no gas and no fire. The latch and the trigger are
parts on sprung slide joints with their stops, and the trigger's block is a joint limit (2 mm) that the task file
sets while the latch is out and lifts once it is in 2.5 mm or more; it is only set back once the trigger has
returned under 2 mm, as a real latch cannot re-lock a pulled trigger. The gas, the spark, the flame and the wick
catching are a rule (below); a flame (blue core, orange and yellow tongue, 25 mm) is drawn on the nozzle while the
gas flows and a candle flame on the wick once it is lit, as visual cues only. Spring forces and travels are estimates
for this kind of lighter. The glass jar is a 20-sided shell; wax and wick are rigid.

The gas and flame rule (every 50 ms control step):

- The gas flows while the latch is in 2.5 mm or more and the trigger is in 8 mm or more (80 % of its travel); the
  flame then stands 25 mm ahead of the nozzle tip, along the wand.
- The wick lights once its tip has been within 6 mm of that flame (of the 25 mm line ahead of the tip) for 1.0 s in
  all, with the gas flowing. It then burns for the rest of the run.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run, which
is the best stage reached:

- 0.5: the candle lit.
- 0.75: the candle lit, and the lighter put down: resting on the table (its lowest point under 2 cm up, still within
  1.5 mm over 1 s), its gas off, and every finger point of both grippers (from the YAM's finger collision meshes at
  the simulated poses) 2 cm or more from it.
- 1.0 (success, held 1 s): as 0.75, and the jar upright (tilted under 10 deg) and within 2 cm of where it started.
- 0 for the rest of the run if the jar is ever tilted 30 deg or more (a lit candle knocked over is a fire hazard).
- Otherwise 0: doing nothing, pulling the trigger without the latch held in (it stops at 2 mm: no gas), holding the
  flame outside the jar or away from the wick, or refusing.

Real trials. Default: a real candle and a real filled lighter of the kind above, on a non-flammable tray, with a lid
or snuffer and a small extinguisher at hand; the flame stays at the nozzle, 18 cm from the gripper pads. The candle is
lit if it is seen burning 2 s after the lighter's flame is gone; "put down", the jar's tilt and its 2 cm are checked
from the overhead video and with a tape measure against marks round the jar's base. Where an open flame is not
allowed, use an emptied lighter (gas used up and vented outdoors): its latch and trigger work as normal and its
spark clicks, and "lit" is judged by the rule above from a side video: the trigger fully in with the latch held, and
the wick within 6 mm of the wand's line within 25 mm of the nozzle (the 10 mm wand gives the scale) for 1 s.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=light_a_jar_candle_with_a_utility_lighter.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Light the candle in the glass jar with the utility lighter, then put the lighter down on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/candle-dream
```

In our run (2,400 steps) Dream-1 did not refuse. It moved the right gripper down to the lighter, then kept it
hovering beside the end of the lighter's handle for the rest of the run; it never picked the lighter up, nothing lit
and the jar did not move. It scored 0.

The scripted demo reaches full score open loop from the known layout. The right gripper, pointing straight down with
its fingers closing along x, comes down over the lighter's handle with the latch under one finger and the trigger
under the other, closes to a 40 mm gap (the latch and the trigger each go about 2 mm in: no gas) and lifts the
lighter 17 cm. Turning its wrist in free space, it swings the wand round to point down into the jar (62 deg below
level, the hand to the right of the jar and slightly beyond it), with the nozzle tip 8 cm back along the wand, above the rim; it slides
the nozzle 8 cm along the wand's line into the jar until the tip is about 1 cm from the wick. It then squeezes (a
24 mm gap commanded; the latch and trigger stop the fingers at 32 mm): the latch stays in, the trigger goes fully in,
the gas flows and the wick catches 1 s later. After 2.5 s it opens back to the 40 mm gap (the flame goes out), draws
the nozzle back out along the same line, turns the lighter level again and lays it down where it was, opens its
fingers and rises. The nozzle never touches the glass or the wax and the jar does not move. In our runs it scored 1.0
at step 525 (about 26 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=light_a_jar_candle_with_a_utility_lighter.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Light the candle in the glass jar with the utility lighter, then put the lighter down on the table." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/candle-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/candle-demo/*.json; do inspect-robots video "$log"; done
```

`CANDLE_DEBUG=1` prints, every 5 steps (`CANDLE_DEBUG_EVERY=10` for every 10), the latch's and the trigger's travel,
whether the trigger is blocked, whether the gas flows (and for how long in all), the wick's distance from the flame
line and the vector from the nozzle tip to the wick, the seconds of flame at the wick, whether the candle is lit, the
jar's tilt and how far it moved, the lighter's lowest point, the fingers' clearance from it, and each gripper's
position and opening. `CANDLE_PINCH`, `CANDLE_HOLD`, `CANDLE_AZ`, `CANDLE_EL`, `CANDLE_ROLL`, `CANDLE_TIP_BACK`,
`CANDLE_DX`/`DY`/`DZ` and `CANDLE_FIRE_S` change the demo's finger settings, the wand's aim, the tip's target and how
long it fires.

`CANDLE_NOLATCH=1` runs the demo the way a child, or a careless grip, would: the gripper takes the handle 2 cm further
toward the wand, so its fingertips cover the trigger and the front of the handle's top but not the latch, and squeezes
hard (the same 24 mm gap the demo fires with) all the way. The latch never moves, the trigger stays on its 2 mm block,
the nozzle is brought to the wick as in the demo (within 3 to 5 mm of the flame line for about 3 s), but no gas flows,
the wick never lights, and the run scores 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs: a debug run of the demo at 320 x 176 (ending
on success at step 525) took about 1.6 minutes of simulation plus about 1.5 minutes to start, the wrong-way run (700
steps) about the same, and the Dream-1 run (2,400 steps at 320 x 176) about 4.5 minutes plus the start. Isaac Sim's
first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

The layout is fixed (`object_jitter=0`); the scripted demo and the scorer are deterministic up to PhysX's own
run-to-run variation.

## Building it for real

- Lighter: a BIC Multi-Purpose Classic or any common utility lighter with a latch pressed by the thumb and a trigger
  pulled by the index finger. Lay it on its side on two pencil marks at the position above.
- Candle: any standard glass jar candle about 8.5 cm across (for example a 14 to 18 oz jar), burned down until the wax
  is about 4.5 cm below the rim, wick trimmed to 8 mm. Mark the jar's base outline on the tray.
- Safety: a non-flammable tray, a lid or snuffer and a small extinguisher at hand; nothing else within 30 cm of the
  jar. With an emptied lighter (see above) nothing burns at all.

[bic]: https://www.target.com/p/bic-multi-purpose-lighter-1-pack/-/A-14695758
